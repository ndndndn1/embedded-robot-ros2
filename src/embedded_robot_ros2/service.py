from __future__ import annotations

import asyncio
import hashlib
import json
from collections import OrderedDict
from dataclasses import dataclass
from datetime import timedelta

from .metrics import Metrics
from .models import (
    TERMINAL_STATUSES,
    CommandRecord,
    CommandRequest,
    CommandStatus,
    CommandTransition,
    CommandType,
    HardwareSafetyState,
    JointState,
    ManipulateAction,
    NavigateAction,
    Pose2D,
    ProductProfile,
    ProtectiveStopAction,
    RobotState,
    RosSafetyLevel,
    RosSafetyState,
    now_utc,
)
from .ports import ActionUpdate, NavigateGoal, RosTransport, TrajectoryGoal
from .profiles import PRODUCTS, PRODUCTS_BY_ID, ROBOTS


class AdapterError(Exception):
    status_code = 400
    code = "adapter_error"

    def __init__(self, message: str, *, code: str | None = None) -> None:
        super().__init__(message)
        self.message = message
        if code is not None:
            self.code = code


class NotFoundError(AdapterError):
    status_code = 404
    code = "not_found"


class ConflictError(AdapterError):
    status_code = 409
    code = "conflict"


class UnavailableError(AdapterError):
    status_code = 503
    code = "transport_unavailable"


@dataclass(slots=True)
class _CommandRecord:
    request_hash: str
    request: CommandRequest
    status: CommandStatus
    transitions: list[CommandTransition]
    error_code: str | None = None
    ros_sequence: int = -1
    timer: asyncio.Task[None] | None = None


class RobotEdgeService:
    def __init__(
        self,
        transport: RosTransport,
        metrics: Metrics | None = None,
        *,
        max_command_records: int = 10_000,
    ) -> None:
        if max_command_records < 1:
            raise ValueError("max_command_records must be positive")
        self.transport = transport
        self.metrics = metrics or Metrics()
        self._commands: OrderedDict[str, _CommandRecord] = OrderedDict()
        self._max_command_records = max_command_records
        self._joint_sequences: dict[str, int] = dict.fromkeys(ROBOTS, -1)
        self._safety_sequences: dict[str, int] = dict.fromkeys(ROBOTS, -1)
        observed_at = now_utc()
        self._states = {
            robot_id: RobotState(
                robot_id=robot_id,
                product_id=profile.product_id,
                state_version=0,
                observed_at=observed_at,
                pose=Pose2D(x_m=0, y_m=0, yaw_rad=0),
                joint_positions_rad=tuple(0.0 for _ in profile.joints),
                hardware_safety_state=HardwareSafetyState.NORMAL,
                software_protective_stop=False,
            )
            for robot_id, profile in ROBOTS.items()
        }
        self._lock = asyncio.Lock()

    async def start(self) -> None:
        await self.transport.connect(self._on_action, self._on_joint, self._on_safety)
        self.metrics.increment("transport_connect", outcome="success")

    async def close(self) -> None:
        timers = [record.timer for record in self._commands.values() if record.timer]
        for timer in timers:
            timer.cancel()
        if timers:
            await asyncio.gather(*timers, return_exceptions=True)
        await self.transport.close()

    async def reconnect(self) -> None:
        await self.transport.connect(self._on_action, self._on_joint, self._on_safety)
        self.metrics.increment("transport_reconnect", outcome="success")

    def products(self) -> tuple[ProductProfile, ...]:
        return PRODUCTS

    def states(self) -> tuple[RobotState, ...]:
        return tuple(self._states.values())

    def state(self, robot_id: str) -> RobotState:
        try:
            return self._states[robot_id]
        except KeyError as exc:
            raise NotFoundError(f"robot {robot_id}") from exc

    async def submit(self, request: CommandRequest) -> CommandRecord:
        digest = self._request_hash(request)
        async with self._lock:
            existing = self._commands.get(request.command_id)
            if existing:
                if existing.request_hash != digest:
                    raise ConflictError(
                        "command_id already exists with different content",
                        code="idempotency_conflict",
                    )
                self.metrics.increment("command_duplicate", kind=request.action.type.value)
                return self._snapshot(existing)
            if request.robot_id not in self._states:
                raise NotFoundError(f"robot {request.robot_id}")
            if not self.transport.connected:
                raise UnavailableError("ROS graph is disconnected")
            self._reserve_command_record()
            record = _CommandRecord(
                request_hash=digest,
                request=request,
                status=CommandStatus.SUBMITTED,
                transitions=[self._transition(CommandStatus.SUBMITTED)],
            )
            self._commands[request.command_id] = record
            rejection = self._validate_submission(request)
            if rejection:
                code, detail = rejection
                self._set_status(record, CommandStatus.REJECTED, detail, code)
                self.metrics.increment(
                    "command_submit", kind=request.action.type.value, outcome="rejected"
                )
                return self._snapshot(record)
            self._set_status(record, CommandStatus.ACCEPTED)
            self._update_state(request.robot_id, active_command_id=request.command_id)

        try:
            action = request.action
            if isinstance(action, NavigateAction):
                await self.transport.navigate(
                    NavigateGoal(
                        request.command_id,
                        request.robot_id,
                        action.target,
                        action.max_speed_mps,
                    )
                )
            elif isinstance(action, ManipulateAction):
                profile = ROBOTS[request.robot_id]
                await self.transport.follow_trajectory(
                    TrajectoryGoal(
                        request.command_id,
                        request.robot_id,
                        profile.joints,
                        action.joint_positions_rad,
                        action.max_force_n,
                    )
                )
            elif isinstance(action, ProtectiveStopAction):
                await self.transport.software_protective_stop(request.robot_id, action.reason)
                await self._on_action(
                    ActionUpdate(
                        request.command_id,
                        1,
                        CommandStatus.COMPLETED,
                        "software protective stop requested; not a certified E-stop",
                    )
                )
            if record.status not in TERMINAL_STATUSES:
                delay = max(0.0, (request.expires_at - now_utc()).total_seconds())
                record.timer = asyncio.create_task(self._timeout(request, delay))
            self.metrics.increment(
                "command_submit", kind=request.action.type.value, outcome="accepted"
            )
            return self._snapshot(record)
        except ConnectionError as exc:
            async with self._lock:
                self._set_status(
                    record,
                    CommandStatus.FAILED,
                    "ROS graph disconnected during dispatch",
                    "transport_disconnected",
                )
                self._release_robot(request.robot_id, request.command_id)
            self.metrics.increment(
                "command_submit", kind=request.action.type.value, outcome="disconnected"
            )
            raise UnavailableError(str(exc)) from exc

    async def cancel(self, command_id: str) -> CommandRecord:
        record = self._find_command(command_id)
        if record.status in TERMINAL_STATUSES:
            raise ConflictError(
                f"cannot cancel command in {record.status} state", code="command_terminal"
            )
        await self.transport.cancel(record.request.robot_id, command_id)
        self.metrics.increment("command_cancel", outcome="requested")
        return self._snapshot(record)

    def command(self, command_id: str) -> CommandRecord:
        return self._snapshot(self._find_command(command_id))

    def active_commands(self) -> int:
        return sum(record.status not in TERMINAL_STATUSES for record in self._commands.values())

    async def _on_action(self, update: ActionUpdate) -> None:
        async with self._lock:
            record = self._commands.get(update.command_id)
            if record is None:
                self.metrics.increment("status_drop", reason="unknown_command")
                return
            if update.sequence <= record.ros_sequence or record.status in TERMINAL_STATUSES:
                self.metrics.increment("status_drop", reason="duplicate_or_stale")
                return
            record.ros_sequence = update.sequence
            self._set_status(record, update.phase, update.message, update.error_code)
            if update.phase in TERMINAL_STATUSES:
                self._finish_state(record)
                if record.timer:
                    if record.timer is not asyncio.current_task():
                        record.timer.cancel()
                    record.timer = None
            self.metrics.increment("status_update", phase=update.phase.value)

    async def _on_joint(self, observation: JointState) -> None:
        if observation.robot_id not in ROBOTS:
            self.metrics.increment("joint_drop", reason="unknown_robot")
            return
        if observation.sequence <= self._joint_sequences[observation.robot_id]:
            self.metrics.increment("joint_drop", reason="duplicate_or_stale")
            return
        expected = len(ROBOTS[observation.robot_id].joints)
        if len(observation.positions) != expected:
            self.metrics.increment("joint_drop", reason="invalid_joint_count")
            return
        self._joint_sequences[observation.robot_id] = observation.sequence
        self._update_state(
            observation.robot_id,
            joint_positions_rad=tuple(observation.positions),
            observed_at=observation.stamp,
        )

    async def _on_safety(self, observation: RosSafetyState) -> None:
        if observation.robot_id not in ROBOTS:
            self.metrics.increment("safety_drop", reason="unknown_robot")
            return
        if observation.sequence <= self._safety_sequences[observation.robot_id]:
            self.metrics.increment("safety_drop", reason="duplicate_or_stale")
            return
        self._safety_sequences[observation.robot_id] = observation.sequence
        if observation.level is RosSafetyLevel.DISCONNECTED:
            command_id = self._states[observation.robot_id].active_command_id
            if command_id:
                record = self._commands[command_id]
                await self._on_action(
                    ActionUpdate(
                        command_id,
                        record.ros_sequence + 1,
                        CommandStatus.FAILED,
                        "ROS graph disconnected; completion is unknown",
                        "transport_disconnected",
                    )
                )
            return
        self._update_state(
            observation.robot_id,
            hardware_safety_state=(
                HardwareSafetyState.ESTOP_ENGAGED
                if observation.level is RosSafetyLevel.ESTOP_ENGAGED
                else HardwareSafetyState.NORMAL
            ),
            observed_at=observation.stamp,
        )

    async def _timeout(self, request: CommandRequest, delay: float) -> None:
        try:
            await asyncio.sleep(delay)
            record = self._commands.get(request.command_id)
            if record and record.status not in TERMINAL_STATUSES:
                await self._on_action(
                    ActionUpdate(
                        request.command_id,
                        record.ros_sequence + 1,
                        CommandStatus.FAILED,
                        "command expired during execution",
                        "expired_during_execution",
                    )
                )
                await self.transport.cancel(request.robot_id, request.command_id)
                self.metrics.increment("command_timeout", kind=request.action.type.value)
        except asyncio.CancelledError:
            return

    def _validate_submission(self, request: CommandRequest) -> tuple[str, str] | None:
        now = now_utc()
        state = self._states[request.robot_id]
        product = PRODUCTS_BY_ID[state.product_id]
        if request.expires_at <= now:
            return ("expired_command", "command expired before acceptance")
        if request.issued_at > now + timedelta(minutes=5):
            return ("future_command", "issued_at is more than five minutes in the future")
        if request.expected_state_version != state.state_version:
            return ("stale_state", "expected_state_version does not match current state")
        if request.action.type not in product.capabilities:
            return ("unsupported_capability", "product does not support this command type")
        if state.active_command_id is not None:
            return ("robot_busy", "robot already has an active command")
        if (
            state.hardware_safety_state is HardwareSafetyState.ESTOP_ENGAGED
            and request.action.type is not CommandType.PROTECTIVE_STOP
        ):
            return ("hardware_estop_engaged", "motion is blocked by the hardware E-stop input")
        if (
            state.software_protective_stop
            and request.action.type is not CommandType.PROTECTIVE_STOP
        ):
            return ("software_protective_stop", "motion is blocked by software protective stop")
        if isinstance(request.action, ManipulateAction) and (
            len(request.action.joint_positions_rad) != product.joint_count
        ):
            return ("invalid_joint_count", "joint target does not match product profile")
        return None

    def _finish_state(self, record: _CommandRecord) -> None:
        robot_id = record.request.robot_id
        updates: dict[str, object] = {"active_command_id": None}
        if record.status is CommandStatus.COMPLETED:
            action = record.request.action
            if isinstance(action, NavigateAction):
                updates["pose"] = action.target
            elif isinstance(action, ManipulateAction):
                updates["joint_positions_rad"] = action.joint_positions_rad
            elif isinstance(action, ProtectiveStopAction):
                updates["software_protective_stop"] = True
        self._update_state(robot_id, **updates)

    def _release_robot(self, robot_id: str, command_id: str) -> None:
        if self._states[robot_id].active_command_id == command_id:
            self._update_state(robot_id, active_command_id=None)

    def _update_state(self, robot_id: str, **updates: object) -> None:
        state = self._states[robot_id]
        updates.setdefault("observed_at", now_utc())
        updates["state_version"] = state.state_version + 1
        self._states[robot_id] = state.model_copy(update=updates)

    def _set_status(
        self,
        record: _CommandRecord,
        status: CommandStatus,
        detail: str | None = None,
        error_code: str | None = None,
    ) -> None:
        record.status = status
        record.error_code = error_code
        record.transitions.append(self._transition(status, detail))

    @staticmethod
    def _transition(status: CommandStatus, detail: str | None = None) -> CommandTransition:
        return CommandTransition(status=status, occurred_at=now_utc(), detail=detail)

    @staticmethod
    def _snapshot(record: _CommandRecord) -> CommandRecord:
        return CommandRecord(
            request=record.request,
            status=record.status,
            transitions=tuple(record.transitions),
            updated_at=record.transitions[-1].occurred_at,
            error_code=record.error_code,
        )

    def _find_command(self, command_id: str) -> _CommandRecord:
        try:
            return self._commands[command_id]
        except KeyError as exc:
            raise NotFoundError(f"command {command_id}") from exc

    def _reserve_command_record(self) -> None:
        if len(self._commands) < self._max_command_records:
            return
        for command_id, record in self._commands.items():
            if record.status in TERMINAL_STATUSES:
                del self._commands[command_id]
                self.metrics.increment("command_evict", reason="bounded_history")
                return
        raise UnavailableError("command history capacity reached with active commands")

    @staticmethod
    def _request_hash(request: CommandRequest) -> str:
        canonical = json.dumps(
            request.model_dump(mode="json"), sort_keys=True, separators=(",", ":")
        )
        return hashlib.sha256(canonical.encode()).hexdigest()
