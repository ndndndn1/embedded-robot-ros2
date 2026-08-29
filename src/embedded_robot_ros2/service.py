from __future__ import annotations

import asyncio
import hashlib
import json
from dataclasses import dataclass

from .metrics import Metrics
from .models import (
    TERMINAL_PHASES,
    CommandKind,
    CommandPhase,
    CommandRequest,
    CommandStatus,
    JointState,
    ManipulatePayload,
    NavigatePayload,
    ProtectiveStopPayload,
    RobotState,
    SafetyLevel,
    SafetyState,
    now_utc,
)
from .ports import ActionUpdate, NavigateGoal, RosTransport, TrajectoryGoal
from .profiles import PROFILES


class AdapterError(Exception):
    status_code = 400


class NotFoundError(AdapterError):
    status_code = 404


class ConflictError(AdapterError):
    status_code = 409


class UnavailableError(AdapterError):
    status_code = 503


@dataclass(slots=True)
class _CommandRecord:
    request_hash: str
    status: CommandStatus
    timer: asyncio.Task[None] | None = None


class RobotEdgeService:
    def __init__(self, transport: RosTransport, metrics: Metrics | None = None) -> None:
        self.transport = transport
        self.metrics = metrics or Metrics()
        self._commands: dict[str, _CommandRecord] = {}
        self._joints: dict[str, JointState] = {}
        self._safety: dict[str, SafetyState] = {}
        self._state_versions = dict.fromkeys(PROFILES, 0)
        self._active: dict[str, str | None] = dict.fromkeys(PROFILES)
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

    async def submit(self, request: CommandRequest) -> CommandStatus:
        try:
            payload = request.parsed_payload()
        except ValueError as exc:
            raise AdapterError(str(exc)) from exc
        digest = self._request_hash(request)
        previous_active: str | None = None
        async with self._lock:
            existing = self._commands.get(request.command_id)
            if existing:
                if existing.request_hash != digest:
                    raise ConflictError("command_id already exists with different content")
                self.metrics.increment("command_duplicate", kind=request.kind.value)
                return existing.status.model_copy(deep=True)
            self._validate_request(request)
            status = CommandStatus(
                command_id=request.command_id,
                robot_id=request.robot_id,
                phase=CommandPhase.ACCEPTED,
                sequence=0,
                message="accepted by edge adapter",
                updated_at=now_utc(),
            )
            record = _CommandRecord(digest, status)
            self._commands[request.command_id] = record
            previous_active = self._active[request.robot_id]
            self._active[request.robot_id] = request.command_id

        try:
            if isinstance(payload, NavigatePayload):
                await self.transport.navigate(
                    NavigateGoal(request.command_id, request.robot_id, payload.pose)
                )
            elif isinstance(payload, ManipulatePayload):
                await self.transport.follow_trajectory(
                    TrajectoryGoal(
                        request.command_id,
                        request.robot_id,
                        tuple(payload.joint_names),
                        tuple(payload.points),
                    )
                )
            elif isinstance(payload, ProtectiveStopPayload):
                if previous_active:
                    await self.transport.cancel(request.robot_id, previous_active)
                await self.transport.software_protective_stop(request.robot_id, payload.reason)
                await self._on_action(
                    ActionUpdate(
                        request.command_id,
                        1,
                        CommandPhase.COMPLETED,
                        "software protective stop requested; not a certified E-stop",
                    )
                )
            if record.status.phase not in TERMINAL_PHASES:
                record.timer = asyncio.create_task(self._timeout(request))
            self.metrics.increment("command_submit", kind=request.kind.value, outcome="accepted")
            return self._commands[request.command_id].status.model_copy(deep=True)
        except ConnectionError as exc:
            async with self._lock:
                del self._commands[request.command_id]
                self._active[request.robot_id] = None
            self.metrics.increment(
                "command_submit", kind=request.kind.value, outcome="disconnected"
            )
            raise UnavailableError(str(exc)) from exc

    async def cancel(self, command_id: str) -> CommandStatus:
        record = self._commands.get(command_id)
        if not record:
            raise NotFoundError("command not found")
        if record.status.phase in TERMINAL_PHASES:
            return record.status.model_copy(deep=True)
        await self.transport.cancel(record.status.robot_id, command_id)
        self.metrics.increment("command_cancel", outcome="requested")
        return self._commands[command_id].status.model_copy(deep=True)

    def command(self, command_id: str) -> CommandStatus:
        record = self._commands.get(command_id)
        if not record:
            raise NotFoundError("command not found")
        return record.status.model_copy(deep=True)

    def state(self, robot_id: str) -> RobotState:
        if robot_id not in PROFILES:
            raise NotFoundError("robot not found")
        safety = self._safety.get(robot_id)
        if safety is None:
            safety = SafetyState(
                robot_id=robot_id,
                sequence=0,
                level=SafetyLevel.DISCONNECTED,
                reason="no ROS safety state received",
                stamp=now_utc(),
            )
        return RobotState(
            robot_id=robot_id,
            profile=robot_id,  # type: ignore[arg-type]
            state_version=self._state_versions[robot_id],
            connected=self.transport.connected,
            safety=safety,
            joints=self._joints.get(robot_id),
            active_command_id=self._active[robot_id],
        )

    def active_commands(self) -> int:
        return sum(record.status.phase not in TERMINAL_PHASES for record in self._commands.values())

    async def _on_action(self, update: ActionUpdate) -> None:
        async with self._lock:
            record = self._commands.get(update.command_id)
            if record is None:
                self.metrics.increment("status_drop", reason="unknown_command")
                return
            if update.sequence <= record.status.sequence or record.status.phase in TERMINAL_PHASES:
                self.metrics.increment("status_drop", reason="duplicate_or_stale")
                return
            record.status = CommandStatus(
                command_id=record.status.command_id,
                robot_id=record.status.robot_id,
                phase=update.phase,
                sequence=update.sequence,
                message=update.message,
                updated_at=now_utc(),
            )
            if update.phase in TERMINAL_PHASES:
                self._active[record.status.robot_id] = None
                if record.timer:
                    record.timer.cancel()
                    record.timer = None
            self.metrics.increment("status_update", phase=update.phase.value)

    async def _on_joint(self, state: JointState) -> None:
        previous = self._joints.get(state.robot_id)
        if previous and state.sequence <= previous.sequence:
            self.metrics.increment("joint_drop", reason="duplicate_or_stale")
            return
        if state.robot_id not in PROFILES:
            self.metrics.increment("joint_drop", reason="unknown_robot")
            return
        self._joints[state.robot_id] = state
        self._state_versions[state.robot_id] += 1

    async def _on_safety(self, state: SafetyState) -> None:
        previous = self._safety.get(state.robot_id)
        if previous and state.sequence <= previous.sequence:
            self.metrics.increment("safety_drop", reason="duplicate_or_stale")
            return
        if state.robot_id not in PROFILES:
            self.metrics.increment("safety_drop", reason="unknown_robot")
            return
        self._safety[state.robot_id] = state
        self._state_versions[state.robot_id] += 1
        if state.level is SafetyLevel.DISCONNECTED:
            command_id = self._active[state.robot_id]
            if command_id:
                record = self._commands[command_id]
                await self._on_action(
                    ActionUpdate(
                        command_id,
                        record.status.sequence + 1,
                        CommandPhase.FAILED,
                        "ROS graph disconnected; completion is unknown",
                    )
                )

    async def _timeout(self, request: CommandRequest) -> None:
        try:
            await asyncio.sleep(request.ttl_ms / 1000)
            record = self._commands.get(request.command_id)
            if record and record.status.phase not in TERMINAL_PHASES:
                await self.transport.cancel(request.robot_id, request.command_id)
                await self._on_action(
                    ActionUpdate(
                        request.command_id,
                        record.status.sequence + 1,
                        CommandPhase.CANCELLED,
                        "command TTL expired; cancellation requested",
                    )
                )
                self.metrics.increment("command_timeout", kind=request.kind.value)
        except asyncio.CancelledError:
            return

    def _validate_request(self, request: CommandRequest) -> None:
        if request.robot_id not in PROFILES:
            raise NotFoundError("robot not found")
        if not self.transport.connected:
            raise UnavailableError("ROS graph is disconnected")
        if request.kind not in PROFILES[request.robot_id].capabilities:
            raise AdapterError(f"{request.kind.value} is not supported by {request.robot_id}")
        if request.expected_state_version != self._state_versions[request.robot_id]:
            raise ConflictError(
                f"state version mismatch: expected {request.expected_state_version}, "
                f"current {self._state_versions[request.robot_id]}"
            )
        active = self._active[request.robot_id]
        if active and request.kind is not CommandKind.PROTECTIVE_STOP:
            raise ConflictError(f"robot already has active command {active}")

    @staticmethod
    def _request_hash(request: CommandRequest) -> str:
        canonical = json.dumps(
            request.model_dump(mode="json"), sort_keys=True, separators=(",", ":")
        )
        return hashlib.sha256(canonical.encode()).hexdigest()
