from __future__ import annotations

from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from typing import Protocol

from .models import CommandStatus, JointState, Pose2D, RosSafetyState


@dataclass(frozen=True, slots=True)
class NavigateGoal:
    command_id: str
    robot_id: str
    pose: Pose2D
    max_speed_mps: float


@dataclass(frozen=True, slots=True)
class TrajectoryGoal:
    command_id: str
    robot_id: str
    joint_names: tuple[str, ...]
    joint_positions_rad: tuple[float, ...]
    max_force_n: float


@dataclass(frozen=True, slots=True)
class ActionUpdate:
    command_id: str
    sequence: int
    phase: CommandStatus
    message: str
    error_code: str | None = None


ActionCallback = Callable[[ActionUpdate], Awaitable[None]]
JointCallback = Callable[[JointState], Awaitable[None]]
SafetyCallback = Callable[[RosSafetyState], Awaitable[None]]


class RosTransport(Protocol):
    @property
    def connected(self) -> bool: ...

    async def connect(
        self,
        on_action: ActionCallback,
        on_joint: JointCallback,
        on_safety: SafetyCallback,
    ) -> None: ...

    async def disconnect(self) -> None: ...

    async def navigate(self, goal: NavigateGoal) -> None: ...

    async def follow_trajectory(self, goal: TrajectoryGoal) -> None: ...

    async def cancel(self, robot_id: str, command_id: str) -> None: ...

    async def software_protective_stop(self, robot_id: str, reason: str) -> None: ...

    async def close(self) -> None: ...
