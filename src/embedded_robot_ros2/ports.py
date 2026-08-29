from __future__ import annotations

from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from typing import Protocol

from .models import CommandPhase, JointState, Pose, SafetyState, TrajectoryPoint


@dataclass(frozen=True, slots=True)
class NavigateGoal:
    command_id: str
    robot_id: str
    pose: Pose


@dataclass(frozen=True, slots=True)
class TrajectoryGoal:
    command_id: str
    robot_id: str
    joint_names: tuple[str, ...]
    points: tuple[TrajectoryPoint, ...]


@dataclass(frozen=True, slots=True)
class ActionUpdate:
    command_id: str
    sequence: int
    phase: CommandPhase
    message: str


ActionCallback = Callable[[ActionUpdate], Awaitable[None]]
JointCallback = Callable[[JointState], Awaitable[None]]
SafetyCallback = Callable[[SafetyState], Awaitable[None]]


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

