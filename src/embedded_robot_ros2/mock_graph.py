from __future__ import annotations

import asyncio
from collections.abc import Coroutine
from datetime import UTC, datetime
from typing import Any

from .models import CommandStatus, JointState, RosSafetyLevel, RosSafetyState
from .ports import (
    ActionCallback,
    ActionUpdate,
    JointCallback,
    NavigateGoal,
    SafetyCallback,
    TrajectoryGoal,
)
from .profiles import ROBOTS


class MockRosGraph:
    """Deterministic in-process substitute for the ROS graph used in tests and demos."""

    def __init__(self, action_delay_s: float = 0.01) -> None:
        self._connected = False
        self._action_delay_s = action_delay_s
        self._action_callback: ActionCallback | None = None
        self._joint_callback: JointCallback | None = None
        self._safety_callback: SafetyCallback | None = None
        self._tasks: set[asyncio.Task[None]] = set()
        self._cancelled: set[str] = set()
        self._sequence = 0
        self._connection_epoch = 0

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def live_tasks(self) -> int:
        return sum(not task.done() for task in self._tasks)

    async def connect(
        self,
        on_action: ActionCallback,
        on_joint: JointCallback,
        on_safety: SafetyCallback,
    ) -> None:
        self._action_callback = on_action
        self._joint_callback = on_joint
        self._safety_callback = on_safety
        self._connection_epoch += 1
        self._connected = True
        for robot_id, profile in ROBOTS.items():
            self._sequence += 1
            await on_safety(
                RosSafetyState(
                    robot_id=robot_id,
                    sequence=self._sequence,
                    level=RosSafetyLevel.NORMAL,
                    reason="mock transport connected",
                    stamp=datetime.now(UTC),
                )
            )
            await on_joint(
                JointState(
                    robot_id=robot_id,
                    sequence=self._sequence,
                    names=list(profile.joints),
                    positions=[0.0] * len(profile.joints),
                    velocities=[0.0] * len(profile.joints),
                    stamp=datetime.now(UTC),
                )
            )

    async def disconnect(self) -> None:
        if not self._connected:
            return
        self._connected = False
        if self._safety_callback:
            for robot_id in ROBOTS:
                self._sequence += 1
                await self._safety_callback(
                    RosSafetyState(
                        robot_id=robot_id,
                        sequence=self._sequence,
                        level=RosSafetyLevel.DISCONNECTED,
                        reason="ROS graph disconnected",
                        stamp=datetime.now(UTC),
                    )
                )

    async def navigate(self, goal: NavigateGoal) -> None:
        self._ensure_connected()
        self._spawn(
            self._run_action(goal.command_id, "navigation completed", self._connection_epoch)
        )

    async def follow_trajectory(self, goal: TrajectoryGoal) -> None:
        self._ensure_connected()
        self._spawn(
            self._run_action(goal.command_id, "trajectory completed", self._connection_epoch)
        )

    async def cancel(self, robot_id: str, command_id: str) -> None:
        del robot_id
        self._cancelled.add(command_id)
        if self._action_callback:
            await self._action_callback(
                ActionUpdate(command_id, 99, CommandStatus.CANCELLED, "cancel acknowledged")
            )

    async def software_protective_stop(self, robot_id: str, reason: str) -> None:
        self._ensure_connected()
        if self._safety_callback:
            self._sequence += 1
            await self._safety_callback(
                RosSafetyState(
                    robot_id=robot_id,
                    sequence=self._sequence,
                    level=RosSafetyLevel.NORMAL,
                    reason=reason,
                    stamp=datetime.now(UTC),
                )
            )

    async def close(self) -> None:
        await self.disconnect()
        for task in tuple(self._tasks):
            task.cancel()
        if self._tasks:
            await asyncio.gather(*self._tasks, return_exceptions=True)
        self._tasks.clear()

    async def emit_action(self, update: ActionUpdate) -> None:
        if self._action_callback:
            await self._action_callback(update)

    async def _run_action(self, command_id: str, message: str, epoch: int) -> None:
        assert self._action_callback is not None
        await self._action_callback(ActionUpdate(command_id, 1, CommandStatus.RUNNING, "running"))
        await asyncio.sleep(self._action_delay_s)
        if (
            command_id not in self._cancelled
            and self._connected
            and epoch == self._connection_epoch
        ):
            update = ActionUpdate(command_id, 2, CommandStatus.COMPLETED, message)
            await self._action_callback(update)
            await self._action_callback(update)  # deterministic duplicate DDS delivery

    def _spawn(self, coro: Coroutine[Any, Any, None]) -> None:
        task = asyncio.create_task(coro)
        self._tasks.add(task)
        task.add_done_callback(self._tasks.discard)

    def _ensure_connected(self) -> None:
        if not self._connected:
            raise ConnectionError("ROS graph is disconnected")
