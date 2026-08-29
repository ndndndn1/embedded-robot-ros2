from __future__ import annotations

from typing import Protocol

from .ports import ActionCallback, JointCallback, NavigateGoal, SafetyCallback, TrajectoryGoal


class RosJazzyRuntime(Protocol):
    """Deployment-owned rclpy executor and generated-message bridge."""

    async def connect(
        self,
        *,
        namespace: str,
        domain_id: int,
        joint_topic: str,
        navigation_action: str,
        trajectory_action: str,
        safety_topic: str,
        on_action: ActionCallback,
        on_joint: JointCallback,
        on_safety: SafetyCallback,
    ) -> None: ...

    async def disconnect(self) -> None: ...
    async def navigate(self, goal: NavigateGoal) -> None: ...
    async def follow_trajectory(self, goal: TrajectoryGoal) -> None: ...
    async def cancel(self, robot_id: str, command_id: str) -> None: ...
    async def software_protective_stop(self, robot_id: str, reason: str) -> None: ...


class RclpyTransport:
    """Functional adapter around a deployment's ROS 2 Jazzy runtime.

    The runtime owns rclpy initialization, executor threads, and generated custom
    SafetyState messages. Dependency injection keeps this package importable in CI;
    production never falls back to the mock when runtime construction fails.
    """

    def __init__(self, runtime: RosJazzyRuntime, *, namespace: str, domain_id: int) -> None:
        if not namespace.startswith("/"):
            raise ValueError("namespace must start with /")
        if not 0 <= domain_id <= 232:
            raise ValueError("ROS_DOMAIN_ID must be between 0 and 232")
        self._runtime = runtime
        self._namespace = namespace.rstrip("/")
        self._domain_id = domain_id
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    async def connect(
        self,
        on_action: ActionCallback,
        on_joint: JointCallback,
        on_safety: SafetyCallback,
    ) -> None:
        await self._runtime.connect(
            namespace=self._namespace,
            domain_id=self._domain_id,
            joint_topic=f"{self._namespace}/joint_states",
            navigation_action=f"{self._namespace}/navigate_to_pose",
            trajectory_action=f"{self._namespace}/follow_joint_trajectory",
            safety_topic=f"{self._namespace}/safety_state",
            on_action=on_action,
            on_joint=on_joint,
            on_safety=on_safety,
        )
        self._connected = True

    async def disconnect(self) -> None:
        await self._runtime.disconnect()
        self._connected = False

    async def navigate(self, goal: NavigateGoal) -> None:
        self._ensure_connected()
        await self._runtime.navigate(goal)

    async def follow_trajectory(self, goal: TrajectoryGoal) -> None:
        self._ensure_connected()
        await self._runtime.follow_trajectory(goal)

    async def cancel(self, robot_id: str, command_id: str) -> None:
        self._ensure_connected()
        await self._runtime.cancel(robot_id, command_id)

    async def software_protective_stop(self, robot_id: str, reason: str) -> None:
        self._ensure_connected()
        await self._runtime.software_protective_stop(robot_id, reason)

    async def close(self) -> None:
        if self._connected:
            await self.disconnect()

    def _ensure_connected(self) -> None:
        if not self._connected:
            raise ConnectionError("ROS graph is disconnected")

