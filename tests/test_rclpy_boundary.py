from typing import Any

from embedded_robot_ros2.rclpy_adapter import RclpyTransport


class RecordingRuntime:
    def __init__(self) -> None:
        self.connection: dict[str, Any] = {}
        self.disconnected = False

    async def connect(self, **kwargs: Any) -> None:
        self.connection = kwargs

    async def disconnect(self) -> None:
        self.disconnected = True

    async def navigate(self, goal: Any) -> None:
        self.goal = goal

    async def follow_trajectory(self, goal: Any) -> None:
        self.goal = goal

    async def cancel(self, robot_id: str, command_id: str) -> None:
        self.cancelled = (robot_id, command_id)

    async def software_protective_stop(self, robot_id: str, reason: str) -> None:
        self.stopped = (robot_id, reason)


async def no_op(*args: Any) -> None:
    del args


async def test_rclpy_boundary_binds_exact_names_and_domain() -> None:
    runtime = RecordingRuntime()
    transport = RclpyTransport(runtime, namespace="/MM-01", domain_id=42)
    await transport.connect(no_op, no_op, no_op)
    assert runtime.connection["joint_topic"] == "/MM-01/joint_states"
    assert runtime.connection["navigation_action"] == "/MM-01/navigate_to_pose"
    assert runtime.connection["trajectory_action"] == "/MM-01/follow_joint_trajectory"
    assert runtime.connection["safety_topic"] == "/MM-01/safety_state"
    assert runtime.connection["domain_id"] == 42
    await transport.close()
    assert runtime.disconnected

