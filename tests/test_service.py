import asyncio

import pytest

from embedded_robot_ros2.mock_graph import MockRosGraph
from embedded_robot_ros2.models import CommandPhase, CommandRequest
from embedded_robot_ros2.ports import ActionUpdate
from embedded_robot_ros2.service import (
    AdapterError,
    ConflictError,
    NotFoundError,
    RobotEdgeService,
    UnavailableError,
)


def command(
    command_id: str,
    *,
    robot_id: str = "MM-01",
    kind: str = "navigate",
    state_version: int = 2,
    ttl_ms: int = 500,
) -> CommandRequest:
    payloads: dict[str, dict[str, object]] = {
        "navigate": {"pose": {"frame_id": "map", "x": 1.0, "y": 2.0, "yaw": 0.2}},
        "manipulate": {
            "joint_names": [f"arm_joint_{index}" for index in range(1, 7)],
            "points": [{"positions": [0.1] * 6, "time_from_start_ms": 100}],
        },
        "protective_stop": {"reason": "operator requested guarded stop"},
    }
    return CommandRequest.model_validate(
        {
            "command_id": command_id,
            "robot_id": robot_id,
            "kind": kind,
            "ttl_ms": ttl_ms,
            "expected_state_version": state_version,
            "payload": payloads[kind],
        }
    )


@pytest.fixture
async def running() -> tuple[RobotEdgeService, MockRosGraph]:
    graph = MockRosGraph(action_delay_s=0.01)
    service = RobotEdgeService(graph)
    await service.start()
    yield service, graph
    await service.close()


async def test_profiles_are_normalized_on_connect(
    running: tuple[RobotEdgeService, MockRosGraph],
) -> None:
    service, _ = running
    mh = service.state("MH-01")
    mm = service.state("MM-01")
    assert mh.profile == "MH-01" and len(mh.joints.names) == 12  # type: ignore[union-attr]
    assert mm.profile == "MM-01" and len(mm.joints.names) == 6  # type: ignore[union-attr]
    assert mh.connected and mm.connected
    assert not mh.safety.certified_emergency_stop


async def test_navigation_maps_to_action_and_drops_duplicate_status(
    running: tuple[RobotEdgeService, MockRosGraph],
) -> None:
    service, _ = running
    accepted = await service.submit(command("nav-1"))
    assert accepted.phase in {CommandPhase.ACCEPTED, CommandPhase.RUNNING}
    await asyncio.sleep(0.03)
    completed = service.command("nav-1")
    assert completed.phase is CommandPhase.COMPLETED
    assert completed.sequence == 2
    assert 'reason="duplicate_or_stale"' in service.metrics.render()


async def test_idempotency_and_conflicting_reuse(
    running: tuple[RobotEdgeService, MockRosGraph],
) -> None:
    service, _ = running
    first = command("same-id")
    await service.submit(first)
    repeated = await service.submit(first)
    assert repeated.command_id == "same-id"
    conflicting = first.model_copy(update={"ttl_ms": 600})
    with pytest.raises(ConflictError):
        await service.submit(conflicting)


async def test_timeout_cancels_and_releases_robot() -> None:
    graph = MockRosGraph(action_delay_s=1)
    service = RobotEdgeService(graph)
    await service.start()
    try:
        await service.submit(command("slow", ttl_ms=50))
        await asyncio.sleep(0.08)
        assert service.command("slow").phase is CommandPhase.CANCELLED
        assert service.state("MM-01").active_command_id is None
    finally:
        await service.close()
    assert graph.live_tasks == 0


async def test_explicit_cancel_and_stale_update_are_safe(
    running: tuple[RobotEdgeService, MockRosGraph],
) -> None:
    service, graph = running
    await service.submit(command("cancel-me"))
    cancelled = await service.cancel("cancel-me")
    assert cancelled.phase is CommandPhase.CANCELLED
    await graph.emit_action(ActionUpdate("cancel-me", 1, CommandPhase.RUNNING, "late"))
    assert service.command("cancel-me").phase is CommandPhase.CANCELLED


async def test_disconnect_fails_closed_and_reconnect_recovers(
    running: tuple[RobotEdgeService, MockRosGraph],
) -> None:
    service, graph = running
    await graph.disconnect()
    assert service.state("MM-01").safety.level.value == "disconnected"
    with pytest.raises(UnavailableError):
        await service.submit(command("offline", state_version=3))
    await service.reconnect()
    version = service.state("MM-01").state_version
    result = await service.submit(command("online", state_version=version))
    assert result.command_id == "online"


async def test_disconnect_marks_inflight_unknown_and_old_epoch_cannot_complete() -> None:
    graph = MockRosGraph(action_delay_s=0.05)
    service = RobotEdgeService(graph)
    await service.start()
    try:
        await service.submit(command("cross-epoch"))
        await graph.disconnect()
        assert service.command("cross-epoch").phase is CommandPhase.FAILED
        await service.reconnect()
        await asyncio.sleep(0.07)
        assert service.command("cross-epoch").phase is CommandPhase.FAILED
    finally:
        await service.close()


async def test_profile_capability_and_state_version_are_enforced(
    running: tuple[RobotEdgeService, MockRosGraph],
) -> None:
    service, _ = running
    with pytest.raises(AdapterError, match="not supported"):
        await service.submit(command("mh-nav", robot_id="MH-01"))
    with pytest.raises(ConflictError, match="state version mismatch"):
        await service.submit(command("stale", state_version=0))


async def test_protective_stop_is_labeled_as_software_request(
    running: tuple[RobotEdgeService, MockRosGraph],
) -> None:
    service, _ = running
    result = await service.submit(command("stop", kind="protective_stop"))
    assert result.phase is CommandPhase.COMPLETED
    assert "not a certified E-stop" in result.message
    assert service.state("MM-01").safety.level.value == "protective_stop"


async def test_protective_stop_cancels_active_motion() -> None:
    graph = MockRosGraph(action_delay_s=1)
    service = RobotEdgeService(graph)
    await service.start()
    try:
        await service.submit(command("motion"))
        version = service.state("MM-01").state_version
        await service.submit(
            command("stop-motion", kind="protective_stop", state_version=version)
        )
        assert service.command("motion").phase is CommandPhase.CANCELLED
        assert service.command("stop-motion").phase is CommandPhase.COMPLETED
    finally:
        await service.close()


async def test_completed_history_is_bounded() -> None:
    graph = MockRosGraph(action_delay_s=0)
    service = RobotEdgeService(graph, max_command_records=2)
    await service.start()
    try:
        for index in range(3):
            await service.submit(command(f"bounded-{index}"))
            await asyncio.sleep(0)
            await asyncio.sleep(0)
        with pytest.raises(NotFoundError, match="command not found"):
            service.command("bounded-0")
        assert service.command("bounded-2").phase is CommandPhase.COMPLETED
        assert 'reason="bounded_history"' in service.metrics.render()
    finally:
        await service.close()
