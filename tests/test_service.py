import asyncio
from datetime import timedelta

import pytest

from embedded_robot_ros2.mock_graph import MockRosGraph
from embedded_robot_ros2.models import CommandRequest, CommandStatus, now_utc
from embedded_robot_ros2.ports import ActionUpdate
from embedded_robot_ros2.service import (
    ConflictError,
    NotFoundError,
    RobotEdgeService,
    UnavailableError,
)


def command(
    command_id: str,
    *,
    robot_id: str = "mm-01-a",
    kind: str = "navigate",
    state_version: int = 2,
    expires_in_ms: int = 500,
) -> CommandRequest:
    now = now_utc()
    actions: dict[str, dict[str, object]] = {
        "navigate": {
            "type": "navigate",
            "target": {"frame": "map", "x_m": 1.0, "y_m": 2.0, "yaw_rad": 0.2},
            "max_speed_mps": 0.5,
        },
        "manipulate": {
            "type": "manipulate",
            "joint_positions_rad": [0.1] * (12 if robot_id == "mh-01-a" else 6),
            "max_force_n": 30,
        },
        "protective_stop": {
            "type": "protective_stop",
            "reason": "operator requested guarded stop",
        },
    }
    return CommandRequest.model_validate(
        {
            "contract_version": "1.0.0",
            "command_id": command_id,
            "robot_id": robot_id,
            "issued_at": now.isoformat(),
            "expires_at": (now + timedelta(milliseconds=expires_in_ms)).isoformat(),
            "expected_state_version": state_version,
            "action": actions[kind],
        }
    )


@pytest.fixture
async def running() -> tuple[RobotEdgeService, MockRosGraph]:
    graph = MockRosGraph(action_delay_s=0.01)
    service = RobotEdgeService(graph)
    await service.start()
    yield service, graph
    await service.close()


async def test_products_and_robot_states_match_canonical_profiles(
    running: tuple[RobotEdgeService, MockRosGraph],
) -> None:
    service, _ = running
    products = {product.product_id: product for product in service.products()}
    assert products["mock-humanoid-mh-01"].joint_count == 12
    assert products["mock-mobile-manipulator-mm-01"].joint_count == 6
    mh = service.state("mh-01-a")
    mm = service.state("mm-01-a")
    assert mh.contract_version == "1.0.0" and len(mh.joint_positions_rad) == 12
    assert mm.product_id == "mock-mobile-manipulator-mm-01"
    assert mm.hardware_safety_state.value == "normal"


async def test_navigation_maps_to_action_and_drops_duplicate_status(
    running: tuple[RobotEdgeService, MockRosGraph],
) -> None:
    service, _ = running
    accepted = await service.submit(command("nav-1"))
    assert accepted.status in {CommandStatus.ACCEPTED, CommandStatus.RUNNING}
    assert [item.status for item in accepted.transitions][:2] == [
        CommandStatus.SUBMITTED,
        CommandStatus.ACCEPTED,
    ]
    await asyncio.sleep(0.03)
    completed = service.command("nav-1")
    assert completed.status is CommandStatus.COMPLETED
    assert service.state("mm-01-a").pose.x_m == 1.0
    assert 'reason="duplicate_or_stale"' in service.metrics.render()


async def test_manipulation_updates_canonical_joint_state(
    running: tuple[RobotEdgeService, MockRosGraph],
) -> None:
    service, _ = running
    await service.submit(command("arm-1", robot_id="mh-01-a", kind="manipulate"))
    await asyncio.sleep(0.03)
    state = service.state("mh-01-a")
    assert state.joint_positions_rad == (0.1,) * 12


async def test_idempotency_and_conflicting_reuse(
    running: tuple[RobotEdgeService, MockRosGraph],
) -> None:
    service, _ = running
    first = command("same-id")
    await service.submit(first)
    repeated = await service.submit(first)
    assert repeated.request.command_id == "same-id"
    conflicting = first.model_copy(update={"expires_at": first.expires_at + timedelta(seconds=1)})
    with pytest.raises(ConflictError):
        await service.submit(conflicting)


async def test_stale_state_and_invalid_joint_count_return_rejected_record(
    running: tuple[RobotEdgeService, MockRosGraph],
) -> None:
    service, _ = running
    stale = await service.submit(command("stale", state_version=0))
    assert stale.status is CommandStatus.REJECTED
    assert stale.error_code == "stale_state"
    invalid = command("bad-joints", kind="manipulate")
    invalid = invalid.model_copy(
        update={
            "action": invalid.action.model_copy(update={"joint_positions_rad": (0.1,)})
        }
    )
    rejected = await service.submit(invalid)
    assert rejected.status is CommandStatus.REJECTED
    assert rejected.error_code == "invalid_joint_count"


async def test_expiration_cancels_transport_and_fails_record() -> None:
    graph = MockRosGraph(action_delay_s=1)
    service = RobotEdgeService(graph)
    await service.start()
    try:
        await service.submit(command("slow", expires_in_ms=50))
        await asyncio.sleep(0.08)
        record = service.command("slow")
        assert record.status is CommandStatus.FAILED
        assert record.error_code == "expired_during_execution"
        assert service.state("mm-01-a").active_command_id is None
    finally:
        await service.close()
    assert graph.live_tasks == 0


async def test_explicit_cancel_and_stale_update_are_safe(
    running: tuple[RobotEdgeService, MockRosGraph],
) -> None:
    service, graph = running
    await service.submit(command("cancel-me"))
    cancelled = await service.cancel("cancel-me")
    assert cancelled.status is CommandStatus.CANCELLED
    await graph.emit_action(ActionUpdate("cancel-me", 1, CommandStatus.RUNNING, "late"))
    assert service.command("cancel-me").status is CommandStatus.CANCELLED
    with pytest.raises(ConflictError, match="cannot cancel"):
        await service.cancel("cancel-me")


async def test_disconnect_fails_closed_and_reconnect_recovers(
    running: tuple[RobotEdgeService, MockRosGraph],
) -> None:
    service, graph = running
    await graph.disconnect()
    with pytest.raises(UnavailableError):
        await service.submit(command("offline", state_version=2))
    await service.reconnect()
    version = service.state("mm-01-a").state_version
    result = await service.submit(command("online", state_version=version))
    assert result.request.command_id == "online"


async def test_disconnect_marks_inflight_unknown_and_old_epoch_cannot_complete() -> None:
    graph = MockRosGraph(action_delay_s=0.05)
    service = RobotEdgeService(graph)
    await service.start()
    try:
        await service.submit(command("cross-epoch"))
        await graph.disconnect()
        assert service.command("cross-epoch").status is CommandStatus.FAILED
        await service.reconnect()
        await asyncio.sleep(0.07)
        assert service.command("cross-epoch").status is CommandStatus.FAILED
    finally:
        await service.close()

async def test_protective_stop_is_software_state_not_hardware_estop(
    running: tuple[RobotEdgeService, MockRosGraph],
) -> None:
    service, _ = running
    result = await service.submit(command("stop", kind="protective_stop"))
    assert result.status is CommandStatus.COMPLETED
    assert "not a certified E-stop" in result.transitions[-1].detail
    state = service.state("mm-01-a")
    assert state.software_protective_stop
    assert state.hardware_safety_state.value == "normal"


async def test_completed_history_is_bounded() -> None:
    graph = MockRosGraph(action_delay_s=0)
    service = RobotEdgeService(graph, max_command_records=2)
    await service.start()
    try:
        for index in range(3):
            version = service.state("mm-01-a").state_version
            await service.submit(command(f"bounded-{index}", state_version=version))
            await asyncio.sleep(0)
            await asyncio.sleep(0)
        with pytest.raises(NotFoundError, match="command bounded-0"):
            service.command("bounded-0")
        assert service.command("bounded-2").status is CommandStatus.COMPLETED
        assert 'reason="bounded_history"' in service.metrics.render()
    finally:
        await service.close()
