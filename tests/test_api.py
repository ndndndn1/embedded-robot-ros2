import asyncio
from datetime import timedelta

from httpx import ASGITransport, AsyncClient

from embedded_robot_ros2.app import create_app
from embedded_robot_ros2.mock_graph import MockRosGraph
from embedded_robot_ros2.models import now_utc
from embedded_robot_ros2.service import RobotEdgeService


def request_json(command_id: str, state_version: int) -> dict[str, object]:
    now = now_utc()
    return {
        "contract_version": "1.0.0",
        "command_id": command_id,
        "robot_id": "mm-01-a",
        "issued_at": now.isoformat(),
        "expires_at": (now + timedelta(seconds=1)).isoformat(),
        "expected_state_version": state_version,
        "action": {
            "type": "navigate",
            "target": {"frame": "map", "x_m": 1, "y_m": 2, "yaw_rad": 0},
            "max_speed_mps": 0.5,
        },
    }


async def test_canonical_catalog_state_command_and_metrics_routes() -> None:
    service = RobotEdgeService(MockRosGraph())
    app = create_app(service)
    async with app.router.lifespan_context(app):
        transport = ASGITransport(app=app)
        async with AsyncClient(transport=transport, base_url="http://test") as client:
            health = await client.get("/healthz")
            assert health.json() == {
                "status": "ok",
                "transport": "connected",
                "robots": 2,
                "active_commands": 0,
                "contract_version": "1.0.0",
            }
            products = (await client.get("/v1/products")).json()
            assert {item["product_id"] for item in products} == {
                "mock-humanoid-mh-01",
                "mock-mobile-manipulator-mm-01",
            }
            robots = (await client.get("/v1/robots")).json()
            assert {item["robot_id"] for item in robots} == {"mh-01-a", "mm-01-a"}
            state_response = await client.get("/v1/robots/mm-01-a")
            assert set(state_response.json()) == {
                "contract_version",
                "robot_id",
                "product_id",
                "state_version",
                "observed_at",
                "pose",
                "joint_positions_rad",
                "hardware_safety_state",
                "software_protective_stop",
                "active_command_id",
            }
            state = state_response.json()
            response = await client.post(
                "/v1/commands", json=request_json("api-nav", state["state_version"])
            )
            assert response.status_code == 202
            body = response.json()
            assert set(body) == {
                "request",
                "status",
                "transitions",
                "updated_at",
                "error_code",
            }
            assert body["request"]["action"]["type"] == "navigate"
            await asyncio.sleep(0.03)
            assert (await client.get("/v1/commands/api-nav")).json()["status"] == "completed"
            assert "robot_adapter_events_total" in (await client.get("/metrics")).text


async def test_stale_state_is_canonical_rejected_record() -> None:
    service = RobotEdgeService(MockRosGraph())
    app = create_app(service)
    async with app.router.lifespan_context(app):
        async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
            response = await client.post("/v1/commands", json=request_json("stale", 0))
            assert response.status_code == 202
            assert response.json()["status"] == "rejected"
            assert response.json()["error_code"] == "stale_state"


async def test_validation_error_uses_canonical_error_wire() -> None:
    service = RobotEdgeService(MockRosGraph())
    app = create_app(service)
    async with app.router.lifespan_context(app):
        async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
            response = await client.post("/v1/commands", json={"wrong": True})
            assert response.status_code == 422
            assert set(response.json()) == {"code", "message"}
            assert response.json()["code"] == "validation_error"

