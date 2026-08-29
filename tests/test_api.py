import asyncio

from httpx import ASGITransport, AsyncClient

from embedded_robot_ros2.app import create_app
from embedded_robot_ros2.mock_graph import MockRosGraph
from embedded_robot_ros2.service import RobotEdgeService


async def test_http_contract_health_command_state_and_metrics() -> None:
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
                "contract_version": "physical-robot-command.v1",
            }
            state = (await client.get("/v1/robots/MM-01/state")).json()
            response = await client.post(
                "/v1/commands",
                json={
                    "command_id": "api-nav",
                    "robot_id": "MM-01",
                    "kind": "navigate",
                    "ttl_ms": 500,
                    "expected_state_version": state["state_version"],
                    "payload": {
                        "pose": {"frame_id": "map", "x": 1, "y": 2, "yaw": 0.0}
                    },
                },
            )
            assert response.status_code == 202
            await asyncio.sleep(0.03)
            assert (await client.get("/v1/commands/api-nav")).json()["phase"] == "completed"
            metrics = await client.get("/metrics")
            assert metrics.status_code == 200
            assert "robot_adapter_events_total" in metrics.text


async def test_http_returns_conflict_for_stale_version() -> None:
    service = RobotEdgeService(MockRosGraph())
    app = create_app(service)
    async with app.router.lifespan_context(app):
        async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as client:
            response = await client.post(
                "/v1/commands",
                json={
                    "command_id": "stale",
                    "robot_id": "MM-01",
                    "kind": "navigate",
                    "ttl_ms": 500,
                    "expected_state_version": 0,
                    "payload": {
                        "pose": {"frame_id": "map", "x": 1, "y": 2, "yaw": 0.0}
                    },
                },
            )
            assert response.status_code == 409

