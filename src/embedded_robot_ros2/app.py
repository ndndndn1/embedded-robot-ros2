from __future__ import annotations

from collections.abc import AsyncIterator
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse, PlainTextResponse

from .mock_graph import MockRosGraph
from .models import CommandRequest, CommandStatus, HealthResponse, RobotState
from .service import AdapterError, RobotEdgeService

CONTRACT_VERSION = "physical-robot-command.v1"


def create_app(service: RobotEdgeService | None = None) -> FastAPI:
    selected = service or RobotEdgeService(MockRosGraph())

    @asynccontextmanager
    async def lifespan(app: FastAPI) -> AsyncIterator[None]:
        app.state.service = selected
        await selected.start()
        try:
            yield
        finally:
            await selected.close()

    app = FastAPI(title="Embedded Robot ROS 2 Adapter", version="0.1.0", lifespan=lifespan)

    @app.exception_handler(AdapterError)
    async def adapter_error(_: Request, exc: AdapterError) -> JSONResponse:
        return JSONResponse(status_code=exc.status_code, content={"detail": str(exc)})

    @app.get("/healthz", response_model=HealthResponse)
    async def health(request: Request) -> HealthResponse:
        current: RobotEdgeService = request.app.state.service
        connected = current.transport.connected
        return HealthResponse(
            status="ok" if connected else "degraded",
            transport="connected" if connected else "disconnected",
            robots=2,
            active_commands=current.active_commands(),
            contract_version=CONTRACT_VERSION,
        )

    @app.get("/metrics", response_class=PlainTextResponse)
    async def metrics(request: Request) -> str:
        current: RobotEdgeService = request.app.state.service
        return current.metrics.render()

    @app.post("/v1/commands", response_model=CommandStatus, status_code=202)
    async def submit(command: CommandRequest, request: Request) -> CommandStatus:
        current: RobotEdgeService = request.app.state.service
        return await current.submit(command)

    @app.get("/v1/commands/{command_id}", response_model=CommandStatus)
    async def command(command_id: str, request: Request) -> CommandStatus:
        current: RobotEdgeService = request.app.state.service
        return current.command(command_id)

    @app.post("/v1/commands/{command_id}/cancel", response_model=CommandStatus)
    async def cancel(command_id: str, request: Request) -> CommandStatus:
        current: RobotEdgeService = request.app.state.service
        return await current.cancel(command_id)

    @app.get("/v1/robots/{robot_id}/state", response_model=RobotState)
    async def state(robot_id: str, request: Request) -> RobotState:
        current: RobotEdgeService = request.app.state.service
        return current.state(robot_id)

    return app


app = create_app()


def run() -> None:
    uvicorn.run("embedded_robot_ros2.app:app", host="127.0.0.1", port=8080)

