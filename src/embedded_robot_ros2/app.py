from __future__ import annotations

from collections.abc import AsyncIterator
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI, Request
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse, PlainTextResponse

from .mock_graph import MockRosGraph
from .models import (
    CommandRecord,
    CommandRequest,
    ErrorResponse,
    HealthResponse,
    ProductProfile,
    RobotState,
)
from .service import AdapterError, RobotEdgeService

CONTRACT_VERSION = "1.0.0"


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
        return JSONResponse(
            status_code=exc.status_code,
            content=ErrorResponse(code=exc.code, message=exc.message).model_dump(),
        )

    @app.exception_handler(RequestValidationError)
    async def validation_error(_: Request, exc: RequestValidationError) -> JSONResponse:
        locations = [".".join(str(item) for item in error["loc"]) for error in exc.errors()]
        return JSONResponse(
            status_code=422,
            content=ErrorResponse(
                code="validation_error", message=f"invalid fields: {', '.join(locations)}"
            ).model_dump(),
        )

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

    @app.get("/v1/products", response_model=list[ProductProfile])
    async def products(request: Request) -> tuple[ProductProfile, ...]:
        current: RobotEdgeService = request.app.state.service
        return current.products()

    @app.get("/v1/robots", response_model=list[RobotState])
    async def robots(request: Request) -> tuple[RobotState, ...]:
        current: RobotEdgeService = request.app.state.service
        return current.states()

    @app.post("/v1/commands", response_model=CommandRecord, status_code=202)
    async def submit(command: CommandRequest, request: Request) -> CommandRecord:
        current: RobotEdgeService = request.app.state.service
        return await current.submit(command)

    @app.get("/v1/commands/{command_id}", response_model=CommandRecord)
    async def command(command_id: str, request: Request) -> CommandRecord:
        current: RobotEdgeService = request.app.state.service
        return current.command(command_id)

    @app.post("/v1/commands/{command_id}/cancel", response_model=CommandRecord)
    async def cancel(command_id: str, request: Request) -> CommandRecord:
        current: RobotEdgeService = request.app.state.service
        return await current.cancel(command_id)

    @app.get("/v1/robots/{robot_id}", response_model=RobotState)
    async def state(robot_id: str, request: Request) -> RobotState:
        current: RobotEdgeService = request.app.state.service
        return current.state(robot_id)

    return app


app = create_app()


def run() -> None:
    uvicorn.run("embedded_robot_ros2.app:app", host="127.0.0.1", port=8080)
