from __future__ import annotations

from datetime import UTC, datetime
from enum import StrEnum
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator


class CommandKind(StrEnum):
    NAVIGATE = "navigate"
    MANIPULATE = "manipulate"
    PROTECTIVE_STOP = "protective_stop"


class CommandPhase(StrEnum):
    ACCEPTED = "accepted"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


TERMINAL_PHASES = {CommandPhase.COMPLETED, CommandPhase.FAILED, CommandPhase.CANCELLED}


class Pose(BaseModel):
    model_config = ConfigDict(extra="forbid")

    frame_id: str = Field(min_length=1, max_length=128)
    x: float = Field(ge=-10_000, le=10_000)
    y: float = Field(ge=-10_000, le=10_000)
    yaw: float = Field(ge=-3.141593, le=3.141593)


class TrajectoryPoint(BaseModel):
    model_config = ConfigDict(extra="forbid")

    positions: list[float] = Field(min_length=1, max_length=64)
    time_from_start_ms: int = Field(ge=1, le=300_000)


class NavigatePayload(BaseModel):
    model_config = ConfigDict(extra="forbid")

    pose: Pose


class ManipulatePayload(BaseModel):
    model_config = ConfigDict(extra="forbid")

    joint_names: list[str] = Field(min_length=1, max_length=64)
    points: list[TrajectoryPoint] = Field(min_length=1, max_length=10_000)

    @model_validator(mode="after")
    def validate_points(self) -> ManipulatePayload:
        expected = len(self.joint_names)
        if len(set(self.joint_names)) != expected:
            raise ValueError("joint_names must be unique")
        previous = 0
        for point in self.points:
            if len(point.positions) != expected:
                raise ValueError("each point must contain one position per joint")
            if point.time_from_start_ms <= previous:
                raise ValueError("point times must be strictly increasing")
            previous = point.time_from_start_ms
        return self


class ProtectiveStopPayload(BaseModel):
    model_config = ConfigDict(extra="forbid")

    reason: str = Field(min_length=3, max_length=256)


CommandPayload = Annotated[
    NavigatePayload | ManipulatePayload | ProtectiveStopPayload,
    Field(discriminator=None),
]


class CommandRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    command_id: str = Field(pattern=r"^[A-Za-z0-9][A-Za-z0-9._:-]{0,127}$")
    robot_id: str = Field(pattern=r"^[A-Za-z0-9][A-Za-z0-9-]{0,62}$")
    kind: CommandKind
    ttl_ms: int = Field(default=30_000, ge=50, le=300_000)
    expected_state_version: int = Field(ge=0)
    payload: dict[str, object]

    def parsed_payload(self) -> NavigatePayload | ManipulatePayload | ProtectiveStopPayload:
        match self.kind:
            case CommandKind.NAVIGATE:
                return NavigatePayload.model_validate(self.payload)
            case CommandKind.MANIPULATE:
                return ManipulatePayload.model_validate(self.payload)
            case CommandKind.PROTECTIVE_STOP:
                return ProtectiveStopPayload.model_validate(self.payload)


class CommandStatus(BaseModel):
    model_config = ConfigDict(extra="forbid")

    command_id: str
    robot_id: str
    phase: CommandPhase
    sequence: int = Field(ge=0)
    message: str = Field(max_length=512)
    updated_at: datetime


class JointState(BaseModel):
    model_config = ConfigDict(extra="forbid")

    robot_id: str
    sequence: int = Field(ge=0)
    names: list[str]
    positions: list[float]
    velocities: list[float]
    stamp: datetime

    @model_validator(mode="after")
    def aligned_arrays(self) -> JointState:
        if not self.names or len(self.names) != len(self.positions):
            raise ValueError("names and positions must have the same non-zero length")
        if self.velocities and len(self.velocities) != len(self.names):
            raise ValueError("velocities must be empty or aligned with names")
        return self


class SafetyLevel(StrEnum):
    OPERATIONAL = "operational"
    PROTECTIVE_STOP = "protective_stop"
    DISCONNECTED = "disconnected"


class SafetyState(BaseModel):
    model_config = ConfigDict(extra="forbid")

    robot_id: str
    sequence: int = Field(ge=0)
    level: SafetyLevel
    reason: str
    stamp: datetime
    certified_emergency_stop: Literal[False] = False


class RobotState(BaseModel):
    model_config = ConfigDict(extra="forbid")

    robot_id: str
    profile: Literal["MH-01", "MM-01"]
    state_version: int = Field(ge=0)
    connected: bool
    safety: SafetyState
    joints: JointState | None
    active_command_id: str | None


class HealthResponse(BaseModel):
    status: Literal["ok", "degraded"]
    transport: Literal["connected", "disconnected"]
    robots: int
    active_commands: int
    contract_version: str


def now_utc() -> datetime:
    return datetime.now(UTC)
