import hashlib
import json
from datetime import timedelta
from pathlib import Path

import pytest
from jsonschema import Draft202012Validator, FormatChecker
from pydantic import TypeAdapter

from embedded_robot_ros2.models import (
    CommandRecord,
    CommandRequest,
    ProductProfile,
    RobotState,
    now_utc,
)

ROOT = Path(__file__).parents[1]


def test_vendored_physical_contract_hashes_are_exact_and_pinned() -> None:
    manifest = json.loads((ROOT / "contracts" / "manifest.json").read_text())
    assert manifest["contract_version"] == "1.0.0"
    for name, expected in manifest["sha256"].items():
        data = (ROOT / "contracts" / name).read_bytes()
        assert hashlib.sha256(data).hexdigest() == expected


def test_models_generate_the_same_four_canonical_schemas() -> None:
    models = {
        "command-request.schema.json": CommandRequest,
        "command-record.schema.json": CommandRecord,
        "product-profile.schema.json": ProductProfile,
        "robot-state.schema.json": RobotState,
    }
    for filename, model in models.items():
        canonical = json.loads((ROOT / "contracts" / filename).read_text())
        generated = TypeAdapter(model).json_schema()
        assert generated == canonical


def test_command_and_record_examples_conform_to_canonical_json_schemas() -> None:
    now = now_utc()
    command = {
        "contract_version": "1.0.0",
        "command_id": "conformance-1",
        "robot_id": "mm-01-a",
        "issued_at": now.isoformat(),
        "expires_at": (now + timedelta(seconds=1)).isoformat(),
        "expected_state_version": 2,
        "action": {
            "type": "navigate",
            "target": {"frame": "map", "x_m": 0, "y_m": 0, "yaw_rad": 0},
            "max_speed_mps": 0.5,
        },
    }
    CommandRequest.model_validate(command)
    schema = json.loads((ROOT / "contracts" / "command-request.schema.json").read_text())
    Draft202012Validator(schema, format_checker=FormatChecker()).validate(command)


def test_command_model_rejects_legacy_and_extra_fields() -> None:
    now = now_utc()
    legacy = {
        "command_id": "legacy",
        "robot_id": "MM-01",
        "kind": "navigate",
        "ttl_ms": 500,
        "issued_at": now.isoformat(),
        "expires_at": (now + timedelta(seconds=1)).isoformat(),
        "expected_state_version": 2,
        "payload": {},
    }
    with pytest.raises(ValueError):
        CommandRequest.model_validate(legacy)
