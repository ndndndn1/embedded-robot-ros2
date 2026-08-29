import hashlib
import json
from pathlib import Path

from embedded_robot_ros2.models import CommandRequest

ROOT = Path(__file__).parents[1]


def test_vendored_contract_hashes_are_canonical_and_pinned() -> None:
    manifest = json.loads((ROOT / "contracts" / "manifest.json").read_text())
    for name, expected in manifest["sha256"].items():
        data = (ROOT / "contracts" / name).read_bytes()
        assert hashlib.sha256(data).hexdigest() == expected


def test_command_model_rejects_unexpected_fields() -> None:
    command = {
        "command_id": "c-1",
        "robot_id": "MM-01",
        "kind": "navigate",
        "ttl_ms": 500,
        "expected_state_version": 2,
        "payload": {"pose": {"frame_id": "map", "x": 0, "y": 0, "yaw": 0}},
        "secret_extra": True,
    }
    try:
        CommandRequest.model_validate(command)
    except ValueError:
        pass
    else:
        raise AssertionError("unexpected fields must fail closed")

