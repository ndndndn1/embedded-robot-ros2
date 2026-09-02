import json
from pathlib import Path

ROOT = Path(__file__).parents[1]


def test_perception_fixture_and_interfaces_are_versioned_and_safe() -> None:
    fixture = json.loads(
        (ROOT / "fixtures/perception/red_cuboid_scene.json").read_text(encoding="utf-8")
    )
    assert fixture["schema_version"] == "1.0"
    assert fixture["robot_id"] == "mock-perception-01"
    assert fixture["ros_namespace"] == "/mock_perception_01"
    assert fixture["camera"]["depth_encoding"] == "16UC1"
    assert fixture["expected"] == {
        "backend": "cpu",
        "detection_count": 1,
        "grasp_candidate_count": 1,
        "calibration_success": True,
    }

    action = (ROOT / "ros2_ws/src/embedded_robot_interfaces/action/ValidateGrasp.action").read_text(
        encoding="utf-8"
    )
    for fail_closed_reason in (
        "STALE_SCENE",
        "CALIBRATION_MISMATCH",
        "COLLISION",
        "REACHABILITY_UNAVAILABLE",
        "IK_FAILED",
    ):
        assert fail_closed_reason in action


def test_existing_physical_contract_manifest_is_unchanged() -> None:
    manifest = json.loads((ROOT / "contracts/manifest.json").read_text(encoding="utf-8"))
    assert manifest["contract_version"] == "1.0.0"
    assert len(manifest["sha256"]) == 4
