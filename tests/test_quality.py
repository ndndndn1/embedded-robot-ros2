from pathlib import Path

from embedded_robot_ros2.quality import check


def test_quality_gate_is_at_least_80_with_existing_evidence() -> None:
    score, failures = check(Path(__file__).parents[1])
    assert score >= 80
    assert failures == []

