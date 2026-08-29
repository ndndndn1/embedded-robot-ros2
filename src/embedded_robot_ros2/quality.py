from __future__ import annotations

import json
from pathlib import Path
from typing import TypedDict

REQUIRED_GATES = {"tests", "runtime_smoke", "memory", "security", "docs_examples"}


class Category(TypedDict):
    id: str
    max: int
    earned: int
    evidence: list[str]


class Scorecard(TypedDict):
    schema_version: str
    target: int
    score: int
    categories: list[Category]
    hard_gates: dict[str, bool]


def check(root: Path) -> tuple[int, list[str]]:
    scorecard: Scorecard = json.loads((root / "quality" / "scorecard.json").read_text())
    failures: list[str] = []
    if scorecard.get("schema_version") != "1.0":
        failures.append("schema_version must be 1.0")
    maximum = sum(item["max"] for item in scorecard["categories"])
    score = sum(item["earned"] for item in scorecard["categories"])
    if maximum != 100:
        failures.append(f"maximum totals {maximum}, expected 100")
    if scorecard.get("score") != score:
        failures.append("declared score does not match earned total")
    for item in scorecard["categories"]:
        if not 0 <= item["earned"] <= item["max"]:
            failures.append(f"invalid score for {item['id']}")
        if not item["evidence"]:
            failures.append(f"missing evidence for {item['id']}")
    if score < scorecard["target"]:
        failures.append(f"score {score} below target {scorecard['target']}")
    gates = scorecard.get("hard_gates", {})
    if set(gates) != REQUIRED_GATES or not all(gates.values()):
        failures.append("hard gates are incomplete or false")
    return score, failures


def main() -> None:
    checkout = Path.cwd()
    root = (
        checkout
        if (checkout / "quality" / "scorecard.json").is_file()
        else Path(__file__).resolve().parents[2]
    )
    score, failures = check(root)
    print(f"quality_score={score}/100")
    if failures:
        raise SystemExit("; ".join(failures))


if __name__ == "__main__":
    main()
