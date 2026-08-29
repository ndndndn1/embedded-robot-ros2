from __future__ import annotations

import json
from pathlib import Path
from typing import TypedDict


class Category(TypedDict):
    name: str
    weight: int
    score: int
    evidence: list[str]


class Scorecard(TypedDict):
    threshold: int
    categories: list[Category]


def check(root: Path) -> tuple[int, list[str]]:
    scorecard: Scorecard = json.loads((root / "quality" / "scorecard.json").read_text())
    failures: list[str] = []
    total_weight = sum(item["weight"] for item in scorecard["categories"])
    total_score = sum(item["score"] for item in scorecard["categories"])
    if total_weight != 100:
        failures.append(f"weights total {total_weight}, expected 100")
    for item in scorecard["categories"]:
        if not 0 <= item["score"] <= item["weight"]:
            failures.append(f"invalid score for {item['name']}")
        for evidence in item["evidence"]:
            if not (root / evidence).is_file():
                failures.append(f"missing evidence: {evidence}")
    if total_score < scorecard["threshold"]:
        failures.append(f"score {total_score} below threshold {scorecard['threshold']}")
    return total_score, failures


def main() -> None:
    root = Path(__file__).resolve().parents[2]
    score, failures = check(root)
    print(f"quality_score={score}/100")
    if failures:
        raise SystemExit("; ".join(failures))


if __name__ == "__main__":
    main()

