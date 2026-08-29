#!/usr/bin/env python3
"""Validate the shared cross-repository quality scorecard contract."""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

REQUIRED_GATES = {"tests", "runtime_smoke", "memory", "security", "docs_examples"}


def validate(path: Path, *, require_target: bool = True) -> list[str]:
    errors: list[str] = []
    try:
        value: dict[str, Any] = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return [f"cannot read scorecard: {exc}"]
    if value.get("schema_version") != "1.0":
        errors.append("schema_version must be 1.0")
    categories = value.get("categories")
    if not isinstance(categories, list) or not categories:
        return [*errors, "categories must be a non-empty list"]
    maximum = 0
    earned = 0
    seen: set[str] = set()
    for category in categories:
        if not isinstance(category, dict):
            errors.append("each category must be an object")
            continue
        category_id = category.get("id")
        if not isinstance(category_id, str) or not category_id or category_id in seen:
            errors.append("category ids must be unique non-empty strings")
        else:
            seen.add(category_id)
        category_max = category.get("max")
        category_earned = category.get("earned")
        if not isinstance(category_max, int) or not isinstance(category_earned, int):
            errors.append(f"category {category_id!r} max and earned must be integers")
            continue
        if not 0 <= category_earned <= category_max:
            errors.append(f"category {category_id!r} earned is outside its range")
        maximum += category_max
        earned += category_earned
        evidence = category.get("evidence")
        if not isinstance(evidence, list) or not evidence or not all(
            isinstance(item, str) and item.strip() for item in evidence
        ):
            errors.append(f"category {category_id!r} requires evidence")
    if maximum != 100:
        errors.append(f"category maximum must total 100, got {maximum}")
    if value.get("score") != earned:
        errors.append(f"score must equal earned total {earned}")
    target = value.get("target")
    if not isinstance(target, int) or not 0 <= target <= 100:
        errors.append("target must be an integer from 0 through 100")
    gates = value.get("hard_gates")
    if not isinstance(gates, dict) or set(gates) != REQUIRED_GATES:
        errors.append("hard_gates must contain the five required gates exactly")
    elif require_target:
        failed = sorted(name for name, passed in gates.items() if passed is not True)
        if failed:
            errors.append("hard gates not passed: " + ", ".join(failed))
    if require_target and isinstance(target, int) and earned < target:
        errors.append(f"quality score {earned} is below target {target}")
    return errors


def main() -> int:
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("quality/scorecard.json")
    errors = validate(path)
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(f"quality scorecard passed: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
