#!/usr/bin/env python3
"""Safe localhost-only smoke for the canonical physical wire and ROS mock."""

from __future__ import annotations

import json
import time
import urllib.error
import urllib.request
import uuid
from datetime import datetime, timedelta, timezone
from typing import Any

BASE_URL = "http://127.0.0.1:8080"


def request(path: str, *, method: str = "GET", body: dict[str, Any] | None = None) -> Any:
    data = json.dumps(body).encode() if body is not None else None
    headers = {"content-type": "application/json"} if data is not None else {}
    operation = urllib.request.Request(BASE_URL + path, data=data, headers=headers, method=method)
    try:
        with urllib.request.urlopen(operation, timeout=5) as response:
            return json.load(response)
    except urllib.error.URLError as exc:
        raise RuntimeError(f"localhost smoke request failed for {path}: {exc}") from exc


def main() -> int:
    health = request("/healthz")
    if health["status"] != "ok" or health["contract_version"] != "1.0.0":
        raise RuntimeError("health response did not match the adapter contract")
    products = request("/v1/products")
    if {item["product_id"] for item in products} != {
        "mock-humanoid-mh-01",
        "mock-mobile-manipulator-mm-01",
    }:
        raise RuntimeError("product catalog did not contain both canonical profiles")
    state = request("/v1/robots/mm-01-a")
    now = datetime.now(timezone.utc)
    command_id = f"smoke-{uuid.uuid4().hex}"
    command = {
        "contract_version": "1.0.0",
        "command_id": command_id,
        "robot_id": state["robot_id"],
        "issued_at": now.isoformat(),
        "expires_at": (now + timedelta(seconds=30)).isoformat(),
        "expected_state_version": state["state_version"],
        "action": {
            "type": "navigate",
            "target": {"x_m": 0.25, "y_m": 0.0, "yaw_rad": 0.0, "frame": "map"},
            "max_speed_mps": 0.5,
        },
    }
    accepted = request("/v1/commands", method="POST", body=command)
    duplicate = request("/v1/commands", method="POST", body=command)
    if (
        accepted["request"] != duplicate["request"]
        or duplicate["request"]["command_id"] != command_id
        or accepted["status"] not in {"accepted", "running", "completed"}
    ):
        raise RuntimeError("idempotent submit contract failed")
    deadline = time.monotonic() + 5
    record = accepted
    while record["status"] not in {"completed", "failed", "cancelled", "rejected"}:
        if time.monotonic() >= deadline:
            raise RuntimeError("command did not become terminal")
        time.sleep(0.02)
        record = request(f"/v1/commands/{command_id}")
    if record["status"] != "completed":
        raise RuntimeError(f"command did not complete: {record['status']}")
    updated = request("/v1/robots/mm-01-a")
    if updated["pose"]["x_m"] != 0.25:
        raise RuntimeError("completed navigation was not normalized into robot state")
    print(json.dumps({"status": "pass", "robot_id": state["robot_id"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
