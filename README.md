# embedded-robot-ros2

Vendor-neutral edge adapter between the canonical Physical Robot Interface 1.0.0 HTTP
wire and ROS 2 Jazzy topic/action semantics. The default runtime uses a deterministic
in-process ROS graph, so contract, timeout, cancellation, reconnection, and duplicate
delivery behavior are testable without ROS or robot hardware.

This component is an integration aid, **not a safety controller**. The
`protective_stop` action is a software request and is never represented as a certified
hardware emergency stop.

## Inputs, outputs, and targets

| Robot ID | Product ID | Classification | Joints | ROS namespace |
|---|---|---|---:|---|
| `mh-01-a` | `mock-humanoid-mh-01` | Industrial humanoid | 12 | `/mh-01-a` |
| `mm-01-a` | `mock-mobile-manipulator-mm-01` | Autonomous mobile manipulator | 6 | `/mm-01-a` |

Both products support `navigate`, `manipulate`, and `protective_stop`. Inputs are the
byte-pinned physical schemas under `contracts/`. Outputs use the exact canonical
`ProductProfile`, `RobotState`, `CommandRecord`, and `{code,message}` error wire.

The adapter maps to these ROS interfaces for each robot instance:

- `/<robot_id>/joint_states` — `sensor_msgs/msg/JointState`
- `/<robot_id>/navigate_to_pose` — `nav2_msgs/action/NavigateToPose`
- `/<robot_id>/follow_joint_trajectory` — `control_msgs/action/FollowJointTrajectory`
- `/<robot_id>/safety_state` — deployment-provided typed `SafetyState`

## Run the deterministic mock

```bash
uv sync --extra dev
uv run pytest
uv run uvicorn embedded_robot_ros2.app:app --host 127.0.0.1 --port 8080
```

Read the canonical state and use its current version:

```bash
curl -s http://127.0.0.1:8080/v1/products
curl -s http://127.0.0.1:8080/v1/robots/mm-01-a
```

```bash
curl -s -X POST http://127.0.0.1:8080/v1/commands \
  -H 'content-type: application/json' \
  -d '{"contract_version":"1.0.0","command_id":"demo-1","robot_id":"mm-01-a","issued_at":"2026-08-29T12:00:00Z","expires_at":"2026-08-29T12:01:00Z","expected_state_version":2,"action":{"type":"navigate","target":{"x_m":1,"y_m":2,"yaw_rad":0,"frame":"map"},"max_speed_mps":0.5}}'
```

Use live timestamps in production. Poll `/v1/commands/demo-1` or cancel with
`POST /v1/commands/demo-1/cancel`. Reusing the same ID and body is idempotent; reusing
it with different content returns HTTP 409. No legacy `kind`, `payload`, `ttl_ms`,
uppercase profile-as-robot IDs, or `/state` route is accepted.

See [real adapter connection](docs/connection.md),
[requirements](docs/enterprise-requirements.md), and
[quality score](docs/quality-scorecard.md) before deployment.

