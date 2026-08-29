# embedded-robot-ros2

Vendor-neutral edge adapter between a physical robot HTTP contract and ROS 2 Jazzy
topic/action semantics. The default runtime uses a deterministic in-process ROS graph,
so contract, timeout, cancellation, reconnection, and duplicate-delivery behavior are
testable without ROS or robot hardware.

This component is an integration aid, **not a safety controller**. The
`protective_stop` command is a software request and is never represented as a certified
hardware emergency stop.

## Targets and boundaries

| Product profile | Classification | Supported commands | ROS namespace |
|---|---|---|---|
| MockHumanoid MH-01 | Industrial humanoid | manipulate, protective stop | `/MH-01` |
| MockMobileManipulator MM-01 | Mobile manipulator | navigate, manipulate, protective stop | `/MM-01` |

Inputs are versioned physical command JSON documents under `contracts/`. Outputs are
normalized command status, robot state, health, and Prometheus text metrics. The
adapter maps to these exact ROS interfaces:

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

Read the current state version before submitting a command:

```bash
curl -s http://127.0.0.1:8080/v1/robots/MM-01/state
curl -s -X POST http://127.0.0.1:8080/v1/commands \
  -H 'content-type: application/json' \
  -d '{"command_id":"demo-1","robot_id":"MM-01","kind":"navigate","ttl_ms":30000,"expected_state_version":2,"payload":{"pose":{"frame_id":"map","x":1,"y":2,"yaw":0}}}'
```

The accepted command maps to `/MM-01/navigate_to_pose`. Poll
`/v1/commands/demo-1` or cancel with `POST /v1/commands/demo-1/cancel`. Reusing the
same ID and body is idempotent; reusing it with different content returns HTTP 409.

See [real adapter connection](docs/connection.md),
[requirements](docs/enterprise-requirements.md), and
[quality score](docs/quality-scorecard.md) before deployment.

