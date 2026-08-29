# Enterprise requirements

| ID | Requirement | Acceptance evidence |
|---|---|---|
| F-01 | Map navigate and trajectory commands to typed ROS action goals | Unit tests inspect both goal paths |
| F-02 | Support `mh-01-a` and `mm-01-a` instances with canonical product profiles | Catalog and state tests |
| F-03 | Normalize joint and typed safety state | State validation tests |
| R-01 | Reject disconnected, stale-version, and concurrent unsafe commands | Failure-mode tests |
| R-02 | Cancel on TTL, bound command history, and release all tasks on close | Timeout, capacity, and leak tests |
| R-03 | Ignore duplicate, stale, and unknown DDS status updates | Sequence tests and metric |
| I-01 | Vendor all four Physical Robot Interface 1.0.0 schemas with pinned SHA-256 | Schema identity test and manifest |
| I-02 | Keep ROS implementation behind a typed transport port | `RosTransport`, mock, rclpy boundary |
| S-01 | Never claim software stop is certified E-stop | Typed literal false and docs/tests |
| S-02 | Run container non-root, read-only, localhost-only, no capabilities | Docker/Compose inspection |
| O-01 | Expose health and low-cardinality metrics | API tests |
| P-01 | Demonstrate bounded latency and no live background tasks | Benchmark and soak scripts |
| U-01 | Explain product, network, namespace, topics/actions, and replacement steps | README and connection guide |

Out of scope: certified functional safety, motor control, vendor-specific protocol
drivers, motion planning, and automatic production enablement.
