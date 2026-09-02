# ROS 2 Jazzy and physical connection

## Network and product connection

The edge target is an Ubuntu 24.04 industrial PC with ROS 2 Jazzy. Connect its robot
NIC to an isolated, non-routed OT VLAN. Assign fixed addresses from an installation
specific subnet; do not bridge the OT VLAN to office Wi-Fi or the public Internet.
Only required DDS discovery/data traffic and vendor traffic may cross an allow-list.

1. Verify the product (`mock-humanoid-mh-01` or `mock-mobile-manipulator-mm-01`) and
   compare advertised capabilities with the physical controller in a safe cell.
2. Set a unique `ROS_DOMAIN_ID` in the inclusive range 0–232 for that cell.
3. Map physical ID `mh-01-a` to ROS namespace `/mh_01_a` and `mm-01-a` to `/mm_01_a`.
   Hyphens are retained in HTTP identifiers and TF frame strings but are invalid in ROS graph names.
   Enable DDS localhost-only or discovery server mode unless multi-host discovery is required.
4. Map `JointState`, `NavigateToPose`, `FollowJointTrajectory`, and the deployment's
   typed `SafetyState` to `RclpyTransport`; never map a boolean or free-form string to
   the safety topic.
5. Validate heartbeat/watchdog, maximum velocities, joint limits, timeout, action
   cancellation, controller restart, and network loss on a hardware-in-loop rig.
6. Keep the certified E-stop and safety PLC hard-wired. The HTTP protective-stop
   request does not replace either device.

`RclpyTransport` imports safely on a developer machine. Its constructor requires a
deployment-provided `RosJazzyRuntime` that owns rclpy initialization, executor threads,
and generated messages; construction must fail if those dependencies are absent and
must not fall back to `MockRosGraph`. Run the same contract and state-machine tests
against that implementation before enabling motors.

## Recovery

On DDS disconnect the adapter reports degraded health, marks safety state disconnected,
and rejects new commands. Reconnect re-subscribes to state topics; clients must fetch a
new state version before submitting another command. Commands that were in flight are
not assumed complete and must be reconciled against the controller audit log.
