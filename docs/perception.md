# RGB-D perception interface and operation

The perception workspace is a ROS 2 Jazzy C++20 data-plane. It does not add or alter any
Physical Robot Interface 1.0.0 HTTP route. It cannot send a robot command. A grasp becomes
executable only after a separate motion-planning adapter and approved safety process accept it.

## Identity, inputs, outputs, and target

Physical `robot_id` and `ros_namespace` are separate values. For example,
`mh-01-a` uses `/mh_01_a`; TF frames retain the physical ID. `<ns>` below means the valid
ROS namespace without a trailing slash.

| Direction | Name | Type | Contract |
|---|---|---|---|
| Input | `<ns>/sensors/front_rgbd/color/image_raw` | `sensor_msgs/msg/Image` | `rgb8` or `bgr8`; acquisition stamp; color optical frame |
| Input | `<ns>/sensors/front_rgbd/color/camera_info` | `sensor_msgs/msg/CameraInfo` | same frame as color Image; nonzero K |
| Input | `<ns>/sensors/front_rgbd/depth/image_raw` | `sensor_msgs/msg/Image` | little-endian `16UC1` mm or `32FC1` m |
| Input | `<ns>/sensors/front_rgbd/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | same frame as depth Image; nonzero K |
| Input | `<ns>/perception/calibration_observation` | `embedded_robot_interfaces/msg/CalibrationObservation` | independent ChArUco/target detector observation |
| Input | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | transform available at acquisition time |
| Output | `<ns>/perception/aligned_depth/image` | `sensor_msgs/msg/Image` | canonical `32FC1` metres in color optical frame |
| Output | `<ns>/perception/aligned_points` | `sensor_msgs/msg/PointCloud2` | `x/y/z/rgb` in `<robot_id>/base_link` |
| Output | `<ns>/perception/detections_3d` | `vision_msgs/msg/Detection3DArray` | 6DoF pose and covariance in base frame |
| Output | `<ns>/perception/grasp_candidates` | `embedded_robot_interfaces/msg/GraspCandidateArray` | scene and calibration bound candidates |
| Output | `<ns>/perception/frame_info` | `embedded_robot_interfaces/msg/PerceptionFrameInfo` | sequence, calibration, model hash, backend, pipeline version |
| Output | `<ns>/perception/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | lifecycle, TF, backend, and parser failures |

Sensor streams use best-effort volatile keep-last 4. CameraInfo is reliable volatile keep-last 4.
Detection, grasp, and frame provenance use reliable volatile keep-last 5. Diagnostics uses reliable
transient-local keep-last 1. RGB/depth skew above 15 ms, inconsistent Image/CameraInfo frames,
invalid intrinsics, unsupported encoding, truncated buffers, or unavailable TF suppress the frame.

The frame tree is:

```text
<robot_id>/map -> <robot_id>/odom -> <robot_id>/base_link
  -> <robot_id>/front_rgbd_link
    -> <robot_id>/front_rgbd_color_optical_frame
    -> <robot_id>/front_rgbd_depth_optical_frame
  -> <robot_id>/tool0 -> <robot_id>/gripper_link
```

Body frames are x-forward/y-left/z-up. Optical frames are x-right/y-down/z-forward. Quaternion
norm must remain within 0.1% of one. Transform lookup has a 50 ms safety timeout and uses the
image acquisition timestamp rather than the latest transform.

## Calibration and grasp actions

`<ns>/sensors/front_rgbd/set_camera_info` uses `sensor_msgs/srv/SetCameraInfo`. The in-memory
value is not durable; persist it through the camera driver's calibration store.

`<ns>/perception/calibrate_extrinsics` uses `CalibrateExtrinsics.action`. Goal fields are
`camera_name`, `camera_frame`, `target_frame`, `target_id`, `required_samples`, and
`max_reprojection_error_px`. At least three matching, independently stamped observations are
required; production uses 30. The result includes `success`, `calibration_id`, `extrinsics`, RMS
error, sample count, failure code, and detail. Acceptance also requires translation spread at most
5 mm and rotation spread at most 0.5 degrees. `calibration_id` is SHA-256 over camera identity,
stream profile, requested frames/target, and the averaged transform.

`<ns>/perception/validate_grasp` uses `ValidateGrasp.action`. Its goal binds
`candidate_id + calibration_id + scene_sequence`, maximum scene age, minimum clearance, and
whether reachability is mandatory. Result reason codes are defined in the action file. Stale or
mismatched provenance, pose uncertainty, collision, workspace failure, missing reachability, or
IK failure always returns `valid=false`. The built-in mock can exercise reachability but defaults
to false. Production must replace it with MoveIt `GetPositionIK` and `GetStateValidity` checks.

## CPU, CUDA, and TensorRT boundary

The CPU path performs depth alignment, point generation, deterministic red-object segmentation,
PCA cluster orientation, matched-keypoint Horn 6DoF estimation, calibration aggregation, and
swept-volume grasp collision checks. It is the correctness reference and needs no GPU.

`backend=auto|cpu|cuda` and `require_gpu` control selection. Every selected backend runs a known
pose self-test. The CUDA plugin owns a nonblocking stream and RAII device buffer, runs a kernel and
copy-back self-test, and exposes no CUDA type in public headers. It currently accelerates only the
validated preprocessing boundary and deliberately uses the CPU numerical pose implementation;
it must not be reported as a TensorRT inference speedup. `ENABLE_TENSORRT=ON` adds TensorRT ABI
validation when an approved engine image is built. Model files are never embedded here; production
must mount a read-only model whose SHA-256 matches `PerceptionFrameInfo.model_sha256`.

In `auto`, an unavailable CUDA build/device degrades to the tested CPU backend between frames.
Explicit `cuda` or `require_gpu=true` fails lifecycle configuration instead of falling back.

## Product targets and connections

| Target | Product | Connection | Compute | Status |
|---|---|---|---|---|
| CI/mock | `mock-perception-01` fixture | internal DDS bridge only | 4-vCPU x86, no GPU | enabled |
| Development camera | Intel RealSense D455 | dedicated USB 3.x port; exact bus device only | Ubuntu 24.04 IPC, CPU or RTX 4000 SFF | HIL approval required |
| Industrial camera | Intel RealSense D457 | GMSL/FAKRA to supported carrier | Jetson AGX Orin Industrial | HIL approval required |
| Robot controller | existing approved robot adapter | separate allow-listed OT NIC/VLAN | never passed into perception container | disabled by default |

Use the official `realsense2_camera` Jazzy driver and select its namespace remaps to match the
input table. Validate serial number, resolution, FPS, USB/GMSL bandwidth, and PTP/chrony clock
discipline before calibration. Do not pass all of `/dev`, use `privileged`, or use host networking.
The default Compose network is internal. Production replaces it with an approved macvlan/ipvlan
OT attachment or a unicast Fast DDS discovery server configuration.

The x86 CUDA image and Jetson L4T/JetPack image are separate build products. The repository ships
the x86 build definition; a Jetson image must pin the deployment's JetPack/L4T release and pass the
same core, graph, CUDA self-test, camera HIL, soak, and safety review before use.

## Reproduction

```bash
./tools/run_cpp_checks.sh
docker build --target ros2-test -f Dockerfile.ros2 -t embedded-robot-perception:test .
docker compose -f compose.perception.yaml config
```

The mock fixture publishes a 64x48 red cuboid, 800 mm depth, valid CameraInfo, static TF, and
calibration observations. `tools/run_ros_graph_check.sh` configures and activates the lifecycle
component, verifies topic types and CPU provenance, and executes the calibration action.
