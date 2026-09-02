#!/usr/bin/env bash
set -eo pipefail

install_prefix=${1:-/opt/embedded_perception}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-162}
. /opt/ros/jazzy/setup.sh
. "$install_prefix/setup.sh"
set -u

log_root=$(mktemp -d)
launch_pid=0
fixture_pid=0
cleanup() {
  if [[ $fixture_pid -gt 0 ]]; then kill "$fixture_pid" 2>/dev/null || true; fi
  if [[ $launch_pid -gt 0 ]]; then kill "$launch_pid" 2>/dev/null || true; fi
  wait "$fixture_pid" "$launch_pid" 2>/dev/null || true
  ros2 daemon stop >/dev/null 2>&1 || true
  rm -rf "$log_root"
}
trap cleanup EXIT

ros2 launch embedded_robot_perception perception.launch.py >"$log_root/launch.log" 2>&1 &
launch_pid=$!
for _ in $(seq 1 15); do
  if ros2 lifecycle get /mock_perception_01/perception >/dev/null 2>&1; then break; fi
  sleep 1
done
ros2 lifecycle set /mock_perception_01/perception configure | grep -q "Transitioning successful"
ros2 lifecycle set /mock_perception_01/perception activate | grep -q "Transitioning successful"

ros2 run embedded_robot_perception mock_fixture_publisher \
  --ros-args -r __ns:=/mock_perception_01 >"$log_root/fixture.log" 2>&1 &
fixture_pid=$!
for _ in $(seq 1 15); do
  if ros2 topic echo /mock_perception_01/perception/frame_info --once --field backend \
      >"$log_root/backend" 2>/dev/null; then break; fi
  sleep 1
done
grep -qx "cpu" "$log_root/backend"

declare -A topic_types=(
  [/mock_perception_01/perception/aligned_depth/image]=sensor_msgs/msg/Image
  [/mock_perception_01/perception/aligned_points]=sensor_msgs/msg/PointCloud2
  [/mock_perception_01/perception/detections_3d]=vision_msgs/msg/Detection3DArray
  [/mock_perception_01/perception/grasp_candidates]=embedded_robot_interfaces/msg/GraspCandidateArray
  [/mock_perception_01/perception/frame_info]=embedded_robot_interfaces/msg/PerceptionFrameInfo
)
for topic in "${!topic_types[@]}"; do
  [[ $(ros2 topic type "$topic") == "${topic_types[$topic]}" ]]
done

sleep 1
ros2 action send_goal \
  /mock_perception_01/perception/calibrate_extrinsics \
  embedded_robot_interfaces/action/CalibrateExtrinsics \
  "{camera_name: front_rgbd, camera_frame: mock-perception-01/front_rgbd_color_optical_frame, target_frame: mock-perception-01/base_link, target_id: mock-charuco-7x5, required_samples: 3, max_reprojection_error_px: 1.0}" \
  >"$log_root/calibration"
grep -q "success: true" "$log_root/calibration"
grep -Eq "calibration_id: [0-9a-f]{64}" "$log_root/calibration"
grep -q "failure_code: 0" "$log_root/calibration"

ros2 action list -t | grep -q "/mock_perception_01/perception/validate_grasp"
echo "ROS graph, mock perception, and calibration action checks passed"
