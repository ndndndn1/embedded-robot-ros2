#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
package_root="$repo_root/ros2_ws/src/embedded_robot_perception"
build_root=${PERCEPTION_BUILD_ROOT:-/tmp/embedded-robot-perception-build}
mkdir -p "$build_root"

common_sources=(
  "$package_root/src/backend_selector.cpp"
  "$package_root/src/calibration_manager.cpp"
  "$package_root/src/cpu_pose_backend.cpp"
  "$package_root/src/frame_aligner.cpp"
  "$package_root/src/grasp_validator.cpp"
)
common_flags=(-std=c++20 -Wall -Wextra -Werror -Wpedantic -I"$package_root/include" -pthread)

g++ "${common_flags[@]}" "${common_sources[@]}" "$package_root/test/core_test.cpp" \
  -o "$build_root/core_test"
"$build_root/core_test"

g++ "${common_flags[@]}" -O3 "${common_sources[@]}" "$package_root/bench/core_benchmark.cpp" \
  -o "$build_root/core_benchmark"
"$build_root/core_benchmark" "${PERCEPTION_BENCH_ITERATIONS:-20}"

if [[ ${PERCEPTION_SANITIZE:-1} == 1 ]]; then
  g++ "${common_flags[@]}" -O1 -g -fno-omit-frame-pointer -fsanitize=address,undefined \
    "${common_sources[@]}" "$package_root/test/core_test.cpp" -o "$build_root/core_test_sanitized"
  ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 \
    "$build_root/core_test_sanitized"
fi
