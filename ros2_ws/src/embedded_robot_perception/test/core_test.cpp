#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "embedded_robot_perception/calibration_manager.hpp"
#include "embedded_robot_perception/frame_aligner.hpp"
#include "embedded_robot_perception/grasp_validator.hpp"
#include "embedded_robot_perception/pose_backend.hpp"
#include "embedded_robot_perception/sha256.hpp"

namespace perception = embedded_robot_perception;

namespace {

int failures = 0;

void expect(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

void test_sha256() {
  expect(perception::sha256("abc") ==
             "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
         "SHA-256 must match the published test vector");
}

void test_alignment() {
  constexpr std::uint32_t width = 4;
  constexpr std::uint32_t height = 3;
  const perception::Intrinsics intrinsics{width, height, 100.0, 100.0, 1.5, 1.0};
  std::vector<std::uint16_t> depth(width * height, 1000);
  depth[0] = 0;
  depth[1] = std::numeric_limits<std::uint16_t>::max();
  std::vector<perception::Rgb8> color(width * height, {200, 10, 10});
  const perception::AlignedFrame aligned = perception::FrameAligner{}.align_u16_mm(
      depth, color, intrinsics, intrinsics, perception::Transform{});
  expect(aligned.depth_m.size() == width * height, "aligned image keeps color geometry");
  expect(aligned.points.size() == width * height - 1, "zero depth is filtered");
  expect(std::abs(aligned.depth_m[5] - 1.0F) < 1e-6F, "millimetres convert to metres");
}

void test_keypoint_pose() {
  const std::array<perception::Vec3, 5> model{{
      {0.0, 0.0, 0.0}, {0.1, 0.0, 0.0}, {0.0, 0.2, 0.0}, {0.0, 0.0, 0.3}, {0.1, 0.2, 0.3}}};
  const perception::Quaternion q{std::cos(0.3), 0.0, std::sin(0.3), 0.0};
  const perception::Transform transform{perception::quaternion_matrix(q), {0.4, -0.1, 0.8}};
  std::array<perception::Vec3, 5> observed{};
  for (std::size_t index = 0; index < model.size(); ++index) {
    observed[index] = transform.apply(model[index]);
  }
  perception::CpuPoseBackend backend;
  const auto result = backend.estimate_correspondences(model, observed);
  expect(result.valid, "known keypoints produce a pose");
  expect(perception::norm(result.pose.position - transform.translation) < 1e-6,
         "pose translation is recovered");
  expect(result.rms_error_m < 1e-6, "pose residual is bounded");
}

void test_cluster_pose() {
  std::vector<perception::PointXYZRGB> points;
  for (int x = -4; x <= 4; ++x) {
    for (int y = -2; y <= 2; ++y) {
      points.push_back({{0.5 + x * 0.01, -0.2 + y * 0.01, 0.8}, {220, 20, 20}});
    }
  }
  perception::CpuPoseBackend backend;
  const auto result = backend.estimate_cluster(points);
  expect(result.valid, "red cluster produces deterministic 6DoF pose");
  expect(perception::norm(result.pose.position - perception::Vec3{0.5, -0.2, 0.8}) < 1e-9,
         "cluster centroid is recovered");
}

void test_calibration() {
  const perception::Quaternion q{std::cos(0.1), std::sin(0.1), 0.0, 0.0};
  std::vector<perception::CalibrationSample> samples;
  for (int index = 0; index < 30; ++index) {
    const double delta = (index % 3 - 1) * 0.0002;
    samples.push_back({{perception::quaternion_matrix(q), {0.1 + delta, -0.2, 0.4}}, 0.4});
  }
  const auto result = perception::CalibrationManager{}.finalize(
      samples, 30, 1.0, "D455|serial-mock|640x480x30|K=100,100,1.5,1.0|");
  expect(result.valid, "stable 30-sample calibration is accepted");
  expect(result.calibration_id.size() == 64, "calibration provenance is SHA-256");
  samples.front().reprojection_error_px = 5.0;
  const auto rejected = perception::CalibrationManager{}.finalize(samples, 30, 0.5, "same");
  expect(!rejected.valid, "high reprojection error is rejected");
}

void test_grasp_validation() {
  const perception::GraspCandidateCore candidate{
      "grasp-1", {{0.5, 0.0, 0.8}, {}}, {0.0, 0.0, -1.0}, 0.08, 0.9, 0.002, 0.05};
  perception::GraspValidator validator;
  const std::vector<perception::PointXYZRGB> clear_scene;
  const auto valid = validator.validate(candidate, clear_scene, false, false, false);
  expect(valid.valid && valid.reason == perception::ValidationReason::ok,
         "clear geometric grasp is valid when reachability is not requested");
  const auto fail_closed = validator.validate(candidate, clear_scene, true, false, false);
  expect(!fail_closed.valid &&
             fail_closed.reason == perception::ValidationReason::reachability_unavailable,
         "missing reachability service fails closed");
  const std::vector<perception::PointXYZRGB> obstacle{{{0.5, 0.0, 0.90}, {1, 1, 1}}};
  const auto collision = validator.validate(candidate, obstacle, false, false, false);
  expect(!collision.valid && collision.reason == perception::ValidationReason::collision,
         "approach obstacle is rejected");
}

void test_backend_selection() {
  auto cpu = perception::select_backend(perception::BackendMode::cpu, false);
  expect(cpu.healthy && cpu.backend && cpu.backend->name() == "cpu", "CPU self-test passes");
  auto automatic = perception::select_backend(perception::BackendMode::automatic, false);
  expect(automatic.healthy && automatic.backend, "automatic mode has a tested fallback");
  auto required = perception::select_backend(perception::BackendMode::automatic, true);
#ifndef EMBEDDED_ROBOT_HAS_CUDA_BACKEND
  expect(!required.healthy && !required.backend, "required GPU fails closed in CPU build");
#endif
}

}  // namespace

int main() {
  test_sha256();
  test_alignment();
  test_keypoint_pose();
  test_cluster_pose();
  test_calibration();
  test_grasp_validation();
  test_backend_selection();
  if (failures != 0) {
    std::cerr << failures << " core tests failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "all embedded perception core tests passed\n";
  return EXIT_SUCCESS;
}
