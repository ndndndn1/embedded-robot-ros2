#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "embedded_robot_perception/frame_aligner.hpp"
#include "embedded_robot_perception/pose_backend.hpp"

namespace perception = embedded_robot_perception;

int main(int argc, char **argv) {
  const int iterations = argc > 1 ? std::atoi(argv[1]) : 100;
  if (iterations < 1 || iterations > 10000) {
    std::cerr << "iterations must be between 1 and 10000\n";
    return 2;
  }
  constexpr std::uint32_t width = 640;
  constexpr std::uint32_t height = 480;
  const perception::Intrinsics intrinsics{width, height, 615.0, 615.0, 319.5, 239.5};
  std::vector<std::uint16_t> depth(width * height, 800);
  std::vector<perception::Rgb8> color(width * height, {20, 20, 20});
  for (std::uint32_t row = 180; row < 300; ++row) {
    for (std::uint32_t column = 240; column < 400; ++column) {
      color[static_cast<std::size_t>(row) * width + column] = {220, 20, 20};
    }
  }
  perception::FrameAligner aligner;
  perception::CpuPoseBackend backend;
  std::vector<double> timings_ms;
  timings_ms.reserve(static_cast<std::size_t>(iterations));
  for (int iteration = 0; iteration < iterations; ++iteration) {
    const auto started = std::chrono::steady_clock::now();
    const auto aligned = aligner.align_u16_mm(depth, color, intrinsics, intrinsics, {});
    const auto pose = backend.estimate_cluster(aligned.points);
    if (!pose.valid) {
      std::cerr << "pose estimation failed\n";
      return 1;
    }
    const auto elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started);
    timings_ms.push_back(elapsed.count());
  }
  std::sort(timings_ms.begin(), timings_ms.end());
  const double p50 = timings_ms[timings_ms.size() / 2];
  const double p95 = timings_ms[static_cast<std::size_t>(std::floor((timings_ms.size() - 1) * 0.95))];
  const double hz = 1000.0 / p50;
  std::cout << "{\"backend\":\"cpu\",\"width\":640,\"height\":480,\"iterations\":"
            << iterations << ",\"p50_ms\":" << p50 << ",\"p95_ms\":" << p95
            << ",\"throughput_hz\":" << hz << "}\n";
  return p95 <= 100.0 ? 0 : 1;
}
