#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include "embedded_robot_perception/types.hpp"

namespace embedded_robot_perception {

struct TensorImageView {
  const std::uint8_t *data{};
  std::size_t size_bytes{};
  std::uint32_t width{};
  std::uint32_t height{};
  std::uint32_t row_stride_bytes{};
};

struct KeypointInference {
  std::vector<Vec3> model_keypoints;
  std::vector<Vec3> observed_keypoints;
  std::string model_id;
  std::string model_sha256;
  double confidence{};
};

class KeypointInferenceBackend {
 public:
  virtual ~KeypointInferenceBackend() = default;
  [[nodiscard]] virtual std::string name() const = 0;
  [[nodiscard]] virtual bool configure(
      const std::string &model_path, const std::string &expected_sha256, std::string &detail) = 0;
  [[nodiscard]] virtual KeypointInference infer(
      const TensorImageView &color, std::span<const float> aligned_depth_m) = 0;
};

// CUDA/TensorRT implementations must implement this ABI-neutral C++ interface. CUDA and
// TensorRT handles are intentionally forbidden from public headers so CPU builds stay portable.

}  // namespace embedded_robot_perception
