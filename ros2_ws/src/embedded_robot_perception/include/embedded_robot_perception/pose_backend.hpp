#pragma once

#include <memory>
#include <span>
#include <string>

#include "embedded_robot_perception/types.hpp"

namespace embedded_robot_perception {

class PoseBackend {
 public:
  virtual ~PoseBackend() = default;
  [[nodiscard]] virtual std::string name() const = 0;
  [[nodiscard]] virtual bool self_test(std::string &detail) = 0;
  [[nodiscard]] virtual PoseEstimate estimate_correspondences(
      std::span<const Vec3> model_points, std::span<const Vec3> observed_points) = 0;
  [[nodiscard]] virtual PoseEstimate estimate_cluster(std::span<const PointXYZRGB> points) = 0;
};

class CpuPoseBackend final : public PoseBackend {
 public:
  [[nodiscard]] std::string name() const override { return "cpu"; }
  [[nodiscard]] bool self_test(std::string &detail) override;
  [[nodiscard]] PoseEstimate estimate_correspondences(
      std::span<const Vec3> model_points, std::span<const Vec3> observed_points) override;
  [[nodiscard]] PoseEstimate estimate_cluster(std::span<const PointXYZRGB> points) override;
};

enum class BackendMode { automatic, cpu, cuda };

struct BackendSelection {
  std::unique_ptr<PoseBackend> backend;
  bool healthy{};
  bool degraded{};
  std::string detail;
};

[[nodiscard]] BackendSelection select_backend(BackendMode mode, bool require_gpu);

#ifdef EMBEDDED_ROBOT_HAS_CUDA_BACKEND
[[nodiscard]] std::unique_ptr<PoseBackend> make_cuda_pose_backend();
#endif

}  // namespace embedded_robot_perception
