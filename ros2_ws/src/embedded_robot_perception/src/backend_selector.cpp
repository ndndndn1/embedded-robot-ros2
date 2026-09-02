#include "embedded_robot_perception/pose_backend.hpp"

#include <exception>

namespace embedded_robot_perception {

BackendSelection select_backend(BackendMode mode, bool require_gpu) {
  if (mode == BackendMode::cpu && require_gpu) {
    return {nullptr, false, false, "REQUIRE_GPU conflicts with PERCEPTION_BACKEND=cpu"};
  }
  if (mode == BackendMode::cpu) {
    auto backend = std::make_unique<CpuPoseBackend>();
    std::string detail;
    const bool healthy = backend->self_test(detail);
    return {healthy ? std::move(backend) : nullptr, healthy, false, detail};
  }

#ifdef EMBEDDED_ROBOT_HAS_CUDA_BACKEND
  try {
    auto cuda_backend = make_cuda_pose_backend();
    std::string detail;
    if (cuda_backend && cuda_backend->self_test(detail)) {
      return {std::move(cuda_backend), true, false, detail};
    }
    if (mode == BackendMode::cuda || require_gpu) {
      return {nullptr, false, false, "CUDA backend self-test failed: " + detail};
    }
    auto cpu_backend = std::make_unique<CpuPoseBackend>();
    std::string cpu_detail;
    const bool cpu_healthy = cpu_backend->self_test(cpu_detail);
    return {cpu_healthy ? std::move(cpu_backend) : nullptr, cpu_healthy, true,
            "CUDA backend self-test failed; " + cpu_detail};
  } catch (const std::exception &error) {
    if (mode == BackendMode::cuda || require_gpu) {
      return {nullptr, false, false, std::string("CUDA backend exception: ") + error.what()};
    }
    auto cpu_backend = std::make_unique<CpuPoseBackend>();
    std::string cpu_detail;
    const bool cpu_healthy = cpu_backend->self_test(cpu_detail);
    return {cpu_healthy ? std::move(cpu_backend) : nullptr, cpu_healthy, true,
            std::string("CUDA unavailable: ") + error.what() + "; " + cpu_detail};
  }
#else
  if (mode == BackendMode::cuda || require_gpu) {
    return {nullptr, false, false, "CUDA backend was not compiled into this image"};
  }
  auto backend = std::make_unique<CpuPoseBackend>();
  std::string detail;
  const bool healthy = backend->self_test(detail);
  return {healthy ? std::move(backend) : nullptr, healthy, true,
          "CUDA backend was not compiled; " + detail};
#endif
}

}  // namespace embedded_robot_perception
