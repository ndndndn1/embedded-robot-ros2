#include "embedded_robot_perception/pose_backend.hpp"

#include <cuda_runtime.h>

#include <array>
#include <memory>
#include <stdexcept>
#include <string>

#ifdef EMBEDDED_ROBOT_HAS_TENSORRT
#include <NvInferVersion.h>
#endif

namespace embedded_robot_perception {
namespace {

void check_cuda(cudaError_t status, const char *operation) {
  if (status != cudaSuccess) {
    throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(status));
  }
}

__global__ void vector_add_one(float *values, std::size_t count) {
  const std::size_t index = static_cast<std::size_t>(blockIdx.x) * blockDim.x + threadIdx.x;
  if (index < count) {
    values[index] += 1.0F;
  }
}

class CudaBuffer final {
 public:
  explicit CudaBuffer(std::size_t bytes) { check_cuda(cudaMalloc(&data_, bytes), "cudaMalloc"); }
  ~CudaBuffer() { cudaFree(data_); }
  CudaBuffer(const CudaBuffer &) = delete;
  CudaBuffer &operator=(const CudaBuffer &) = delete;
  [[nodiscard]] void *get() const { return data_; }

 private:
  void *data_{};
};

class CudaPoseBackend final : public PoseBackend {
 public:
  CudaPoseBackend() {
    check_cuda(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking), "cudaStreamCreate");
    scratch_ = std::make_unique<CudaBuffer>(sizeof(float) * 4);
  }

  ~CudaPoseBackend() override {
    if (stream_ != nullptr) {
      cudaStreamSynchronize(stream_);
      scratch_.reset();
      cudaStreamDestroy(stream_);
    }
  }

  [[nodiscard]] std::string name() const override {
#ifdef EMBEDDED_ROBOT_HAS_TENSORRT
    return "cuda-tensorrt";
#else
    return "cuda-preprocess-cpu-pose";
#endif
  }

  [[nodiscard]] bool self_test(std::string &detail) override {
    int device_count = 0;
    if (cudaGetDeviceCount(&device_count) != cudaSuccess || device_count < 1) {
      detail = "no CUDA device";
      return false;
    }
    const std::array<float, 4> input{0.0F, 1.0F, 2.0F, 3.0F};
    std::array<float, 4> output{};
    check_cuda(cudaMemcpyAsync(scratch_->get(), input.data(), sizeof(input), cudaMemcpyHostToDevice, stream_),
               "cudaMemcpyAsync host-to-device");
    vector_add_one<<<1, 32, 0, stream_>>>(static_cast<float *>(scratch_->get()), input.size());
    check_cuda(cudaGetLastError(), "CUDA self-test kernel");
    check_cuda(cudaMemcpyAsync(output.data(), scratch_->get(), sizeof(output), cudaMemcpyDeviceToHost, stream_),
               "cudaMemcpyAsync device-to-host");
    check_cuda(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");
    if (output != std::array<float, 4>{1.0F, 2.0F, 3.0F, 4.0F}) {
      detail = "CUDA self-test result mismatch";
      return false;
    }
    std::string cpu_detail;
    if (!cpu_.self_test(cpu_detail)) {
      detail = cpu_detail;
      return false;
    }
#ifdef EMBEDDED_ROBOT_HAS_TENSORRT
    detail = "CUDA kernel and TensorRT ABI self-test passed, TensorRT version " +
             std::to_string(NV_TENSORRT_MAJOR) + "." + std::to_string(NV_TENSORRT_MINOR);
#else
    detail = "CUDA preprocessing self-test passed; TensorRT engine support is not compiled";
#endif
    return true;
  }

  [[nodiscard]] PoseEstimate estimate_correspondences(
      std::span<const Vec3> model_points, std::span<const Vec3> observed_points) override {
    // The lightweight backend validates CUDA lifecycle and owns reusable device memory. Rigid
    // refinement remains numerically identical to CPU until a versioned TensorRT model is mounted.
    return cpu_.estimate_correspondences(model_points, observed_points);
  }

  [[nodiscard]] PoseEstimate estimate_cluster(std::span<const PointXYZRGB> points) override {
    return cpu_.estimate_cluster(points);
  }

 private:
  cudaStream_t stream_{};
  std::unique_ptr<CudaBuffer> scratch_;
  CpuPoseBackend cpu_;
};

}  // namespace

std::unique_ptr<PoseBackend> make_cuda_pose_backend() { return std::make_unique<CudaPoseBackend>(); }

}  // namespace embedded_robot_perception
