#include "embedded_robot_perception/frame_aligner.hpp"

#include <stdexcept>

namespace embedded_robot_perception {
namespace {

template <typename Depth, typename Scale>
AlignedFrame align_impl(
    std::span<const Depth> depth, std::span<const Rgb8> color, const Intrinsics &depth_intrinsics,
    const Intrinsics &color_intrinsics, const Transform &color_from_depth, Scale scale) {
  if (!depth_intrinsics.valid() || !color_intrinsics.valid()) {
    throw std::invalid_argument("camera intrinsics are invalid");
  }
  if (depth.size() != static_cast<std::size_t>(depth_intrinsics.width) * depth_intrinsics.height) {
    throw std::invalid_argument("depth buffer size does not match intrinsics");
  }
  if (color.size() != static_cast<std::size_t>(color_intrinsics.width) * color_intrinsics.height) {
    throw std::invalid_argument("color buffer size does not match intrinsics");
  }

  AlignedFrame result;
  result.width = color_intrinsics.width;
  result.height = color_intrinsics.height;
  result.depth_m.assign(
      static_cast<std::size_t>(result.width) * result.height, std::numeric_limits<float>::quiet_NaN());
  result.points.reserve(depth.size());

  for (std::uint32_t row = 0; row < depth_intrinsics.height; ++row) {
    for (std::uint32_t column = 0; column < depth_intrinsics.width; ++column) {
      const std::size_t source_index = static_cast<std::size_t>(row) * depth_intrinsics.width + column;
      const double z = scale(depth[source_index]);
      if (!std::isfinite(z) || z <= 0.0) {
        continue;
      }
      const Vec3 depth_point{
          (static_cast<double>(column) - depth_intrinsics.cx) * z / depth_intrinsics.fx,
          (static_cast<double>(row) - depth_intrinsics.cy) * z / depth_intrinsics.fy,
          z,
      };
      const Vec3 color_point = color_from_depth.apply(depth_point);
      if (!std::isfinite(color_point.z) || color_point.z <= 0.0) {
        continue;
      }
      const auto projected_column = static_cast<long>(std::llround(
          color_intrinsics.fx * color_point.x / color_point.z + color_intrinsics.cx));
      const auto projected_row = static_cast<long>(std::llround(
          color_intrinsics.fy * color_point.y / color_point.z + color_intrinsics.cy));
      if (projected_column < 0 || projected_row < 0 ||
          projected_column >= static_cast<long>(color_intrinsics.width) ||
          projected_row >= static_cast<long>(color_intrinsics.height)) {
        continue;
      }
      const std::size_t target_index = static_cast<std::size_t>(projected_row) * color_intrinsics.width +
                                       static_cast<std::size_t>(projected_column);
      float &target_depth = result.depth_m[target_index];
      if (!std::isfinite(target_depth) || color_point.z < target_depth) {
        target_depth = static_cast<float>(color_point.z);
      }
      result.points.push_back({color_point, color[target_index]});
    }
  }
  return result;
}

}  // namespace

AlignedFrame FrameAligner::align_u16_mm(
    std::span<const std::uint16_t> depth_mm, std::span<const Rgb8> color,
    const Intrinsics &depth_intrinsics, const Intrinsics &color_intrinsics,
    const Transform &color_from_depth) const {
  return align_impl(depth_mm, color, depth_intrinsics, color_intrinsics, color_from_depth,
                    [](std::uint16_t value) { return static_cast<double>(value) * 0.001; });
}

AlignedFrame FrameAligner::align_f32_m(
    std::span<const float> depth_m, std::span<const Rgb8> color,
    const Intrinsics &depth_intrinsics, const Intrinsics &color_intrinsics,
    const Transform &color_from_depth) const {
  return align_impl(depth_m, color, depth_intrinsics, color_intrinsics, color_from_depth,
                    [](float value) { return static_cast<double>(value); });
}

}  // namespace embedded_robot_perception
