#pragma once

#include <span>

#include "embedded_robot_perception/types.hpp"

namespace embedded_robot_perception {

class FrameAligner {
 public:
  [[nodiscard]] AlignedFrame align_u16_mm(
      std::span<const std::uint16_t> depth_mm, std::span<const Rgb8> color,
      const Intrinsics &depth_intrinsics, const Intrinsics &color_intrinsics,
      const Transform &color_from_depth) const;

  [[nodiscard]] AlignedFrame align_f32_m(
      std::span<const float> depth_m, std::span<const Rgb8> color,
      const Intrinsics &depth_intrinsics, const Intrinsics &color_intrinsics,
      const Transform &color_from_depth) const;
};

}  // namespace embedded_robot_perception
