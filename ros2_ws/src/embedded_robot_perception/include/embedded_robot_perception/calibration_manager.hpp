#pragma once

#include <cstddef>
#include <span>
#include <string>

#include "embedded_robot_perception/types.hpp"

namespace embedded_robot_perception {

struct CalibrationSample {
  Transform target_from_camera;
  double reprojection_error_px{};
};

struct CalibrationResult {
  bool valid{};
  Transform target_from_camera;
  double rms_reprojection_error_px{};
  double translation_spread_m{};
  double rotation_spread_rad{};
  std::size_t samples_used{};
  std::string calibration_id;
  std::string detail;
};

class CalibrationManager {
 public:
  [[nodiscard]] CalibrationResult finalize(
      std::span<const CalibrationSample> samples, std::size_t required_samples,
      double maximum_reprojection_error_px, const std::string &canonical_provenance) const;
};

}  // namespace embedded_robot_perception
