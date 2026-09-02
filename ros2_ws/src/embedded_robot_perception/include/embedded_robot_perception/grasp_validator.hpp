#pragma once

#include <span>

#include "embedded_robot_perception/types.hpp"

namespace embedded_robot_perception {

struct GraspValidationConfig {
  Vec3 workspace_min{-2.0, -2.0, 0.0};
  Vec3 workspace_max{2.0, 2.0, 2.0};
  double minimum_clearance_m{0.02};
  double approach_distance_m{0.15};
  double object_exclusion_radius_m{0.04};
  double max_position_stddev_m{0.02};
  double max_orientation_stddev_rad{0.15};
};

class GraspValidator {
 public:
  explicit GraspValidator(GraspValidationConfig config = {}) : config_(config) {}

  [[nodiscard]] GraspValidation validate(
      const GraspCandidateCore &candidate, std::span<const PointXYZRGB> scene_points,
      bool require_reachability, bool reachability_available, bool ik_succeeded) const;

 private:
  GraspValidationConfig config_;
};

}  // namespace embedded_robot_perception
