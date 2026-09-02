#include "embedded_robot_perception/grasp_validator.hpp"

#include <limits>

namespace embedded_robot_perception {
namespace {

double point_segment_distance(const Vec3 &point, const Vec3 &start, const Vec3 &end) {
  const Vec3 segment = end - start;
  const double length_squared = dot(segment, segment);
  if (length_squared <= 1e-15) {
    return norm(point - start);
  }
  const double parameter = std::clamp(dot(point - start, segment) / length_squared, 0.0, 1.0);
  return norm(point - (start + segment * parameter));
}

}  // namespace

GraspValidation GraspValidator::validate(
    const GraspCandidateCore &candidate, std::span<const PointXYZRGB> scene_points,
    bool require_reachability, bool reachability_available, bool ik_succeeded) const {
  if (candidate.position_stddev_m > config_.max_position_stddev_m ||
      candidate.orientation_stddev_rad > config_.max_orientation_stddev_rad) {
    return {false, ValidationReason::pose_uncertain, 0.0, "pose uncertainty exceeds limits"};
  }
  const Vec3 position = candidate.pose.position;
  if (position.x < config_.workspace_min.x || position.y < config_.workspace_min.y ||
      position.z < config_.workspace_min.z || position.x > config_.workspace_max.x ||
      position.y > config_.workspace_max.y || position.z > config_.workspace_max.z) {
    return {false, ValidationReason::out_of_workspace, 0.0, "grasp pose is outside workspace"};
  }
  const Vec3 approach = normalized(candidate.approach);
  if (norm(approach) < 0.99) {
    return {false, ValidationReason::internal_error, 0.0, "approach vector is invalid"};
  }
  const Vec3 start = position - approach * config_.approach_distance_m;
  const Vec3 end = position - approach * config_.object_exclusion_radius_m;
  double clearance = std::numeric_limits<double>::infinity();
  for (const auto &point : scene_points) {
    if (!std::isfinite(point.position.x) || !std::isfinite(point.position.y) ||
        !std::isfinite(point.position.z) ||
        norm(point.position - position) <= config_.object_exclusion_radius_m) {
      continue;
    }
    clearance = std::min(clearance, point_segment_distance(point.position, start, end));
  }
  if (clearance < config_.minimum_clearance_m) {
    return {false, ValidationReason::collision, clearance, "approach swept volume is obstructed"};
  }
  if (require_reachability && !reachability_available) {
    return {false, ValidationReason::reachability_unavailable, clearance,
            "MoveIt reachability service is unavailable"};
  }
  if (require_reachability && !ik_succeeded) {
    return {false, ValidationReason::ik_failed, clearance, "IK or state-validity check failed"};
  }
  return {true, ValidationReason::ok, clearance, "geometry and requested reachability checks passed"};
}

}  // namespace embedded_robot_perception
