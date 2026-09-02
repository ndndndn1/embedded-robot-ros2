#include "embedded_robot_perception/calibration_manager.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "embedded_robot_perception/sha256.hpp"

namespace embedded_robot_perception {

CalibrationResult CalibrationManager::finalize(
    std::span<const CalibrationSample> samples, std::size_t required_samples,
    double maximum_reprojection_error_px, const std::string &canonical_provenance) const {
  CalibrationResult result;
  if (required_samples < 3 || samples.size() < required_samples) {
    result.detail = "insufficient independent calibration samples";
    return result;
  }
  Vec3 translation_sum;
  Quaternion quaternion_sum{};
  quaternion_sum = {0.0, 0.0, 0.0, 0.0};
  Quaternion reference = matrix_quaternion(samples.front().target_from_camera.rotation);
  double reprojection_square_sum = 0.0;
  for (const auto &sample : samples) {
    if (!std::isfinite(sample.reprojection_error_px) || sample.reprojection_error_px < 0.0) {
      result.detail = "calibration sample has invalid reprojection error";
      return result;
    }
    translation_sum = translation_sum + sample.target_from_camera.translation;
    Quaternion q = matrix_quaternion(sample.target_from_camera.rotation);
    if (q.w * reference.w + q.x * reference.x + q.y * reference.y + q.z * reference.z < 0.0) {
      q = {-q.w, -q.x, -q.y, -q.z};
    }
    quaternion_sum.w += q.w;
    quaternion_sum.x += q.x;
    quaternion_sum.y += q.y;
    quaternion_sum.z += q.z;
    reprojection_square_sum += sample.reprojection_error_px * sample.reprojection_error_px;
  }
  const double count = static_cast<double>(samples.size());
  const Vec3 mean_translation = translation_sum / count;
  const double quaternion_norm = std::sqrt(
      quaternion_sum.w * quaternion_sum.w + quaternion_sum.x * quaternion_sum.x +
      quaternion_sum.y * quaternion_sum.y + quaternion_sum.z * quaternion_sum.z);
  if (quaternion_norm <= 1e-12) {
    result.detail = "calibration quaternion average is degenerate";
    return result;
  }
  const Quaternion mean_quaternion{
      quaternion_sum.w / quaternion_norm, quaternion_sum.x / quaternion_norm,
      quaternion_sum.y / quaternion_norm, quaternion_sum.z / quaternion_norm};
  for (const auto &sample : samples) {
    result.translation_spread_m = std::max(
        result.translation_spread_m, norm(sample.target_from_camera.translation - mean_translation));
    const Quaternion q = matrix_quaternion(sample.target_from_camera.rotation);
    const double cosine = std::clamp(
        std::abs(q.w * mean_quaternion.w + q.x * mean_quaternion.x + q.y * mean_quaternion.y +
                 q.z * mean_quaternion.z),
        0.0, 1.0);
    result.rotation_spread_rad = std::max(result.rotation_spread_rad, 2.0 * std::acos(cosine));
  }
  result.rms_reprojection_error_px = std::sqrt(reprojection_square_sum / count);
  result.samples_used = samples.size();
  result.target_from_camera = {quaternion_matrix(mean_quaternion), mean_translation};
  if (result.rms_reprojection_error_px > maximum_reprojection_error_px) {
    result.detail = "RMS reprojection error exceeds goal limit";
    return result;
  }
  if (result.translation_spread_m > 0.005 || result.rotation_spread_rad > 0.00872664626) {
    result.detail = "extrinsic sample spread exceeds 5 mm or 0.5 degree";
    return result;
  }
  std::ostringstream canonical;
  canonical << canonical_provenance << std::fixed << std::setprecision(12) << mean_translation.x << ','
            << mean_translation.y << ',' << mean_translation.z << ',' << mean_quaternion.w << ','
            << mean_quaternion.x << ',' << mean_quaternion.y << ',' << mean_quaternion.z;
  result.calibration_id = sha256(canonical.str());
  result.valid = true;
  result.detail = "calibration accepted";
  return result;
}

}  // namespace embedded_robot_perception
