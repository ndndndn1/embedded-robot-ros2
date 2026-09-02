#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace embedded_robot_perception {

struct Vec3 {
  double x{};
  double y{};
  double z{};
};

inline Vec3 operator+(const Vec3 &a, const Vec3 &b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(const Vec3 &a, const Vec3 &b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator*(const Vec3 &a, double scale) { return {a.x * scale, a.y * scale, a.z * scale}; }
inline Vec3 operator/(const Vec3 &a, double scale) { return a * (1.0 / scale); }
inline double dot(const Vec3 &a, const Vec3 &b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3 &a, const Vec3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double norm(const Vec3 &value) { return std::sqrt(dot(value, value)); }
inline Vec3 normalized(const Vec3 &value) {
  const double length = norm(value);
  return length > 1e-12 ? value / length : Vec3{};
}

struct Quaternion {
  double w{1.0};
  double x{};
  double y{};
  double z{};
};

struct Pose {
  Vec3 position;
  Quaternion orientation;
};

using Matrix3 = std::array<std::array<double, 3>, 3>;

inline Matrix3 identity_matrix() { return {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}}; }

inline Vec3 multiply(const Matrix3 &matrix, const Vec3 &value) {
  return {
      matrix[0][0] * value.x + matrix[0][1] * value.y + matrix[0][2] * value.z,
      matrix[1][0] * value.x + matrix[1][1] * value.y + matrix[1][2] * value.z,
      matrix[2][0] * value.x + matrix[2][1] * value.y + matrix[2][2] * value.z,
  };
}

inline Matrix3 quaternion_matrix(const Quaternion &input) {
  const double magnitude = std::sqrt(
      input.w * input.w + input.x * input.x + input.y * input.y + input.z * input.z);
  const Quaternion q = magnitude > 1e-12
                           ? Quaternion{input.w / magnitude, input.x / magnitude, input.y / magnitude,
                                        input.z / magnitude}
                           : Quaternion{};
  return {{{1.0 - 2.0 * (q.y * q.y + q.z * q.z), 2.0 * (q.x * q.y - q.z * q.w),
            2.0 * (q.x * q.z + q.y * q.w)},
           {2.0 * (q.x * q.y + q.z * q.w), 1.0 - 2.0 * (q.x * q.x + q.z * q.z),
            2.0 * (q.y * q.z - q.x * q.w)},
           {2.0 * (q.x * q.z - q.y * q.w), 2.0 * (q.y * q.z + q.x * q.w),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y)}}};
}

inline Quaternion matrix_quaternion(const Matrix3 &m) {
  Quaternion q;
  const double trace = m[0][0] + m[1][1] + m[2][2];
  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    q = {0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s,
         (m[1][0] - m[0][1]) / s};
  } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
    const double s = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
    q = {(m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s,
         (m[0][2] + m[2][0]) / s};
  } else if (m[1][1] > m[2][2]) {
    const double s = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
    q = {(m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s,
         (m[1][2] + m[2][1]) / s};
  } else {
    const double s = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
    q = {(m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s,
         (m[1][2] + m[2][1]) / s, 0.25 * s};
  }
  const double magnitude = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  return magnitude > 1e-12 ? Quaternion{q.w / magnitude, q.x / magnitude, q.y / magnitude,
                                        q.z / magnitude}
                           : Quaternion{};
}

struct Transform {
  Matrix3 rotation{identity_matrix()};
  Vec3 translation;

  [[nodiscard]] Vec3 apply(const Vec3 &point) const { return multiply(rotation, point) + translation; }
};

struct Intrinsics {
  std::uint32_t width{};
  std::uint32_t height{};
  double fx{};
  double fy{};
  double cx{};
  double cy{};

  [[nodiscard]] bool valid() const {
    return width > 0 && height > 0 && std::isfinite(fx) && std::isfinite(fy) && fx > 0.0 && fy > 0.0 &&
           std::isfinite(cx) && std::isfinite(cy);
  }
};

struct Rgb8 {
  std::uint8_t r{};
  std::uint8_t g{};
  std::uint8_t b{};
};

struct PointXYZRGB {
  Vec3 position;
  Rgb8 color;
};

struct AlignedFrame {
  std::uint32_t width{};
  std::uint32_t height{};
  std::vector<float> depth_m;
  std::vector<PointXYZRGB> points;
};

struct PoseEstimate {
  bool valid{};
  Pose pose;
  double rms_error_m{std::numeric_limits<double>::infinity()};
  double position_stddev_m{std::numeric_limits<double>::infinity()};
  double orientation_stddev_rad{std::numeric_limits<double>::infinity()};
  std::string detail;
};

struct GraspCandidateCore {
  std::string candidate_id;
  Pose pose;
  Vec3 approach{0.0, 0.0, -1.0};
  double gripper_width_m{0.08};
  double score{1.0};
  double position_stddev_m{};
  double orientation_stddev_rad{};
};

enum class ValidationReason : std::uint8_t {
  ok = 0,
  candidate_not_found = 1,
  stale_scene = 2,
  calibration_mismatch = 3,
  tf_unavailable = 4,
  pose_uncertain = 5,
  collision = 6,
  out_of_workspace = 7,
  reachability_unavailable = 8,
  ik_failed = 9,
  cancelled = 10,
  internal_error = 255,
};

struct GraspValidation {
  bool valid{};
  ValidationReason reason{ValidationReason::internal_error};
  double measured_clearance_m{};
  std::string detail;
};

}  // namespace embedded_robot_perception
