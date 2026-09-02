#include "embedded_robot_perception/pose_backend.hpp"

#include <algorithm>
#include <array>
#include <numeric>

namespace embedded_robot_perception {
namespace {

Vec3 centroid(std::span<const Vec3> points) {
  Vec3 sum;
  for (const auto &point : points) {
    sum = sum + point;
  }
  return sum / static_cast<double>(points.size());
}

Quaternion dominant_quaternion(const std::array<std::array<double, 4>, 4> &matrix) {
  auto values = matrix;
  std::array<std::array<double, 4>, 4> vectors{{
      {{1.0, 0.0, 0.0, 0.0}},
      {{0.0, 1.0, 0.0, 0.0}},
      {{0.0, 0.0, 1.0, 0.0}},
      {{0.0, 0.0, 0.0, 1.0}},
  }};
  for (int iteration = 0; iteration < 64; ++iteration) {
    std::size_t p = 0;
    std::size_t q = 1;
    double largest = std::abs(values[p][q]);
    for (std::size_t row = 0; row < 4; ++row) {
      for (std::size_t column = row + 1; column < 4; ++column) {
        if (std::abs(values[row][column]) > largest) {
          largest = std::abs(values[row][column]);
          p = row;
          q = column;
        }
      }
    }
    if (largest < 1e-14) {
      break;
    }
    const double angle = 0.5 * std::atan2(2.0 * values[p][q], values[q][q] - values[p][p]);
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    for (std::size_t index = 0; index < 4; ++index) {
      const double vip = values[index][p];
      const double viq = values[index][q];
      values[index][p] = cosine * vip - sine * viq;
      values[index][q] = sine * vip + cosine * viq;
    }
    for (std::size_t index = 0; index < 4; ++index) {
      const double vpi = values[p][index];
      const double vqi = values[q][index];
      values[p][index] = cosine * vpi - sine * vqi;
      values[q][index] = sine * vpi + cosine * vqi;
    }
    for (std::size_t index = 0; index < 4; ++index) {
      const double eip = vectors[index][p];
      const double eiq = vectors[index][q];
      vectors[index][p] = cosine * eip - sine * eiq;
      vectors[index][q] = sine * eip + cosine * eiq;
    }
  }
  std::size_t maximum = 0;
  for (std::size_t index = 1; index < 4; ++index) {
    if (values[index][index] > values[maximum][maximum]) {
      maximum = index;
    }
  }
  std::array<double, 4> result{
      vectors[0][maximum], vectors[1][maximum], vectors[2][maximum], vectors[3][maximum]};
  if (result[0] < 0.0) {
    for (double &component : result) {
      component = -component;
    }
  }
  return {result[0], result[1], result[2], result[3]};
}

void jacobi_eigenvectors(Matrix3 matrix, Matrix3 &vectors, std::array<double, 3> &values) {
  vectors = identity_matrix();
  for (int iteration = 0; iteration < 32; ++iteration) {
    std::size_t p = 0;
    std::size_t q = 1;
    double largest = std::abs(matrix[0][1]);
    for (std::size_t row = 0; row < 3; ++row) {
      for (std::size_t column = row + 1; column < 3; ++column) {
        if (std::abs(matrix[row][column]) > largest) {
          largest = std::abs(matrix[row][column]);
          p = row;
          q = column;
        }
      }
    }
    if (largest < 1e-12) {
      break;
    }
    const double angle = 0.5 * std::atan2(2.0 * matrix[p][q], matrix[q][q] - matrix[p][p]);
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    for (std::size_t index = 0; index < 3; ++index) {
      const double mip = matrix[index][p];
      const double miq = matrix[index][q];
      matrix[index][p] = cosine * mip - sine * miq;
      matrix[index][q] = sine * mip + cosine * miq;
    }
    for (std::size_t index = 0; index < 3; ++index) {
      const double mpi = matrix[p][index];
      const double mqi = matrix[q][index];
      matrix[p][index] = cosine * mpi - sine * mqi;
      matrix[q][index] = sine * mpi + cosine * mqi;
    }
    for (std::size_t index = 0; index < 3; ++index) {
      const double vip = vectors[index][p];
      const double viq = vectors[index][q];
      vectors[index][p] = cosine * vip - sine * viq;
      vectors[index][q] = sine * vip + cosine * viq;
    }
  }
  values = {matrix[0][0], matrix[1][1], matrix[2][2]};
}

}  // namespace

bool CpuPoseBackend::self_test(std::string &detail) {
  const std::array<Vec3, 4> model{{{0.0, 0.0, 0.0}, {0.1, 0.0, 0.0}, {0.0, 0.2, 0.0},
                                   {0.0, 0.0, 0.3}}};
  const Quaternion rotation{std::cos(0.2), 0.0, 0.0, std::sin(0.2)};
  const Transform transform{quaternion_matrix(rotation), {0.4, -0.2, 0.8}};
  std::array<Vec3, 4> observed{};
  std::transform(model.begin(), model.end(), observed.begin(),
                 [&transform](const Vec3 &point) { return transform.apply(point); });
  const PoseEstimate estimate = estimate_correspondences(model, observed);
  const double translation_error = norm(estimate.pose.position - transform.translation);
  if (!estimate.valid || translation_error > 1e-6 || estimate.rms_error_m > 1e-6) {
    detail = "CPU rigid-pose self-test failed";
    return false;
  }
  detail = "CPU rigid-pose self-test passed";
  return true;
}

PoseEstimate CpuPoseBackend::estimate_correspondences(
    std::span<const Vec3> model_points, std::span<const Vec3> observed_points) {
  PoseEstimate estimate;
  if (model_points.size() != observed_points.size() || model_points.size() < 3) {
    estimate.detail = "at least three matched keypoints are required";
    return estimate;
  }
  const Vec3 model_centroid = centroid(model_points);
  const Vec3 observed_centroid = centroid(observed_points);
  Matrix3 covariance{};
  for (std::size_t index = 0; index < model_points.size(); ++index) {
    const Vec3 model = model_points[index] - model_centroid;
    const Vec3 observed = observed_points[index] - observed_centroid;
    const std::array<double, 3> a{model.x, model.y, model.z};
    const std::array<double, 3> b{observed.x, observed.y, observed.z};
    for (std::size_t row = 0; row < 3; ++row) {
      for (std::size_t column = 0; column < 3; ++column) {
        covariance[row][column] += a[row] * b[column];
      }
    }
  }
  const double sxx = covariance[0][0];
  const double sxy = covariance[0][1];
  const double sxz = covariance[0][2];
  const double syx = covariance[1][0];
  const double syy = covariance[1][1];
  const double syz = covariance[1][2];
  const double szx = covariance[2][0];
  const double szy = covariance[2][1];
  const double szz = covariance[2][2];
  const std::array<std::array<double, 4>, 4> horn{{
      {{sxx + syy + szz, syz - szy, szx - sxz, sxy - syx}},
      {{syz - szy, sxx - syy - szz, sxy + syx, szx + sxz}},
      {{szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy}},
      {{sxy - syx, szx + sxz, syz + szy, -sxx - syy + szz}},
  }};
  estimate.pose.orientation = dominant_quaternion(horn);
  const Matrix3 rotation = quaternion_matrix(estimate.pose.orientation);
  estimate.pose.position = observed_centroid - multiply(rotation, model_centroid);

  double squared_error = 0.0;
  for (std::size_t index = 0; index < model_points.size(); ++index) {
    const Vec3 predicted = multiply(rotation, model_points[index]) + estimate.pose.position;
    const Vec3 residual = predicted - observed_points[index];
    squared_error += dot(residual, residual);
  }
  estimate.rms_error_m = std::sqrt(squared_error / static_cast<double>(model_points.size()));
  estimate.position_stddev_m = estimate.rms_error_m;
  double radius = 0.0;
  for (const auto &point : model_points) {
    radius = std::max(radius, norm(point - model_centroid));
  }
  estimate.orientation_stddev_rad = radius > 1e-9 ? estimate.rms_error_m / radius : 1.0;
  estimate.valid = std::isfinite(estimate.rms_error_m);
  estimate.detail = estimate.valid ? "rigid keypoint pose estimated" : "non-finite pose estimate";
  return estimate;
}

PoseEstimate CpuPoseBackend::estimate_cluster(std::span<const PointXYZRGB> points) {
  PoseEstimate estimate;
  std::vector<Vec3> selected;
  selected.reserve(points.size());
  for (const auto &point : points) {
    if (point.color.r > 128 && point.color.r > static_cast<int>(point.color.g) * 3 / 2 &&
        point.color.r > static_cast<int>(point.color.b) * 3 / 2 && std::isfinite(point.position.x) &&
        std::isfinite(point.position.y) && std::isfinite(point.position.z)) {
      selected.push_back(point.position);
    }
  }
  if (selected.size() < 8) {
    estimate.detail = "fewer than eight red-object points";
    return estimate;
  }
  const Vec3 center = centroid(selected);
  Matrix3 covariance{};
  for (const auto &point : selected) {
    const Vec3 delta = point - center;
    const std::array<double, 3> d{delta.x, delta.y, delta.z};
    for (std::size_t row = 0; row < 3; ++row) {
      for (std::size_t column = 0; column < 3; ++column) {
        covariance[row][column] += d[row] * d[column];
      }
    }
  }
  Matrix3 eigenvectors{};
  std::array<double, 3> eigenvalues{};
  jacobi_eigenvectors(covariance, eigenvectors, eigenvalues);
  std::array<std::size_t, 3> order{0, 1, 2};
  std::sort(order.begin(), order.end(), [&eigenvalues](std::size_t a, std::size_t b) {
    return eigenvalues[a] > eigenvalues[b];
  });
  Vec3 x_axis{eigenvectors[0][order[0]], eigenvectors[1][order[0]], eigenvectors[2][order[0]]};
  Vec3 y_axis{eigenvectors[0][order[1]], eigenvectors[1][order[1]], eigenvectors[2][order[1]]};
  x_axis = normalized(x_axis);
  y_axis = normalized(y_axis - x_axis * dot(x_axis, y_axis));
  Vec3 z_axis = normalized(cross(x_axis, y_axis));
  y_axis = normalized(cross(z_axis, x_axis));
  if (x_axis.x < 0.0) {
    x_axis = x_axis * -1.0;
    y_axis = y_axis * -1.0;
  }
  const Matrix3 rotation{{{x_axis.x, y_axis.x, z_axis.x},
                          {x_axis.y, y_axis.y, z_axis.y},
                          {x_axis.z, y_axis.z, z_axis.z}}};
  estimate.valid = true;
  estimate.pose = {center, matrix_quaternion(rotation)};
  estimate.rms_error_m = 0.0;
  estimate.position_stddev_m = 0.002;
  estimate.orientation_stddev_rad = 0.05;
  estimate.detail = "red-object PCA pose estimated";
  return estimate;
}

}  // namespace embedded_robot_perception
