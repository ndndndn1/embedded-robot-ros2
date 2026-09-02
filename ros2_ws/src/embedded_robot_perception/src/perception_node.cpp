#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <embedded_robot_interfaces/action/calibrate_extrinsics.hpp>
#include <embedded_robot_interfaces/action/validate_grasp.hpp>
#include <embedded_robot_interfaces/msg/calibration_observation.hpp>
#include <embedded_robot_interfaces/msg/grasp_candidate_array.hpp>
#include <embedded_robot_interfaces/msg/perception_frame_info.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include "embedded_robot_perception/calibration_manager.hpp"
#include "embedded_robot_perception/frame_aligner.hpp"
#include "embedded_robot_perception/grasp_validator.hpp"
#include "embedded_robot_perception/pose_backend.hpp"

namespace embedded_robot_perception {
namespace {

using namespace std::chrono_literals;

Transform transform_from_message(const geometry_msgs::msg::Transform &message) {
  const Quaternion quaternion{
      message.rotation.w, message.rotation.x, message.rotation.y, message.rotation.z};
  const double magnitude = std::sqrt(
      quaternion.w * quaternion.w + quaternion.x * quaternion.x + quaternion.y * quaternion.y +
      quaternion.z * quaternion.z);
  if (!std::isfinite(magnitude) || magnitude < 0.999 || magnitude > 1.001) {
    throw std::invalid_argument("TF quaternion is not normalized");
  }
  return {quaternion_matrix(quaternion),
          {message.translation.x, message.translation.y, message.translation.z}};
}

geometry_msgs::msg::Quaternion quaternion_message(const Quaternion &value) {
  geometry_msgs::msg::Quaternion message;
  message.w = value.w;
  message.x = value.x;
  message.y = value.y;
  message.z = value.z;
  return message;
}

geometry_msgs::msg::Pose pose_message(const Pose &value) {
  geometry_msgs::msg::Pose message;
  message.position.x = value.position.x;
  message.position.y = value.position.y;
  message.position.z = value.position.z;
  message.orientation = quaternion_message(value.orientation);
  return message;
}

Intrinsics intrinsics_from_message(const sensor_msgs::msg::CameraInfo &message) {
  return {message.width, message.height, message.k[0], message.k[4], message.k[2], message.k[5]};
}

BackendMode parse_backend_mode(const std::string &value) {
  if (value == "auto") {
    return BackendMode::automatic;
  }
  if (value == "cpu") {
    return BackendMode::cpu;
  }
  if (value == "cuda") {
    return BackendMode::cuda;
  }
  throw std::invalid_argument("backend must be auto, cpu, or cuda");
}

}  // namespace

class PerceptionNode final : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit PerceptionNode(const rclcpp::NodeOptions &options)
      : rclcpp_lifecycle::LifecycleNode("embedded_robot_perception", options) {
    robot_id_ = declare_parameter<std::string>("robot_id", "mock-perception-01");
    camera_name_ = declare_parameter<std::string>("camera_name", "front_rgbd");
    base_frame_ = declare_parameter<std::string>("base_frame", robot_id_ + "/base_link");
    backend_name_ = declare_parameter<std::string>("backend", "auto");
    require_gpu_ = declare_parameter<bool>("require_gpu", false);
    model_id_ = declare_parameter<std::string>("model_id", "red-cuboid-v1");
    model_sha256_ = declare_parameter<std::string>(
        "model_sha256", "fixture-only-no-production-model");
    max_sync_skew_ms_ = declare_parameter<double>("max_sync_skew_ms", 15.0);
    mock_reachability_ = declare_parameter<bool>("mock_reachability", false);
    mock_ik_success_ = declare_parameter<bool>("mock_ik_success", false);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    try {
      auto selection = select_backend(parse_backend_mode(backend_name_), require_gpu_);
      if (!selection.healthy || !selection.backend) {
        RCLCPP_ERROR(get_logger(), "backend configure failed: %s", selection.detail.c_str());
        return CallbackReturn::FAILURE;
      }
      backend_ = std::move(selection.backend);
      backend_degraded_ = selection.degraded;
      backend_detail_ = selection.detail;
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "configuration rejected: %s", error.what());
      return CallbackReturn::FAILURE;
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(4);
    const auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(4)).reliable().durability_volatile();
    color_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "sensors/front_rgbd/color/image_raw", sensor_qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
          { std::scoped_lock lock(input_mutex_); color_ = std::move(message); }
          process_if_ready();
        });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "sensors/front_rgbd/depth/image_raw", sensor_qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
          { std::scoped_lock lock(input_mutex_); depth_ = std::move(message); }
          process_if_ready();
        });
    color_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "sensors/front_rgbd/color/camera_info", reliable_qos,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
          std::scoped_lock lock(input_mutex_);
          color_info_ = std::move(message);
        });
    depth_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "sensors/front_rgbd/depth/camera_info", reliable_qos,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
          std::scoped_lock lock(input_mutex_);
          depth_info_ = std::move(message);
        });
    calibration_observation_sub_ = create_subscription<
        embedded_robot_interfaces::msg::CalibrationObservation>(
        "perception/calibration_observation", reliable_qos,
        [this](embedded_robot_interfaces::msg::CalibrationObservation::ConstSharedPtr message) {
          std::scoped_lock lock(calibration_mutex_);
          if (calibration_observations_.size() >= 256) {
            calibration_observations_.erase(calibration_observations_.begin());
          }
          calibration_observations_.push_back(*message);
        });

    aligned_depth_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "perception/aligned_depth/image", sensor_qos);
    aligned_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "perception/aligned_points", sensor_qos);
    detections_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>(
        "perception/detections_3d", rclcpp::QoS(5).reliable());
    grasps_pub_ = create_publisher<embedded_robot_interfaces::msg::GraspCandidateArray>(
        "perception/grasp_candidates", rclcpp::QoS(5).reliable());
    frame_info_pub_ = create_publisher<embedded_robot_interfaces::msg::PerceptionFrameInfo>(
        "perception/frame_info", rclcpp::QoS(5).reliable());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "perception/diagnostics", rclcpp::QoS(1).reliable().transient_local());

    set_camera_info_service_ = create_service<sensor_msgs::srv::SetCameraInfo>(
        "sensors/front_rgbd/set_camera_info",
        [this](const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> request,
               std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> response) {
          const auto intrinsics = intrinsics_from_message(request->camera_info);
          if (!intrinsics.valid() || request->camera_info.header.frame_id.empty()) {
            response->success = false;
            response->status_message = "invalid camera geometry or frame_id";
            return;
          }
          std::scoped_lock lock(input_mutex_);
          color_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(request->camera_info);
          response->success = true;
          response->status_message = "camera info accepted in memory; persist via the camera driver";
        });

    create_action_servers();
    publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::OK, backend_detail_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    aligned_depth_pub_->on_activate();
    aligned_points_pub_->on_activate();
    detections_pub_->on_activate();
    grasps_pub_->on_activate();
    frame_info_pub_->on_activate();
    diagnostics_pub_->on_activate();
    publish_diagnostic(
        backend_degraded_ ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                          : diagnostic_msgs::msg::DiagnosticStatus::OK,
        backend_detail_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    aligned_depth_pub_->on_deactivate();
    aligned_points_pub_->on_deactivate();
    detections_pub_->on_deactivate();
    grasps_pub_->on_deactivate();
    frame_info_pub_->on_deactivate();
    diagnostics_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    std::scoped_lock input_lock(input_mutex_);
    color_.reset();
    depth_.reset();
    color_info_.reset();
    depth_info_.reset();
    latest_candidate_.reset();
    latest_scene_.clear();
    backend_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    return CallbackReturn::SUCCESS;
  }

 private:
  using Calibrate = embedded_robot_interfaces::action::CalibrateExtrinsics;
  using CalibrateHandle = rclcpp_action::ServerGoalHandle<Calibrate>;
  using Validate = embedded_robot_interfaces::action::ValidateGrasp;
  using ValidateHandle = rclcpp_action::ServerGoalHandle<Validate>;

  void create_action_servers() {
    calibrate_server_ = rclcpp_action::create_server<Calibrate>(
        get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(),
        get_node_waitables_interface(), "perception/calibrate_extrinsics",
        [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Calibrate::Goal> goal) {
          return goal->required_samples >= 3 && !goal->camera_frame.empty() &&
                         !goal->target_frame.empty() && goal->max_reprojection_error_px > 0.0F
                     ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                     : rclcpp_action::GoalResponse::REJECT;
        },
        [](const std::shared_ptr<CalibrateHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](const std::shared_ptr<CalibrateHandle> handle) { execute_calibration(handle); });
    validate_server_ = rclcpp_action::create_server<Validate>(
        get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(),
        get_node_waitables_interface(), "perception/validate_grasp",
        [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Validate::Goal> goal) {
          return !goal->candidate_id.empty() && !goal->calibration_id.empty()
                     ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                     : rclcpp_action::GoalResponse::REJECT;
        },
        [](const std::shared_ptr<ValidateHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](const std::shared_ptr<ValidateHandle> handle) { execute_validation(handle); });
  }

  void execute_calibration(const std::shared_ptr<CalibrateHandle> handle) {
    const auto goal = handle->get_goal();
    auto result = std::make_shared<Calibrate::Result>();
    if (handle->is_canceling()) {
      result->failure_code = Calibrate::Result::CANCELLED;
      result->detail = "calibration cancelled";
      handle->canceled(result);
      return;
    }
    std::vector<embedded_robot_interfaces::msg::CalibrationObservation> observations;
    {
      std::scoped_lock lock(calibration_mutex_);
      for (const auto &observation : calibration_observations_) {
        if (observation.camera_name == goal->camera_name && observation.target_id == goal->target_id &&
            observation.target_from_camera.header.frame_id == goal->target_frame &&
            observation.target_from_camera.child_frame_id == goal->camera_frame) {
          observations.push_back(observation);
        }
      }
    }
    std::vector<CalibrationSample> samples;
    samples.reserve(observations.size());
    std::string provenance = goal->camera_name + '|' + goal->camera_frame + '|' + goal->target_frame +
                             '|' + goal->target_id + '|';
    try {
      for (const auto &observation : observations) {
        samples.push_back(
            {transform_from_message(observation.target_from_camera.transform),
             observation.reprojection_error_px});
        provenance += observation.camera_serial + '|' + observation.stream_profile + '|';
      }
    } catch (const std::exception &error) {
      result->failure_code = Calibrate::Result::INVALID_GOAL;
      result->detail = error.what();
      handle->abort(result);
      return;
    }
    const auto calibration = calibration_manager_.finalize(
        samples, goal->required_samples, goal->max_reprojection_error_px, provenance);
    result->success = calibration.valid;
    result->calibration_id = calibration.calibration_id;
    result->rms_reprojection_error_px = static_cast<float>(calibration.rms_reprojection_error_px);
    result->samples_used = static_cast<std::uint32_t>(calibration.samples_used);
    result->detail = calibration.detail;
    result->extrinsics.header.stamp = now();
    result->extrinsics.header.frame_id = goal->target_frame;
    result->extrinsics.child_frame_id = goal->camera_frame;
    result->extrinsics.transform.translation.x = calibration.target_from_camera.translation.x;
    result->extrinsics.transform.translation.y = calibration.target_from_camera.translation.y;
    result->extrinsics.transform.translation.z = calibration.target_from_camera.translation.z;
    result->extrinsics.transform.rotation =
        quaternion_message(matrix_quaternion(calibration.target_from_camera.rotation));
    if (!calibration.valid) {
      result->failure_code = samples.size() < goal->required_samples
                                 ? Calibrate::Result::INSUFFICIENT_SAMPLES
                                 : Calibrate::Result::REPROJECTION_ERROR;
      handle->abort(result);
      return;
    }
    {
      std::scoped_lock lock(input_mutex_);
      calibration_id_ = calibration.calibration_id;
    }
    result->failure_code = Calibrate::Result::OK;
    handle->succeed(result);
  }

  void execute_validation(const std::shared_ptr<ValidateHandle> handle) {
    const auto goal = handle->get_goal();
    auto result = std::make_shared<Validate::Result>();
    if (handle->is_canceling()) {
      result->reason_code = Validate::Result::CANCELLED;
      result->detail = "validation cancelled";
      handle->canceled(result);
      return;
    }
    std::optional<GraspCandidateCore> candidate;
    std::vector<PointXYZRGB> scene;
    std::string calibration_id;
    std::uint64_t sequence = 0;
    rclcpp::Time scene_stamp{0, 0, get_clock()->get_clock_type()};
    {
      std::scoped_lock lock(input_mutex_);
      candidate = latest_candidate_;
      scene = latest_scene_;
      calibration_id = calibration_id_;
      sequence = scene_sequence_;
      scene_stamp = latest_scene_stamp_;
    }
    if (!candidate || candidate->candidate_id != goal->candidate_id) {
      result->reason_code = Validate::Result::CANDIDATE_NOT_FOUND;
      result->detail = "candidate is not in the current bounded scene";
      handle->abort(result);
      return;
    }
    if (goal->calibration_id != calibration_id || goal->scene_sequence != sequence) {
      result->reason_code = Validate::Result::CALIBRATION_MISMATCH;
      result->detail = "scene sequence or calibration provenance mismatch";
      handle->abort(result);
      return;
    }
    const rclcpp::Duration maximum_age(goal->max_scene_age);
    if (maximum_age.nanoseconds() <= 0 || now() - scene_stamp > maximum_age) {
      result->reason_code = Validate::Result::STALE_SCENE;
      result->detail = "scene is older than the requested limit";
      handle->abort(result);
      return;
    }
    GraspValidationConfig config;
    config.minimum_clearance_m = std::max(0.001, static_cast<double>(goal->minimum_clearance_m));
    const auto validation = GraspValidator(config).validate(
        *candidate, scene, goal->require_reachability, mock_reachability_, mock_ik_success_);
    result->valid = validation.valid;
    result->reason_code = static_cast<std::uint8_t>(validation.reason);
    result->detail = validation.detail;
    result->measured_clearance_m = static_cast<float>(validation.measured_clearance_m);
    result->pose_confidence = static_cast<float>(candidate->score);
    result->validated_pose.header.stamp = scene_stamp;
    result->validated_pose.header.frame_id = base_frame_;
    result->validated_pose.pose = pose_message(candidate->pose);
    validation.valid ? handle->succeed(result) : handle->abort(result);
  }

  void process_if_ready() {
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return;
    }
    sensor_msgs::msg::Image::ConstSharedPtr color;
    sensor_msgs::msg::Image::ConstSharedPtr depth;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr color_info;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr depth_info;
    {
      std::scoped_lock lock(input_mutex_);
      color = color_;
      depth = depth_;
      color_info = color_info_;
      depth_info = depth_info_;
    }
    if (!color || !depth || !color_info || !depth_info || !backend_) {
      return;
    }
    const rclcpp::Time depth_stamp(depth->header.stamp, get_clock()->get_clock_type());
    {
      std::scoped_lock lock(input_mutex_);
      if (depth_stamp == last_processed_stamp_) {
        return;
      }
      last_processed_stamp_ = depth_stamp;
    }
    const double skew_ms = std::abs(
        (rclcpp::Time(color->header.stamp, get_clock()->get_clock_type()) - depth_stamp).seconds() *
        1000.0);
    if (skew_ms > max_sync_skew_ms_ || color->header.frame_id != color_info->header.frame_id ||
        depth->header.frame_id != depth_info->header.frame_id) {
      publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                         "frame mismatch or RGB-depth skew exceeds configured limit");
      return;
    }
    try {
      const auto color_from_depth_message = tf_buffer_->lookupTransform(
          color->header.frame_id, depth->header.frame_id, depth_stamp, 50ms);
      const auto base_from_color_message = tf_buffer_->lookupTransform(
          base_frame_, color->header.frame_id, depth_stamp, 50ms);
      const Transform color_from_depth = transform_from_message(color_from_depth_message.transform);
      const Transform base_from_color = transform_from_message(base_from_color_message.transform);
      const auto color_pixels = unpack_color(*color);
      AlignedFrame aligned;
      if (depth->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        aligned = aligner_.align_u16_mm(
            unpack_depth<std::uint16_t>(*depth), color_pixels, intrinsics_from_message(*depth_info),
            intrinsics_from_message(*color_info), color_from_depth);
      } else if (depth->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
        aligned = aligner_.align_f32_m(
            unpack_depth<float>(*depth), color_pixels, intrinsics_from_message(*depth_info),
            intrinsics_from_message(*color_info), color_from_depth);
      } else {
        throw std::invalid_argument("depth encoding must be 16UC1 or 32FC1");
      }
      for (auto &point : aligned.points) {
        point.position = base_from_color.apply(point.position);
      }
      publish_frame(*color, aligned, backend_->estimate_cluster(aligned.points));
    } catch (const tf2::TransformException &error) {
      publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                         std::string("TF unavailable at acquisition time: ") + error.what());
    } catch (const std::exception &error) {
      publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR, error.what());
    }
  }

  std::vector<Rgb8> unpack_color(const sensor_msgs::msg::Image &image) const {
    if (image.encoding != sensor_msgs::image_encodings::RGB8 &&
        image.encoding != sensor_msgs::image_encodings::BGR8) {
      throw std::invalid_argument("color encoding must be rgb8 or bgr8");
    }
    if (image.step < image.width * 3U || image.data.size() < static_cast<std::size_t>(image.step) * image.height) {
      throw std::invalid_argument("color image buffer is truncated");
    }
    std::vector<Rgb8> pixels(static_cast<std::size_t>(image.width) * image.height);
    for (std::uint32_t row = 0; row < image.height; ++row) {
      for (std::uint32_t column = 0; column < image.width; ++column) {
        const std::size_t input = static_cast<std::size_t>(row) * image.step + column * 3U;
        const std::size_t output = static_cast<std::size_t>(row) * image.width + column;
        if (image.encoding == sensor_msgs::image_encodings::RGB8) {
          pixels[output] = {image.data[input], image.data[input + 1], image.data[input + 2]};
        } else {
          pixels[output] = {image.data[input + 2], image.data[input + 1], image.data[input]};
        }
      }
    }
    return pixels;
  }

  template <typename Value>
  std::vector<Value> unpack_depth(const sensor_msgs::msg::Image &image) const {
    if (image.is_bigendian != 0U) {
      throw std::invalid_argument("big-endian depth images are not supported");
    }
    if (image.step < image.width * sizeof(Value) ||
        image.data.size() < static_cast<std::size_t>(image.step) * image.height) {
      throw std::invalid_argument("depth image buffer is truncated");
    }
    std::vector<Value> values(static_cast<std::size_t>(image.width) * image.height);
    for (std::uint32_t row = 0; row < image.height; ++row) {
      std::memcpy(values.data() + static_cast<std::size_t>(row) * image.width,
                  image.data.data() + static_cast<std::size_t>(row) * image.step,
                  static_cast<std::size_t>(image.width) * sizeof(Value));
    }
    return values;
  }

  void publish_frame(
      const sensor_msgs::msg::Image &source, const AlignedFrame &aligned,
      const PoseEstimate &estimate) {
    sensor_msgs::msg::Image aligned_depth;
    aligned_depth.header = source.header;
    aligned_depth.height = aligned.height;
    aligned_depth.width = aligned.width;
    aligned_depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    aligned_depth.is_bigendian = false;
    aligned_depth.step = aligned.width * sizeof(float);
    aligned_depth.data.resize(aligned.depth_m.size() * sizeof(float));
    std::memcpy(aligned_depth.data.data(), aligned.depth_m.data(), aligned_depth.data.size());
    aligned_depth_pub_->publish(aligned_depth);

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = source.header.stamp;
    cloud.header.frame_id = base_frame_;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(aligned.points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> r(cloud, "r");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> g(cloud, "g");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> b(cloud, "b");
    for (const auto &point : aligned.points) {
      *x = static_cast<float>(point.position.x);
      *y = static_cast<float>(point.position.y);
      *z = static_cast<float>(point.position.z);
      *r = point.color.r;
      *g = point.color.g;
      *b = point.color.b;
      ++x;
      ++y;
      ++z;
      ++r;
      ++g;
      ++b;
    }
    aligned_points_pub_->publish(cloud);

    std::uint64_t sequence = 0;
    std::string calibration_id;
    {
      std::scoped_lock lock(input_mutex_);
      sequence = ++scene_sequence_;
      if (calibration_id_.empty()) {
        calibration_id_ = "uncalibrated";
      }
      calibration_id = calibration_id_;
    }
    embedded_robot_interfaces::msg::PerceptionFrameInfo frame_info;
    frame_info.header = cloud.header;
    frame_info.scene_sequence = sequence;
    frame_info.calibration_id = calibration_id;
    frame_info.model_id = model_id_;
    frame_info.model_sha256 = model_sha256_;
    frame_info.backend = backend_->name();
    frame_info.pipeline_version = "1.0.0";
    frame_info_pub_->publish(frame_info);

    vision_msgs::msg::Detection3DArray detections;
    detections.header = cloud.header;
    embedded_robot_interfaces::msg::GraspCandidateArray grasps;
    grasps.header = cloud.header;
    grasps.calibration_id = calibration_id;
    grasps.scene_sequence = sequence;
    std::optional<GraspCandidateCore> candidate;
    if (estimate.valid) {
      vision_msgs::msg::Detection3D detection;
      detection.header = cloud.header;
      detection.id = "red-object-0";
      detection.bbox.center = pose_message(estimate.pose);
      detection.bbox.size.x = 0.08;
      detection.bbox.size.y = 0.05;
      detection.bbox.size.z = 0.04;
      vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
      hypothesis.hypothesis.class_id = model_id_;
      hypothesis.hypothesis.score = std::clamp(1.0 - estimate.rms_error_m * 10.0, 0.0, 1.0);
      hypothesis.pose.pose = pose_message(estimate.pose);
      hypothesis.pose.covariance[0] = estimate.position_stddev_m * estimate.position_stddev_m;
      hypothesis.pose.covariance[7] = hypothesis.pose.covariance[0];
      hypothesis.pose.covariance[14] = hypothesis.pose.covariance[0];
      hypothesis.pose.covariance[21] =
          estimate.orientation_stddev_rad * estimate.orientation_stddev_rad;
      hypothesis.pose.covariance[28] = hypothesis.pose.covariance[21];
      hypothesis.pose.covariance[35] = hypothesis.pose.covariance[21];
      detection.results.push_back(hypothesis);
      detections.detections.push_back(detection);

      candidate = GraspCandidateCore{
          "grasp-" + std::to_string(sequence), estimate.pose, {0.0, 0.0, -1.0}, 0.08,
          hypothesis.hypothesis.score, estimate.position_stddev_m, estimate.orientation_stddev_rad};
      embedded_robot_interfaces::msg::GraspCandidate message;
      message.header = cloud.header;
      message.candidate_id = candidate->candidate_id;
      message.detection_id = detection.id;
      message.calibration_id = calibration_id;
      message.scene_sequence = sequence;
      message.grasp_pose = pose_message(candidate->pose);
      message.approach.x = candidate->approach.x;
      message.approach.y = candidate->approach.y;
      message.approach.z = candidate->approach.z;
      message.gripper_width_m = static_cast<float>(candidate->gripper_width_m);
      message.score = static_cast<float>(candidate->score);
      message.position_stddev_m = static_cast<float>(candidate->position_stddev_m);
      message.orientation_stddev_rad = static_cast<float>(candidate->orientation_stddev_rad);
      grasps.candidates.push_back(message);
    }
    detections_pub_->publish(detections);
    grasps_pub_->publish(grasps);
    {
      std::scoped_lock lock(input_mutex_);
      latest_candidate_ = candidate;
      latest_scene_ = aligned.points;
      latest_scene_stamp_ = rclcpp::Time(source.header.stamp, get_clock()->get_clock_type());
    }
    publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::OK,
                       estimate.valid ? estimate.detail : "frame aligned; no object pose");
  }

  void publish_diagnostic(std::uint8_t level, const std::string &message) {
    if (!diagnostics_pub_ || !diagnostics_pub_->is_activated()) {
      return;
    }
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = level;
    status.name = robot_id_ + "/embedded_robot_perception";
    status.hardware_id = camera_name_;
    status.message = message;
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  std::string robot_id_;
  std::string camera_name_;
  std::string base_frame_;
  std::string backend_name_;
  std::string model_id_;
  std::string model_sha256_;
  std::string calibration_id_;
  bool require_gpu_{};
  bool mock_reachability_{};
  bool mock_ik_success_{};
  bool backend_degraded_{};
  double max_sync_skew_ms_{15.0};
  std::string backend_detail_;
  std::uint64_t scene_sequence_{};
  rclcpp::Time latest_scene_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_processed_stamp_{0, 0, RCL_ROS_TIME};

  std::mutex input_mutex_;
  std::mutex calibration_mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr color_;
  sensor_msgs::msg::Image::ConstSharedPtr depth_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr color_info_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr depth_info_;
  std::optional<GraspCandidateCore> latest_candidate_;
  std::vector<PointXYZRGB> latest_scene_;
  std::vector<embedded_robot_interfaces::msg::CalibrationObservation> calibration_observations_;

  FrameAligner aligner_;
  CalibrationManager calibration_manager_;
  std::unique_ptr<PoseBackend> backend_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
  rclcpp::Subscription<embedded_robot_interfaces::msg::CalibrationObservation>::SharedPtr
      calibration_observation_sub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr aligned_depth_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_points_pub_;
  rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection3DArray>::SharedPtr detections_pub_;
  rclcpp_lifecycle::LifecyclePublisher<embedded_robot_interfaces::msg::GraspCandidateArray>::SharedPtr
      grasps_pub_;
  rclcpp_lifecycle::LifecyclePublisher<embedded_robot_interfaces::msg::PerceptionFrameInfo>::SharedPtr
      frame_info_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Service<sensor_msgs::srv::SetCameraInfo>::SharedPtr set_camera_info_service_;
  rclcpp_action::Server<Calibrate>::SharedPtr calibrate_server_;
  rclcpp_action::Server<Validate>::SharedPtr validate_server_;
};

}  // namespace embedded_robot_perception

RCLCPP_COMPONENTS_REGISTER_NODE(embedded_robot_perception::PerceptionNode)
