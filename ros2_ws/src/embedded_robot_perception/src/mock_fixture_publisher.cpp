#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

#include <embedded_robot_interfaces/msg/calibration_observation.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

namespace {

using namespace std::chrono_literals;

class MockFixturePublisher final : public rclcpp::Node {
 public:
  MockFixturePublisher() : Node("mock_fixture_publisher") {
    robot_id_ = declare_parameter<std::string>("robot_id", "mock-perception-01");
    width_ = static_cast<std::uint32_t>(declare_parameter<int>("width", 64));
    height_ = static_cast<std::uint32_t>(declare_parameter<int>("height", 48));
    if (width_ < 16 || height_ < 16) {
      throw std::invalid_argument("fixture dimensions must be at least 16x16");
    }
    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(4);
    const auto reliable_qos = rclcpp::QoS(4).reliable();
    color_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "sensors/front_rgbd/color/image_raw", sensor_qos);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "sensors/front_rgbd/depth/image_raw", sensor_qos);
    color_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "sensors/front_rgbd/color/camera_info", reliable_qos);
    depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "sensors/front_rgbd/depth/camera_info", reliable_qos);
    observation_pub_ = create_publisher<embedded_robot_interfaces::msg::CalibrationObservation>(
        "perception/calibration_observation", reliable_qos);
    broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    publish_static_transforms();
    timer_ = create_wall_timer(100ms, [this]() { publish_frame(); });
  }

 private:
  void publish_static_transforms() {
    geometry_msgs::msg::TransformStamped base_from_color;
    base_from_color.header.stamp = now();
    base_from_color.header.frame_id = robot_id_ + "/base_link";
    base_from_color.child_frame_id = robot_id_ + "/front_rgbd_color_optical_frame";
    base_from_color.transform.rotation.w = 1.0;
    geometry_msgs::msg::TransformStamped color_from_depth;
    color_from_depth.header = base_from_color.header;
    color_from_depth.header.frame_id = base_from_color.child_frame_id;
    color_from_depth.child_frame_id = robot_id_ + "/front_rgbd_depth_optical_frame";
    color_from_depth.transform.rotation.w = 1.0;
    broadcaster_->sendTransform({base_from_color, color_from_depth});
  }

  sensor_msgs::msg::CameraInfo camera_info(
      const builtin_interfaces::msg::Time &stamp, const std::string &frame) const {
    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = stamp;
    info.header.frame_id = frame;
    info.width = width_;
    info.height = height_;
    info.distortion_model = "plumb_bob";
    info.d.assign(5, 0.0);
    const double focal = 80.0;
    info.k = {focal, 0.0, (width_ - 1.0) / 2.0, 0.0, focal, (height_ - 1.0) / 2.0,
              0.0, 0.0, 1.0};
    info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info.p = {focal, 0.0, (width_ - 1.0) / 2.0, 0.0, 0.0, focal,
              (height_ - 1.0) / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0};
    return info;
  }

  void publish_frame() {
    const auto stamp = now();
    const std::string color_frame = robot_id_ + "/front_rgbd_color_optical_frame";
    const std::string depth_frame = robot_id_ + "/front_rgbd_depth_optical_frame";
    sensor_msgs::msg::Image color;
    color.header.stamp = stamp;
    color.header.frame_id = color_frame;
    color.height = height_;
    color.width = width_;
    color.encoding = sensor_msgs::image_encodings::RGB8;
    color.step = width_ * 3U;
    color.data.assign(static_cast<std::size_t>(color.step) * height_, 20U);
    for (std::uint32_t row = height_ / 3; row < height_ * 2 / 3; ++row) {
      for (std::uint32_t column = width_ / 3; column < width_ * 2 / 3; ++column) {
        const std::size_t offset = static_cast<std::size_t>(row) * color.step + column * 3U;
        color.data[offset] = 220U;
        color.data[offset + 1] = 20U;
        color.data[offset + 2] = 20U;
      }
    }
    sensor_msgs::msg::Image depth;
    depth.header.stamp = stamp;
    depth.header.frame_id = depth_frame;
    depth.height = height_;
    depth.width = width_;
    depth.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    depth.step = width_ * sizeof(std::uint16_t);
    depth.data.resize(static_cast<std::size_t>(depth.step) * height_);
    for (std::size_t offset = 0; offset < depth.data.size(); offset += sizeof(std::uint16_t)) {
      const std::uint16_t value = 800U;
      std::memcpy(depth.data.data() + offset, &value, sizeof(value));
    }
    color_pub_->publish(color);
    depth_pub_->publish(depth);
    color_info_pub_->publish(camera_info(stamp, color_frame));
    depth_info_pub_->publish(camera_info(stamp, depth_frame));

    embedded_robot_interfaces::msg::CalibrationObservation observation;
    observation.header.stamp = stamp;
    observation.header.frame_id = robot_id_ + "/base_link";
    observation.camera_name = "front_rgbd";
    observation.target_id = "mock-charuco-7x5";
    observation.camera_serial = "MOCK-D455-0001";
    observation.stream_profile =
        std::to_string(width_) + "x" + std::to_string(height_) + "@10";
    observation.target_from_camera.header = observation.header;
    observation.target_from_camera.child_frame_id = color_frame;
    observation.target_from_camera.transform.rotation.w = 1.0;
    observation.reprojection_error_px = 0.4F;
    observation_pub_->publish(observation);
  }

  std::string robot_id_;
  std::uint32_t width_{};
  std::uint32_t height_{};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
  rclcpp::Publisher<embedded_robot_interfaces::msg::CalibrationObservation>::SharedPtr observation_pub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockFixturePublisher>());
  rclcpp::shutdown();
  return 0;
}
