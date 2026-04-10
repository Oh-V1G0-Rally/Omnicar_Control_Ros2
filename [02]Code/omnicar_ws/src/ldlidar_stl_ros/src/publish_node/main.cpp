#include "ros_api.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "ldlidar_driver.h"

namespace {

constexpr float kFullTurnRad = static_cast<float>(2.0 * M_PI);

float normalizeAngle0To2Pi(float angle)
{
  while (angle < 0.0F) {
    angle += kFullTurnRad;
  }
  while (angle >= kFullTurnRad) {
    angle -= kFullTurnRad;
  }
  return angle;
}

class LdLidarPublisher : public rclcpp::Node
{
public:
  LdLidarPublisher()
  : Node("sdpo_driver_laser_2d")
  {
    pose_.base_frame_id = this->declare_parameter<std::string>("base_frame_id", "base_footprint");
    setting_.frame_id = this->declare_parameter<std::string>("laser_frame_id", "laser");
    port_name_ = this->declare_parameter<std::string>("port_name", "/dev/omnicar_lidar");
    serial_port_baudrate_ = this->declare_parameter<int>("port_baudrate", 230400);
    topic_name_ = this->declare_parameter<std::string>("topic_name", "scan");
    pose_.x = this->declare_parameter<double>("laser_pose_x", 0.09);
    pose_.y = this->declare_parameter<double>("laser_pose_y", 0.0);
    pose_.z = this->declare_parameter<double>("laser_pose_z", 0.0);
    pose_.roll = this->declare_parameter<double>("laser_pose_roll", 0.0) * M_PI / 180.0;
    pose_.pitch = this->declare_parameter<double>("laser_pose_pitch", 0.0) * M_PI / 180.0;
    pose_.yaw = this->declare_parameter<double>("laser_pose_yaw", -90.0) * M_PI / 180.0;
    setting_.laser_scan_dir = this->declare_parameter<bool>("laser_scan_dir", true);
    setting_.enable_angle_crop_func = this->declare_parameter<bool>("enable_angle_crop_func", false);
    setting_.angle_crop_min = this->declare_parameter<double>("angle_crop_min", 135.0);
    setting_.angle_crop_max = this->declare_parameter<double>("angle_crop_max", 225.0);
    setting_.range_min = this->declare_parameter<double>("dist_min", 0.02);
    setting_.range_max = this->declare_parameter<double>("dist_max", 12.0);

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_name_, 10);
    pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud>("laser_scan_point_cloud", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    lidar_driver_ = std::make_unique<ldlidar::LDLidarDriver>();
    lidar_driver_->RegisterGetTimestampFunctional(std::bind(&LdLidarPublisher::getSystemTimeStamp));
    lidar_driver_->EnableFilterAlgorithnmProcess(true);

    if (!lidar_driver_->Start(
        ldlidar::LDType::LD_19, port_name_, serial_port_baudrate_, ldlidar::COMM_SERIAL_MODE))
    {
      throw std::runtime_error("Failed to start LD19 lidar on port " + port_name_);
    }

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&LdLidarPublisher::pollLidar, this));
  }

  ~LdLidarPublisher() override
  {
    if (lidar_driver_) {
      lidar_driver_->Stop();
    }
  }

private:
  void pollLidar()
  {
    ldlidar::Points2D laser_scan_points;
    if (lidar_driver_->GetLaserScanData(laser_scan_points, 1500) != ldlidar::LidarStatus::NORMAL) {
      return;
    }

    const auto current_timestamp = this->now();
    publishTransform(current_timestamp);
    publishLidarData(laser_scan_points, current_timestamp);
  }

  void publishTransform(const rclcpp::Time & current_timestamp)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = current_timestamp;
    transform.header.frame_id = pose_.base_frame_id;
    transform.child_frame_id = setting_.frame_id;
    transform.transform.translation.x = pose_.x;
    transform.transform.translation.y = pose_.y;
    transform.transform.translation.z = pose_.z;

    tf2::Quaternion q;
    q.setRPY(pose_.roll, pose_.pitch, pose_.yaw);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(transform);
  }

  void publishLidarData(ldlidar::Points2D & src, const rclcpp::Time & current_timestamp)
  {
    if (src.empty()) {
      return;
    }

    sensor_msgs::msg::PointCloud pc;
    pc.header.stamp = current_timestamp;
    pc.header.frame_id = setting_.frame_id;

    sensor_msgs::msg::LaserScan scan;
    scan.header = pc.header;
    scan.angle_min = 0.0;
    scan.angle_max = kFullTurnRad;
    scan.range_min = static_cast<float>(setting_.range_min);
    scan.range_max = static_cast<float>(setting_.range_max);
    const size_t beam_count = std::max<size_t>(src.size(), 1);
    scan.angle_increment = kFullTurnRad / static_cast<float>(beam_count);
    scan.ranges.assign(beam_count, std::numeric_limits<float>::quiet_NaN());

    for (size_t i = 0; i < src.size(); ++i) {
      const float range = src[i].distance / 1000.0F;
      float angle = -ANGLE_TO_RADIAN(src[i].angle);

      if (!setting_.laser_scan_dir) {
        angle = -angle;
      }

      if (setting_.enable_angle_crop_func) {
        const float angle_deg = angle * 180.0F / static_cast<float>(M_PI);
        if (angle_deg > setting_.angle_crop_min && angle_deg < setting_.angle_crop_max) {
          continue;
        }
      }

      geometry_msgs::msg::Point32 p;
      p.x = range * std::cos(angle);
      p.y = range * std::sin(angle);
      p.z = 0.0F;
      pc.points.push_back(p);

      const float wrapped_angle = normalizeAngle0To2Pi(angle);
      size_t beam_index = static_cast<size_t>(wrapped_angle / scan.angle_increment);
      if (beam_index >= scan.ranges.size()) {
        beam_index = scan.ranges.size() - 1;
      }

      auto & beam_range = scan.ranges[beam_index];
      if (std::isnan(beam_range) || range < beam_range) {
        beam_range = range;
      }
    }

    pc_pub_->publish(pc);
    scan_pub_->publish(scan);
  }

  static uint64_t getSystemTimeStamp()
  {
    const auto tp = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count();
  }

  LaserScanSetting setting_{};
  LaserPose pose_{};
  std::string port_name_;
  int serial_port_baudrate_{};
  std::string topic_name_;
  std::unique_ptr<ldlidar::LDLidarDriver> lidar_driver_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pc_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LdLidarPublisher>());
  } catch (const std::exception & e) {
    auto logger = rclcpp::get_logger("ldlidar_stl_ros");
    RCLCPP_FATAL(logger, "Lidar node failed: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
