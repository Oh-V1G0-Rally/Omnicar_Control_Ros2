#pragma once

#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sdpo_drivers_interfaces/msg/mot_enc_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "sdpo_ros_odom/OdomWh.h"

namespace sdpo_ros_odom {

class OdomWhROS2 : public rclcpp::Node
{
public:
  OdomWhROS2();

private:
  void readParameters();
  void subMotEnc(const sdpo_drivers_interfaces::msg::MotEncArray::SharedPtr msg);
  bool isPureRotationPattern(const sdpo_drivers_interfaces::msg::MotEncArray::SharedPtr msg) const;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Subscription<sdpo_drivers_interfaces::msg::MotEncArray>::SharedPtr sub_mot_enc_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broad_;

  std::unique_ptr<OdomWh> odom_;
  std::string base_frame_id_;
  std::string odom_frame_id_;
  bool publish_tf_ = true;
  bool invert_odom_x_ = false;
  bool invert_odom_y_ = false;
  bool invert_odom_yaw_ = false;
  bool suppress_encoder_pure_rotation_translation_ = true;
  int pure_rotation_min_abs_ticks_ = 2;
  double translation_covariance_ = 0.0001;
  double rotation_covariance_ = 0.001;
};

}  // namespace sdpo_ros_odom
