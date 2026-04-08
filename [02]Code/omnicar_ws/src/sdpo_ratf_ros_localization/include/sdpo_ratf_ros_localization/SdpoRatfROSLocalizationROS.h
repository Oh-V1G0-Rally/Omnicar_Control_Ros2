#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "sdpo_ratf_ros_localization/SdpoRatfBeacons.h"
#include "sdpo_ratf_ros_localization/SdpoRatfEKF.h"

namespace sdpo_ratf_ros_localization {

class SdpoRatfROSLocalizationROS : public rclcpp::Node
{
public:
  SdpoRatfROSLocalizationROS();

private:
  void readParam();

  void subInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void subLaserPointCloud(const sensor_msgs::msg::PointCloud::SharedPtr msg);
  void subOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

  void pubMapBeacons();
  void pubMapTf(const std_msgs::msg::Header & msg_header);
  void pubObsBeacons(const std_msgs::msg::Header & msg_header);
  void pubPose(const std_msgs::msg::Header & msg_header);
  void detectBeacons(const sensor_msgs::msg::PointCloud & msg);

  static double quaternionToYaw(const geometry_msgs::msg::Quaternion & q_msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr sub_laser_point_cloud_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_obs_beacons_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_map_beacons_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broad_;
  std::shared_ptr<tf2_ros::CreateTimerROS> tf_timer_interface_;

  std::string map_frame_id_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string laser_frame_id_;

  std::vector<std::array<double, 2>> map_beacons_;
  double beacons_diam_;
  double beacons_valid_dist_;
  std::vector<SdpoRatfBeacons> beacons_;

  SdpoRatfEKF ekf_;
  double ekf_pose_ini_x_;
  double ekf_pose_ini_y_;
  double ekf_pose_ini_th_;
  double ekf_cov_ini_p_x_;
  double ekf_cov_ini_p_y_;
  double ekf_cov_ini_p_th_;
  double ekf_cov_q_d_;
  double ekf_cov_q_dn_;
  double ekf_cov_q_dth_;
  double ekf_cov_r_dist_;
  double ekf_cov_r_ang_;
  std::string ekf_mode_ini_;
  bool publish_pose_;
  bool use_lidar_for_localization_ = true;

  bool is_initial_pose_init_ = false;
  bool is_odom_init_ = false;

  nav_msgs::msg::Odometry odom_msg_prev_;
};

}  // namespace sdpo_ratf_ros_localization
