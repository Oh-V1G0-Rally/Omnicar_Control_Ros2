#include "sdpo_ratf_ros_localization/SdpoRatfROSLocalizationROS.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "sdpo_ratf_ros_localization/utils.h"

namespace sdpo_ratf_ros_localization {

SdpoRatfROSLocalizationROS::SdpoRatfROSLocalizationROS()
: Node("sdpo_ratf_ros_localization")
{
  readParam();

  ekf_.initCovP(ekf_cov_ini_p_x_, ekf_cov_ini_p_y_, ekf_cov_ini_p_th_);
  ekf_.initCovQ(ekf_cov_q_d_, ekf_cov_q_dn_, ekf_cov_q_dth_);
  ekf_.initCovR(ekf_cov_r_dist_, ekf_cov_r_ang_);
  ekf_.initPose(ekf_pose_ini_x_, ekf_pose_ini_y_, ekf_pose_ini_th_);
  ekf_.setMode(ekf_mode_ini_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_timer_interface_ = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(tf_timer_interface_);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broad_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  sub_initial_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initial_pose", 5, std::bind(&SdpoRatfROSLocalizationROS::subInitialPose, this, std::placeholders::_1));
  sub_initialpose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 5, std::bind(&SdpoRatfROSLocalizationROS::subInitialPose, this, std::placeholders::_1));
  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&SdpoRatfROSLocalizationROS::subOdom, this, std::placeholders::_1));

  if (use_lidar_for_localization_) {
    sub_laser_point_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
      "laser_scan_point_cloud", 5,
      std::bind(&SdpoRatfROSLocalizationROS::subLaserPointCloud, this, std::placeholders::_1));
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Lidar input disabled for localization: laser_scan_point_cloud will be ignored and remain visualization-only.");
  }

  pub_obs_beacons_ = this->create_publisher<sensor_msgs::msg::PointCloud>("beacons", 5);
  pub_map_beacons_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("map_beacons", 5);
  if (publish_pose_) {
    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 5);
  }

  pubMapBeacons();
}

void SdpoRatfROSLocalizationROS::readParam()
{
  map_frame_id_ = this->declare_parameter<std::string>("map_frame_id", "map");
  odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
  base_frame_id_ = this->declare_parameter<std::string>("base_frame_id", "base_footprint");
  laser_frame_id_ = this->declare_parameter<std::string>("laser_frame_id", "laser");

  beacons_diam_ = this->declare_parameter<double>("beacons_diam", 0.09);
  beacons_valid_dist_ = this->declare_parameter<double>("beacons_valid_dist", 0.20);

  const auto beacons_raw = this->declare_parameter<std::vector<double>>(
    "beacons", std::vector<double>{0.910, 0.552, -0.910, 0.552, 0.910, -0.552, -0.910, -0.552});
  if (beacons_raw.size() % 2 != 0) {
    throw std::runtime_error("beacons parameter must contain an even number of values");
  }

  map_beacons_.clear();
  for (size_t i = 0; i < beacons_raw.size(); i += 2) {
    map_beacons_.push_back({beacons_raw[i], beacons_raw[i + 1]});
  }
  beacons_.assign(map_beacons_.size(), SdpoRatfBeacons{});

  ekf_pose_ini_x_ = this->declare_parameter<double>("ekf_pose_ini_x", 0.0);
  ekf_pose_ini_y_ = this->declare_parameter<double>("ekf_pose_ini_y", 0.0);
  ekf_pose_ini_th_ = deg2rad(this->declare_parameter<double>("ekf_pose_ini_th", 0.0));
  ekf_cov_ini_p_x_ = this->declare_parameter<double>("ekf_cov_ini_p_x", 0.1);
  ekf_cov_ini_p_y_ = this->declare_parameter<double>("ekf_cov_ini_p_y", 0.1);
  ekf_cov_ini_p_th_ = this->declare_parameter<double>("ekf_cov_ini_p_th", 0.1);
  ekf_cov_q_d_ = this->declare_parameter<double>("ekf_cov_q_d", 0.1);
  ekf_cov_q_dn_ = this->declare_parameter<double>("ekf_cov_q_dn", 0.1);
  ekf_cov_q_dth_ = this->declare_parameter<double>("ekf_cov_q_dth", 0.05);
  ekf_cov_r_dist_ = this->declare_parameter<double>("ekf_cov_r_dist", 0.0001);
  ekf_cov_r_ang_ = this->declare_parameter<double>("ekf_cov_r_ang", 0.001);
  ekf_mode_ini_ = this->declare_parameter<std::string>("ekf_mode_ini", kSdpoRatfEKFModeFusionStr);
  publish_pose_ = this->declare_parameter<bool>("publish_pose", true);
  use_lidar_for_localization_ =
    this->declare_parameter<bool>("use_lidar_for_localization", true);
}

void SdpoRatfROSLocalizationROS::subInitialPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (msg->header.frame_id != map_frame_id_) {
    RCLCPP_WARN(
      this->get_logger(), "Ignoring initial pose in frame '%s' (expected '%s')",
      msg->header.frame_id.c_str(), map_frame_id_.c_str());
    return;
  }

  ekf_.initCovP(ekf_cov_ini_p_x_, ekf_cov_ini_p_y_, ekf_cov_ini_p_th_);
  ekf_.initPose(
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    quaternionToYaw(msg->pose.pose.orientation));

  try {
    pubMapTf(msg->header);
    pubPose(msg->header);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to apply initial pose: %s", e.what());
    return;
  }

  is_initial_pose_init_ = true;
}

void SdpoRatfROSLocalizationROS::subLaserPointCloud(
  const sensor_msgs::msg::PointCloud::SharedPtr msg)
{
  if (!is_initial_pose_init_) {
    return;
  }

  try {
    detectBeacons(*msg);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000, "Beacon detection skipped: %s", e.what());
    return;
  }

  bool do_tf_update = false;
  for (size_t i = 0; i < beacons_.size(); ++i) {
    if (beacons_[i].num_pts > 0) {
      do_tf_update = true;
      ekf_.update(beacons_[i], map_beacons_[i]);
    }
  }

  if (do_tf_update) {
    try {
      pubMapTf(msg->header);
      pubPose(msg->header);
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Failed to publish corrected localization TF: %s", e.what());
    }
  }

  pubObsBeacons(msg->header);
}

void SdpoRatfROSLocalizationROS::subOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!is_odom_init_) {
    is_odom_init_ = true;
  } else {
    const double dx = msg->pose.pose.position.x - odom_msg_prev_.pose.pose.position.x;
    const double dy = msg->pose.pose.position.y - odom_msg_prev_.pose.pose.position.y;
    const double prev_yaw = quaternionToYaw(odom_msg_prev_.pose.pose.orientation);
    const double dth = normAngRad(quaternionToYaw(msg->pose.pose.orientation) - prev_yaw);

    const double dd = dx * std::cos(prev_yaw) + dy * std::sin(prev_yaw);
    const double ddn = -dx * std::sin(prev_yaw) + dy * std::cos(prev_yaw);
    ekf_.predict(dd, ddn, dth);
  }

  try {
    pubMapTf(msg->header);
    pubPose(msg->header);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000, "Failed to publish map TF from odom: %s", e.what());
    is_odom_init_ = false;
    return;
  }

  odom_msg_prev_ = *msg;
  is_initial_pose_init_ = true;
}

void SdpoRatfROSLocalizationROS::pubMapBeacons()
{
  visualization_msgs::msg::MarkerArray map;
  map.markers.resize(map_beacons_.size());
  const auto stamp = this->now();

  for (size_t i = 0; i < map_beacons_.size(); ++i) {
    auto & marker = map.markers[i];
    marker.header.frame_id = map_frame_id_;
    marker.header.stamp = stamp;
    marker.ns = "map";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = map_beacons_[i][0];
    marker.pose.position.y = map_beacons_[i][1];
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = beacons_diam_;
    marker.scale.y = beacons_diam_;
    marker.scale.z = 0.5;
    marker.color.a = 0.25;
    marker.color.r = 0.5F;
    marker.color.g = 0.5F;
    marker.color.b = 0.5F;
  }

  pub_map_beacons_->publish(map);
}

void SdpoRatfROSLocalizationROS::pubMapTf(const std_msgs::msg::Header & msg_header)
{
  const auto tf_base_to_odom = tf_buffer_->lookupTransform(
    odom_frame_id_, base_frame_id_, tf2::TimePointZero);

  const double ekf_yaw = ekf_.XR(2);
  const double base_to_odom_yaw = quaternionToYaw(tf_base_to_odom.transform.rotation);

  const double cos_yaw = std::cos(ekf_yaw);
  const double sin_yaw = std::sin(ekf_yaw);
  const double base_odom_x = tf_base_to_odom.transform.translation.x;
  const double base_odom_y = tf_base_to_odom.transform.translation.y;

  geometry_msgs::msg::TransformStamped tf_odom_to_map;
  tf_odom_to_map.header.stamp = msg_header.stamp;
  tf_odom_to_map.header.frame_id = map_frame_id_;
  tf_odom_to_map.child_frame_id = odom_frame_id_;
  tf_odom_to_map.transform.translation.x = ekf_.XR(0) - (cos_yaw * base_odom_x - sin_yaw * base_odom_y);
  tf_odom_to_map.transform.translation.y = ekf_.XR(1) - (sin_yaw * base_odom_x + cos_yaw * base_odom_y);
  tf_odom_to_map.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, normAngRad(ekf_yaw - base_to_odom_yaw));
  tf_odom_to_map.transform.rotation.x = q.x();
  tf_odom_to_map.transform.rotation.y = q.y();
  tf_odom_to_map.transform.rotation.z = q.z();
  tf_odom_to_map.transform.rotation.w = q.w();
  tf_broad_->sendTransform(tf_odom_to_map);
}

void SdpoRatfROSLocalizationROS::pubObsBeacons(const std_msgs::msg::Header & msg_header)
{
  sensor_msgs::msg::PointCloud msg;
  msg.header = msg_header;
  msg.header.frame_id = map_frame_id_;

  for (const auto & beacon : beacons_) {
    if (beacon.num_pts <= 0) {
      continue;
    }

    geometry_msgs::msg::Point32 pt;
    pt.x = static_cast<float>(beacon.x);
    pt.y = static_cast<float>(beacon.y);
    pt.z = 0.0F;
    msg.points.push_back(pt);
  }

  if (!msg.points.empty()) {
    pub_obs_beacons_->publish(msg);
  }
}

void SdpoRatfROSLocalizationROS::pubPose(const std_msgs::msg::Header & msg_header)
{
  if (!publish_pose_ || !pub_pose_) {
    return;
  }

  geometry_msgs::msg::PoseStamped msg;
  msg.header = msg_header;
  msg.header.frame_id = map_frame_id_;
  msg.pose.position.x = ekf_.XR(0);
  msg.pose.position.y = ekf_.XR(1);
  msg.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, ekf_.XR(2));
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();

  pub_pose_->publish(msg);
}

void SdpoRatfROSLocalizationROS::detectBeacons(
  const sensor_msgs::msg::PointCloud & msg)
{
  const auto tf_laser_to_map = tf_buffer_->lookupTransform(
    map_frame_id_, laser_frame_id_, tf2::TimePointZero);
  const double laser_yaw = quaternionToYaw(tf_laser_to_map.transform.rotation);
  const double cos_yaw = std::cos(laser_yaw);
  const double sin_yaw = std::sin(laser_yaw);
  const double tx = tf_laser_to_map.transform.translation.x;
  const double ty = tf_laser_to_map.transform.translation.y;

  for (auto & beacon : beacons_) {
    beacon = SdpoRatfBeacons{};
  }

  for (const auto & point : msg.points) {
    const double pt_x = static_cast<double>(point.x);
    const double pt_y = static_cast<double>(point.y);
    const double pt_dist = dist(pt_x, pt_y);
    const double pt_ang = std::atan2(pt_y, pt_x);
    const double meas_dist = pt_dist + beacons_diam_ / 2.0;
    const double laser_x = meas_dist * std::cos(pt_ang);
    const double laser_y = meas_dist * std::sin(pt_ang);

    const double map_x = tx + cos_yaw * laser_x - sin_yaw * laser_y;
    const double map_y = ty + sin_yaw * laser_x + cos_yaw * laser_y;

    for (size_t j = 0; j < map_beacons_.size(); ++j) {
      if (dist(map_beacons_[j][0] - map_x, map_beacons_[j][1] - map_y) >= beacons_valid_dist_) {
        continue;
      }

      auto & beacon = beacons_[j];
      beacon.num_pts++;
      beacon.x = ((beacon.x * (beacon.num_pts - 1.0)) + map_x) / beacon.num_pts;
      beacon.y = ((beacon.y * (beacon.num_pts - 1.0)) + map_y) / beacon.num_pts;
    }
  }

  for (auto & beacon : beacons_) {
    beacon.dist = dist(beacon.x - ekf_.XR(0), beacon.y - ekf_.XR(1));
    beacon.ang = normAngRad(std::atan2(beacon.y - ekf_.XR(1), beacon.x - ekf_.XR(0)) - ekf_.XR(2));
  }
}

double SdpoRatfROSLocalizationROS::quaternionToYaw(const geometry_msgs::msg::Quaternion & q_msg)
{
  const double siny_cosp = 2.0 * (q_msg.w * q_msg.z + q_msg.x * q_msg.y);
  const double cosy_cosp = 1.0 - 2.0 * (q_msg.y * q_msg.y + q_msg.z * q_msg.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace sdpo_ratf_ros_localization
