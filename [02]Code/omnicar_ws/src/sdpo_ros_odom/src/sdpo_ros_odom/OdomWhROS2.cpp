#include "sdpo_ros_odom/OdomWhROS2.h"

#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include "sdpo_ros_odom/OdomWhOmni4.h"

namespace sdpo_ros_odom {

OdomWhROS2::OdomWhROS2()
: Node("sdpo_ros_odom")
{
  readParameters();

  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
  sub_mot_enc_ = this->create_subscription<sdpo_drivers_interfaces::msg::MotEncArray>(
    "motors_enc", 10, std::bind(&OdomWhROS2::subMotEnc, this, std::placeholders::_1));
  tf_broad_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void OdomWhROS2::readParameters()
{
  const std::string steering_geometry =
    this->declare_parameter<std::string>("steering_geometry", kOdomWhOmni4Str);
  base_frame_id_ = this->declare_parameter<std::string>("base_frame_id", "base_footprint");
  odom_frame_id_ = this->declare_parameter<std::string>("odom_frame_id", "odom");
  publish_tf_ = this->declare_parameter<bool>("publish_tf", true);
  invert_odom_x_ = this->declare_parameter<bool>("invert_odom_x", false);
  invert_odom_y_ = this->declare_parameter<bool>("invert_odom_y", false);
  invert_odom_yaw_ = this->declare_parameter<bool>("invert_odom_yaw", false);
  suppress_encoder_pure_rotation_translation_ =
    this->declare_parameter<bool>("suppress_encoder_pure_rotation_translation", true);
  pure_rotation_min_abs_ticks_ =
    this->declare_parameter<int>("pure_rotation_min_abs_ticks", 2);
  translation_covariance_ = this->declare_parameter<double>("translation_covariance", 0.0001);
  rotation_covariance_ = this->declare_parameter<double>("rotation_covariance", 0.001);

  if (steering_geometry != kOdomWhOmni4Str) {
    throw std::runtime_error("Only omni4 steering_geometry is currently supported in ROS2 odom");
  }

  std::vector<double> rob_len(2);
  rob_len[OdomWhOmni4::kRobLenIdxF2B] =
    this->declare_parameter<double>("rob_dist_between_front_back_wh", 0.185);
  rob_len[OdomWhOmni4::kRobLenIdxL2R] =
    this->declare_parameter<double>("rob_dist_between_left_right_wh", 0.220);

  std::vector<double> wh_d(4);
  wh_d[OdomWhOmni4::kWhIdxFL] = this->declare_parameter<double>("wh_front_left_diam", 0.06);
  wh_d[OdomWhOmni4::kWhIdxFR] = this->declare_parameter<double>("wh_front_right_diam", 0.06);
  wh_d[OdomWhOmni4::kWhIdxBL] = this->declare_parameter<double>("wh_back_left_diam", 0.06);
  wh_d[OdomWhOmni4::kWhIdxBR] = this->declare_parameter<double>("wh_back_right_diam", 0.06);

  std::vector<size_t> wh_idx(4);
  wh_idx[OdomWhOmni4::kWhIdxFL] =
    static_cast<size_t>(this->declare_parameter<int>("wh_front_left_idx", 0));
  wh_idx[OdomWhOmni4::kWhIdxFR] =
    static_cast<size_t>(this->declare_parameter<int>("wh_front_right_idx", 1));
  wh_idx[OdomWhOmni4::kWhIdxBL] =
    static_cast<size_t>(this->declare_parameter<int>("wh_back_left_idx", 2));
  wh_idx[OdomWhOmni4::kWhIdxBR] =
    static_cast<size_t>(this->declare_parameter<int>("wh_back_right_idx", 3));

  std::vector<bool> wh_inv(4);
  wh_inv[OdomWhOmni4::kWhIdxFL] = this->declare_parameter<bool>("wh_front_left_inv", false);
  wh_inv[OdomWhOmni4::kWhIdxFR] = this->declare_parameter<bool>("wh_front_right_inv", true);
  wh_inv[OdomWhOmni4::kWhIdxBL] = this->declare_parameter<bool>("wh_back_left_inv", false);
  wh_inv[OdomWhOmni4::kWhIdxBR] = this->declare_parameter<bool>("wh_back_right_inv", true);

  odom_ = std::make_unique<OdomWhOmni4>(wh_idx, wh_d, wh_inv, rob_len);
}

void OdomWhROS2::subMotEnc(const sdpo_drivers_interfaces::msg::MotEncArray::SharedPtr msg)
{
  try {
    for (size_t i = 0; i < msg->mot_enc.size(); ++i) {
      odom_->setMotorDriveEncTicksDelta(i, msg->mot_enc[i].enc_delta, msg->mot_enc[i].ticks_per_rev);
      odom_->setMotorDriveW(i, msg->mot_enc[i].ang_speed);
    }

    odom_->update();

    if (suppress_encoder_pure_rotation_translation_ && isPureRotationPattern(msg)) {
      odom_->pose.x -= odom_->odo.x_delta;
      odom_->pose.y -= odom_->odo.y_delta;
      odom_->odo.x_delta = 0.0;
      odom_->odo.y_delta = 0.0;
      odom_->vel.v = 0.0;
      odom_->vel.vn = 0.0;
    }

    const double pose_x = invert_odom_x_ ? -odom_->pose.x : odom_->pose.x;
    const double pose_y = invert_odom_y_ ? -odom_->pose.y : odom_->pose.y;
    const double pose_yaw = invert_odom_yaw_ ? -odom_->pose.th : odom_->pose.th;
    const double vel_x = invert_odom_x_ ? -odom_->vel.v : odom_->vel.v;
    const double vel_y = invert_odom_y_ ? -odom_->vel.vn : odom_->vel.vn;
    const double vel_yaw = invert_odom_yaw_ ? -odom_->vel.w : odom_->vel.w;

    geometry_msgs::msg::Twist odom_vel;
    odom_vel.linear.x = vel_x;
    odom_vel.linear.y = vel_y;
    odom_vel.angular.z = vel_yaw;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose_yaw);

    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = msg->stamp;
    odom_tf.header.frame_id = odom_frame_id_;
    odom_tf.child_frame_id = base_frame_id_;
    odom_tf.transform.translation.x = pose_x;
    odom_tf.transform.translation.y = pose_y;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation.x = q.x();
    odom_tf.transform.rotation.y = q.y();
    odom_tf.transform.rotation.z = q.z();
    odom_tf.transform.rotation.w = q.w();

    if (publish_tf_) {
      tf_broad_->sendTransform(odom_tf);
    }

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header = odom_tf.header;
    odom_msg.child_frame_id = odom_tf.child_frame_id;
    odom_msg.pose.pose.position.x = pose_x;
    odom_msg.pose.pose.position.y = pose_y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_tf.transform.rotation;
    odom_msg.twist.twist = odom_vel;
    odom_msg.pose.covariance[0] = translation_covariance_;
    odom_msg.pose.covariance[7] = translation_covariance_;
    odom_msg.pose.covariance[14] = 1000.0;
    odom_msg.pose.covariance[21] = 1000.0;
    odom_msg.pose.covariance[28] = 1000.0;
    odom_msg.pose.covariance[35] = rotation_covariance_;
    pub_odom_->publish(odom_msg);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to process motors_enc message: %s", e.what());
  }
}

bool OdomWhROS2::isPureRotationPattern(
  const sdpo_drivers_interfaces::msg::MotEncArray::SharedPtr msg) const
{
  if (msg->mot_enc.size() < 4) {
    return false;
  }

  const int fl = msg->mot_enc[0].enc_delta;
  const int fr = msg->mot_enc[1].enc_delta;
  const int bl = msg->mot_enc[2].enc_delta;
  const int br = msg->mot_enc[3].enc_delta;

  const auto strong = [this](int v) {
      return std::abs(v) >= pure_rotation_min_abs_ticks_;
    };

  if (!strong(fl) || !strong(fr) || !strong(bl) || !strong(br)) {
    return false;
  }

  const bool pattern_cw = fl > 0 && fr < 0 && bl > 0 && br < 0;
  const bool pattern_ccw = fl < 0 && fr > 0 && bl < 0 && br > 0;

  return pattern_cw || pattern_ccw;
}

}  // namespace sdpo_ros_odom
