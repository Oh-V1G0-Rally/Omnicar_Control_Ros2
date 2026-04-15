#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sdpo_motion_control/msg/go_to_point_debug.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

namespace sdpo_motion_control
{

class GoToPointControllerNode : public rclcpp::Node
{
public:
  GoToPointControllerNode()
  : Node("go_to_point_controller")
  {
    pose_source_ = declare_parameter<std::string>("pose_source", "odom");
    control_frame_id_ = declare_parameter<std::string>("control_frame_id", "odom");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "odom");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "pose");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "goal_pose");
    simple_goal_topic_ = declare_parameter<std::string>("simple_goal_topic", "goal_xyyaw");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    goal_reached_topic_ = declare_parameter<std::string>("goal_reached_topic", "goal_reached");
    error_topic_ = declare_parameter<std::string>("error_topic", "control_error");
    debug_topic_ = declare_parameter<std::string>("debug_topic", "go_to_point_debug");
    publish_debug_topics_ = declare_parameter<bool>("publish_debug_topics", true);
    publish_zero_when_inactive_ = declare_parameter<bool>("publish_zero_when_inactive", true);
    stop_at_goal_ = declare_parameter<bool>("stop_at_goal", true);

    const double control_rate_hz = declare_parameter<double>("control_rate_hz", 30.0);
    debug_publish_rate_hz_ = declare_parameter<double>("debug_publish_rate_hz", 20.0);

    use_initial_goal_ = declare_parameter<bool>("use_initial_goal", false);
    initial_goal_x_ = declare_parameter<double>("initial_goal_x", 0.0);
    initial_goal_y_ = declare_parameter<double>("initial_goal_y", 0.0);
    initial_goal_yaw_ = declare_parameter<double>("initial_goal_yaw", 0.0);
    initial_goal_frame_id_ =
      declare_parameter<std::string>("initial_goal_frame_id", control_frame_id_);

    kp_x_ = declare_parameter<double>("kp_x", 1.2);
    ki_x_ = declare_parameter<double>("ki_x", 0.0);
    kd_x_ = declare_parameter<double>("kd_x", 0.0);
    kp_y_ = declare_parameter<double>("kp_y", 1.2);
    ki_y_ = declare_parameter<double>("ki_y", 0.0);
    kd_y_ = declare_parameter<double>("kd_y", 0.0);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
    ki_yaw_ = declare_parameter<double>("ki_yaw", 0.0);
    kd_yaw_ = declare_parameter<double>("kd_yaw", 0.0);

    max_linear_x_ = declare_parameter<double>("max_linear_x", 0.25);
    max_linear_y_ = declare_parameter<double>("max_linear_y", 0.25);
    max_angular_z_ = declare_parameter<double>("max_angular_z", 1.2);
    max_integral_x_ = declare_parameter<double>("max_integral_x", 0.5);
    max_integral_y_ = declare_parameter<double>("max_integral_y", 0.5);
    max_integral_yaw_ = declare_parameter<double>("max_integral_yaw", 0.5);
    goal_tolerance_xy_ = declare_parameter<double>("goal_tolerance_xy", 0.03);
    goal_tolerance_yaw_ = declare_parameter<double>("goal_tolerance_yaw", 0.08);

    pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    pub_goal_reached_ = create_publisher<std_msgs::msg::Bool>(goal_reached_topic_, 10);

    if (publish_debug_topics_) {
      pub_error_ = create_publisher<geometry_msgs::msg::Twist>(error_topic_, 10);
      pub_debug_ = create_publisher<sdpo_motion_control::msg::GoToPointDebug>(debug_topic_, 10);
    }

    sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 10,
      std::bind(&GoToPointControllerNode::onGoal, this, std::placeholders::_1));
    sub_simple_goal_ = create_subscription<geometry_msgs::msg::Vector3>(
      simple_goal_topic_, 10,
      std::bind(&GoToPointControllerNode::onSimpleGoal, this, std::placeholders::_1));

    if (pose_source_ == "odom") {
      sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&GoToPointControllerNode::onOdom, this, std::placeholders::_1));
    } else if (pose_source_ == "pose") {
      sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, 10,
        std::bind(&GoToPointControllerNode::onPose, this, std::placeholders::_1));
    } else {
      throw std::runtime_error("pose_source must be either 'odom' or 'pose'");
    }

    if (use_initial_goal_) {
      goal_.header.frame_id = initial_goal_frame_id_;
      goal_.pose.position.x = initial_goal_x_;
      goal_.pose.position.y = initial_goal_y_;
      goal_.pose.position.z = 0.0;
      goal_.pose.orientation.w = std::cos(initial_goal_yaw_ * 0.5);
      goal_.pose.orientation.z = std::sin(initial_goal_yaw_ * 0.5);
      goal_active_ = true;
    }

    last_update_time_ = now();
    last_debug_publish_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(control_rate_hz, 1.0))),
      std::bind(&GoToPointControllerNode::onTimer, this));
  }

private:
  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_ = *msg;
    goal_active_ = true;
    resetControllerState();
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_frame_id_ = msg->header.frame_id;
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    pose_received_ = true;
  }

  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_frame_id_ = msg->header.frame_id;
    current_x_ = msg->pose.position.x;
    current_y_ = msg->pose.position.y;
    current_yaw_ = tf2::getYaw(msg->pose.orientation);
    pose_received_ = true;
  }

  void onSimpleGoal(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    goal_.header.frame_id = control_frame_id_;
    goal_.pose.position.x = msg->x;
    goal_.pose.position.y = msg->y;
    goal_.pose.position.z = 0.0;
    goal_.pose.orientation.x = 0.0;
    goal_.pose.orientation.y = 0.0;
    goal_.pose.orientation.z = std::sin(msg->z * 0.5);
    goal_.pose.orientation.w = std::cos(msg->z * 0.5);
    goal_active_ = true;
    resetControllerState();
  }

  void onTimer()
  {
    const auto current_time = now();
    double dt = (current_time - last_update_time_).seconds();
    last_update_time_ = current_time;
    if (dt <= 0.0) {
      dt = 1e-3;
    }

    if (!pose_received_ || !goal_active_) {
      if (publish_zero_when_inactive_) {
        publishZero();
      }
      publishGoalReached(false);
      return;
    }

    const std::string expected_frame =
      !control_frame_id_.empty() ? control_frame_id_ : current_frame_id_;

    if (!expected_frame.empty() && !current_frame_id_.empty() &&
      current_frame_id_ != expected_frame)
    {
      const std::string stop_reason =
        "current_pose_frame_mismatch: current_frame_id='" + current_frame_id_ +
        "', expected='" + expected_frame + "'";
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Stopping controller: current pose frame '%s' does not match control frame '%s'.",
        current_frame_id_.c_str(), expected_frame.c_str());
      if (publish_zero_when_inactive_) {
        publishZero();
      }
      publishDebug(
        current_time, false, stop_reason, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      publishGoalReached(false);
      return;
    }

    if (!goal_.header.frame_id.empty() && !expected_frame.empty() &&
      goal_.header.frame_id != expected_frame)
    {
      const std::string stop_reason =
        "goal_frame_mismatch: goal_frame_id='" + goal_.header.frame_id +
        "', expected='" + expected_frame + "'";
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Stopping controller: goal frame '%s' does not match control frame '%s'.",
        goal_.header.frame_id.c_str(), expected_frame.c_str());
      if (publish_zero_when_inactive_) {
        publishZero();
      }
      publishDebug(
        current_time, false, stop_reason, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      publishGoalReached(false);
      return;
    }

    const double dx_world = goal_.pose.position.x - current_x_;
    const double dy_world = goal_.pose.position.y - current_y_;
    const double goal_yaw = tf2::getYaw(goal_.pose.orientation);
    const double yaw_error = normalizeAngle(goal_yaw - current_yaw_);

    const double cos_yaw = std::cos(current_yaw_);
    const double sin_yaw = std::sin(current_yaw_);
    const double ex_body = cos_yaw * dx_world + sin_yaw * dy_world;
    const double ey_body = -sin_yaw * dx_world + cos_yaw * dy_world;

    if (std::hypot(dx_world, dy_world) <= goal_tolerance_xy_ &&
      std::abs(yaw_error) <= goal_tolerance_yaw_)
    {
      resetControllerState();
      if (stop_at_goal_) {
        publishZero();
      }
      publishDebug(
        current_time, true, "goal_reached", dx_world, dy_world, ex_body, ey_body, yaw_error,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      publishGoalReached(true);
      return;
    }

    integral_x_ = clamp(integral_x_ + ex_body * dt, -max_integral_x_, max_integral_x_);
    integral_y_ = clamp(integral_y_ + ey_body * dt, -max_integral_y_, max_integral_y_);
    integral_yaw_ = clamp(integral_yaw_ + yaw_error * dt, -max_integral_yaw_, max_integral_yaw_);

    const double dex = (ex_body - prev_error_x_) / dt;
    const double dey = (ey_body - prev_error_y_) / dt;
    const double dyaw = (yaw_error - prev_error_yaw_) / dt;

    prev_error_x_ = ex_body;
    prev_error_y_ = ey_body;
    prev_error_yaw_ = yaw_error;

    const double cmd_vx_raw = kp_x_ * ex_body + ki_x_ * integral_x_ + kd_x_ * dex;
    const double cmd_vy_raw = kp_y_ * ey_body + ki_y_ * integral_y_ + kd_y_ * dey;
    const double cmd_w_raw = kp_yaw_ * yaw_error + ki_yaw_ * integral_yaw_ + kd_yaw_ * dyaw;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = clamp(cmd_vx_raw, -max_linear_x_, max_linear_x_);
    cmd.linear.y = clamp(cmd_vy_raw, -max_linear_y_, max_linear_y_);
    cmd.angular.z = clamp(cmd_w_raw, -max_angular_z_, max_angular_z_);

    pub_cmd_vel_->publish(cmd);
    publishDebug(
      current_time, false, "", dx_world, dy_world, ex_body, ey_body, yaw_error, dex, dey,
      dyaw, cmd_vx_raw, cmd_vy_raw, cmd_w_raw, cmd.linear.x, cmd.linear.y, cmd.angular.z);
    publishGoalReached(false);
  }

  void publishDebug(
    const rclcpp::Time & stamp, bool goal_reached, const std::string & stop_reason,
    double dx_world, double dy_world, double ex_body, double ey_body, double yaw_error,
    double dex, double dey, double dyaw, double cmd_vx_raw, double cmd_vy_raw,
    double cmd_w_raw, double cmd_vx, double cmd_vy, double cmd_w)
  {
    if (!publish_debug_topics_) {
      return;
    }

    publishDebugError(ex_body, ey_body, yaw_error);

    if (!shouldPublishStructuredDebug(stamp)) {
      return;
    }

    const double goal_yaw = tf2::getYaw(goal_.pose.orientation);

    sdpo_motion_control::msg::GoToPointDebug msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = control_frame_id_;
    msg.active = goal_active_;
    msg.goal_reached = goal_reached;
    msg.pose_source = pose_source_;
    msg.control_frame_id = control_frame_id_;
    msg.current_frame_id = current_frame_id_;
    msg.goal_frame_id = goal_.header.frame_id;
    msg.stop_reason = stop_reason;
    msg.current_x = current_x_;
    msg.current_y = current_y_;
    msg.current_yaw = current_yaw_;
    msg.target_x = goal_.pose.position.x;
    msg.target_y = goal_.pose.position.y;
    msg.target_yaw = goal_yaw;
    msg.error_x_world = dx_world;
    msg.error_y_world = dy_world;
    msg.error_x_body = ex_body;
    msg.error_y_body = ey_body;
    msg.error_yaw = yaw_error;
    msg.distance_error = std::hypot(dx_world, dy_world);
    msg.integral_x = integral_x_;
    msg.integral_y = integral_y_;
    msg.integral_yaw = integral_yaw_;
    msg.derivative_x = dex;
    msg.derivative_y = dey;
    msg.derivative_yaw = dyaw;
    msg.cmd_vx_raw = cmd_vx_raw;
    msg.cmd_vy_raw = cmd_vy_raw;
    msg.cmd_w_raw = cmd_w_raw;
    msg.cmd_vx = cmd_vx;
    msg.cmd_vy = cmd_vy;
    msg.cmd_w = cmd_w;
    msg.linear_cmd_norm = std::hypot(cmd_vx, cmd_vy);
    msg.saturated_vx = isSaturated(cmd_vx_raw, cmd_vx);
    msg.saturated_vy = isSaturated(cmd_vy_raw, cmd_vy);
    msg.saturated_w = isSaturated(cmd_w_raw, cmd_w);
    pub_debug_->publish(msg);
    last_debug_publish_time_ = stamp;
  }

  void publishDebugError(double ex_body, double ey_body, double yaw_error)
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = ex_body;
    msg.linear.y = ey_body;
    msg.angular.z = yaw_error;
    pub_error_->publish(msg);
  }

  bool shouldPublishStructuredDebug(const rclcpp::Time & stamp) const
  {
    if (debug_publish_rate_hz_ <= 0.0) {
      return true;
    }
    if (last_debug_publish_time_.nanoseconds() == 0) {
      return true;
    }
    const double elapsed = (stamp - last_debug_publish_time_).seconds();
    return elapsed >= (1.0 / debug_publish_rate_hz_);
  }

  static bool isSaturated(double raw_value, double clamped_value)
  {
    return std::abs(raw_value - clamped_value) > 1e-9;
  }

  void publishGoalReached(bool reached)
  {
    std_msgs::msg::Bool msg;
    msg.data = reached;
    pub_goal_reached_->publish(msg);
  }

  void publishZero()
  {
    geometry_msgs::msg::Twist msg;
    pub_cmd_vel_->publish(msg);
  }

  void resetControllerState()
  {
    integral_x_ = 0.0;
    integral_y_ = 0.0;
    integral_yaw_ = 0.0;
    prev_error_x_ = 0.0;
    prev_error_y_ = 0.0;
    prev_error_yaw_ = 0.0;
  }

  static double normalizeAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static double clamp(double value, double min_value, double max_value)
  {
    return std::max(min_value, std::min(value, max_value));
  }

  std::string pose_source_;
  std::string control_frame_id_;
  std::string current_frame_id_;
  std::string odom_topic_;
  std::string pose_topic_;
  std::string goal_topic_;
  std::string simple_goal_topic_;
  std::string cmd_vel_topic_;
  std::string goal_reached_topic_;
  std::string error_topic_;
  std::string debug_topic_;
  std::string initial_goal_frame_id_;

  bool publish_debug_topics_ = true;
  bool publish_zero_when_inactive_ = true;
  bool stop_at_goal_ = true;
  bool use_initial_goal_ = false;
  bool pose_received_ = false;
  bool goal_active_ = false;

  double initial_goal_x_ = 0.0;
  double initial_goal_y_ = 0.0;
  double initial_goal_yaw_ = 0.0;

  double kp_x_ = 0.0;
  double ki_x_ = 0.0;
  double kd_x_ = 0.0;
  double kp_y_ = 0.0;
  double ki_y_ = 0.0;
  double kd_y_ = 0.0;
  double kp_yaw_ = 0.0;
  double ki_yaw_ = 0.0;
  double kd_yaw_ = 0.0;

  double max_linear_x_ = 0.0;
  double max_linear_y_ = 0.0;
  double max_angular_z_ = 0.0;
  double max_integral_x_ = 0.0;
  double max_integral_y_ = 0.0;
  double max_integral_yaw_ = 0.0;
  double goal_tolerance_xy_ = 0.0;
  double goal_tolerance_yaw_ = 0.0;
  double debug_publish_rate_hz_ = 0.0;

  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double current_yaw_ = 0.0;
  double integral_x_ = 0.0;
  double integral_y_ = 0.0;
  double integral_yaw_ = 0.0;
  double prev_error_x_ = 0.0;
  double prev_error_y_ = 0.0;
  double prev_error_yaw_ = 0.0;

  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_debug_publish_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::PoseStamped goal_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_error_;
  rclcpp::Publisher<sdpo_motion_control::msg::GoToPointDebug>::SharedPtr pub_debug_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_goal_reached_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_simple_goal_;
};

}  // namespace sdpo_motion_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sdpo_motion_control::GoToPointControllerNode>());
  rclcpp::shutdown();
  return 0;
}
