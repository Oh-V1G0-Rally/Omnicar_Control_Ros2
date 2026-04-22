#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sdpo_motion_control/msg/motion_state.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

namespace sdpo_motion_control
{

class MotionStateAdapterNode : public rclcpp::Node
{
public:
  MotionStateAdapterNode()
  : Node("motion_state_adapter")
  {
    input_type_ = declare_parameter<std::string>("input_type", "odom");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "odom");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "pose");
    state_topic_ = declare_parameter<std::string>("state_topic", "motion_state");
    body_frame_id_ = declare_parameter<std::string>("body_frame_id", "base_footprint");
    default_state_frame_id_ = declare_parameter<std::string>("default_state_frame_id", "");
    publish_zero_velocity_for_pose_ =
      declare_parameter<bool>("publish_zero_velocity_for_pose", true);

    pub_state_ = create_publisher<sdpo_motion_control::msg::MotionState>(state_topic_, 10);

    if (input_type_ == "odom") {
      sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&MotionStateAdapterNode::onOdom, this, std::placeholders::_1));
    } else if (input_type_ == "pose") {
      sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, 10,
        std::bind(&MotionStateAdapterNode::onPose, this, std::placeholders::_1));
    } else {
      throw std::runtime_error("input_type must be either 'odom' or 'pose'");
    }
  }

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    sdpo_motion_control::msg::MotionState state;
    state.header = msg->header;
    if (state.header.frame_id.empty() && !default_state_frame_id_.empty()) {
      state.header.frame_id = default_state_frame_id_;
    }
    state.child_frame_id = msg->child_frame_id.empty() ? body_frame_id_ : msg->child_frame_id;
    state.source_type = "odom";
    state.x = msg->pose.pose.position.x;
    state.y = msg->pose.pose.position.y;
    state.yaw = tf2::getYaw(msg->pose.pose.orientation);
    state.vx = msg->twist.twist.linear.x;
    state.vy = msg->twist.twist.linear.y;
    state.w = msg->twist.twist.angular.z;
    state.has_velocity = true;
    pub_state_->publish(state);
  }

  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    sdpo_motion_control::msg::MotionState state;
    state.header = msg->header;
    if (state.header.frame_id.empty() && !default_state_frame_id_.empty()) {
      state.header.frame_id = default_state_frame_id_;
    }
    state.child_frame_id = body_frame_id_;
    state.source_type = "pose";
    state.x = msg->pose.position.x;
    state.y = msg->pose.position.y;
    state.yaw = tf2::getYaw(msg->pose.orientation);
    if (publish_zero_velocity_for_pose_) {
      state.vx = 0.0;
      state.vy = 0.0;
      state.w = 0.0;
    }
    state.has_velocity = false;
    pub_state_->publish(state);
  }

  std::string input_type_;
  std::string odom_topic_;
  std::string pose_topic_;
  std::string state_topic_;
  std::string body_frame_id_;
  std::string default_state_frame_id_;
  bool publish_zero_velocity_for_pose_ = true;

  rclcpp::Publisher<sdpo_motion_control::msg::MotionState>::SharedPtr pub_state_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
};

}  // namespace sdpo_motion_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sdpo_motion_control::MotionStateAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
