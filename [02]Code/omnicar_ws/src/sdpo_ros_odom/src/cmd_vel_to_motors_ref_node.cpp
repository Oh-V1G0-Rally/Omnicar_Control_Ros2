#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sdpo_drivers_interfaces/msg/mot_ref.hpp>
#include <sdpo_drivers_interfaces/msg/mot_ref_array.hpp>

namespace sdpo_ros_odom
{

class CmdVelToMotorsRefNode : public rclcpp::Node
{
public:
  CmdVelToMotorsRefNode()
  : Node("sdpo_ros_odom")
  {
    steering_geometry_ =
        this->declare_parameter<std::string>("steering_geometry", "omni4");
    w_ref_max_enabled_ =
        this->declare_parameter<bool>("w_ref_max_enabled", true);
    w_ref_max_ = this->declare_parameter<double>("w_ref_max", 55.0);
    dist_front_back_ =
        this->declare_parameter<double>("rob_dist_between_front_back_wh", 0.185);
    dist_left_right_ =
        this->declare_parameter<double>("rob_dist_between_left_right_wh", 0.220);

    wheel_diam_[0] = this->declare_parameter<double>("wh_front_left_diam", 0.06);
    wheel_diam_[1] = this->declare_parameter<double>("wh_front_right_diam", 0.06);
    wheel_diam_[2] = this->declare_parameter<double>("wh_back_left_diam", 0.06);
    wheel_diam_[3] = this->declare_parameter<double>("wh_back_right_diam", 0.06);

    wheel_inverted_[0] =
        this->declare_parameter<bool>("wh_front_left_inv", false);
    wheel_inverted_[1] =
        this->declare_parameter<bool>("wh_front_right_inv", true);
    wheel_inverted_[2] =
        this->declare_parameter<bool>("wh_back_left_inv", false);
    wheel_inverted_[3] =
        this->declare_parameter<bool>("wh_back_right_inv", true);

    if (steering_geometry_ != "omni4") {
      RCLCPP_WARN(
          this->get_logger(),
          "Only omni4 inverse kinematics is currently ported. Received steering_geometry='%s'",
          steering_geometry_.c_str());
    }

    pub_mot_ref_ = this->create_publisher<sdpo_drivers_interfaces::msg::MotRefArray>(
        "motors_ref", 1);
    pub_cmd_vel_ref_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_ref", 1);
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&CmdVelToMotorsRefNode::subCmdVel, this, std::placeholders::_1));
  }

private:
  void subCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double v = msg->linear.x;
    const double vn = msg->linear.y;
    const double w = msg->angular.z;
    const double robot_span = dist_front_back_ + dist_left_right_;

    std::array<double, 4> wheel_linear = {
        v - vn - robot_span * w * 0.5,
        -v - vn - robot_span * w * 0.5,
        v + vn - robot_span * w * 0.5,
        -v + vn - robot_span * w * 0.5,
    };

    std::array<double, 4> wheel_ang{};
    for (size_t i = 0; i < wheel_linear.size(); ++i) {
      wheel_ang[i] = lin2ang(wheel_linear[i], wheel_diam_[i]);
      if (wheel_inverted_[i]) {
        wheel_ang[i] = -wheel_ang[i];
      }
    }

    if (w_ref_max_enabled_ && w_ref_max_ > 0.0) {
      double curr_w_max = 0.0;
      for (double value : wheel_ang) {
        curr_w_max = std::max(curr_w_max, std::abs(value));
      }
      if (curr_w_max > w_ref_max_) {
        const double scale = w_ref_max_ / curr_w_max;
        for (double & value : wheel_ang) {
          value *= scale;
        }
      }
    }

    // Reconstruct scaled robot reference, mirroring the legacy cmd_vel_ref behavior.
    std::array<double, 4> scaled_linear{};
    for (size_t i = 0; i < wheel_ang.size(); ++i) {
      double linear = ang2lin(wheel_ang[i], wheel_diam_[i]);
      scaled_linear[i] = wheel_inverted_[i] ? -linear : linear;
    }

    geometry_msgs::msg::Twist cmd_vel_ref;
    cmd_vel_ref.linear.x =
        (scaled_linear[0] - scaled_linear[1] + scaled_linear[2] - scaled_linear[3]) /
        4.0;
    cmd_vel_ref.linear.y =
        (-scaled_linear[0] - scaled_linear[1] + scaled_linear[2] + scaled_linear[3]) /
        4.0;
    cmd_vel_ref.angular.z =
        -(scaled_linear[0] + scaled_linear[1] + scaled_linear[2] + scaled_linear[3]) /
        (2.0 * robot_span);
    pub_cmd_vel_ref_->publish(cmd_vel_ref);

    sdpo_drivers_interfaces::msg::MotRefArray mot_ref_msg;
    mot_ref_msg.stamp = this->now();
    mot_ref_msg.ang_speed_ref.resize(4);
    for (size_t i = 0; i < wheel_ang.size(); ++i) {
      sdpo_drivers_interfaces::msg::MotRef ref_msg;
      ref_msg.ref = wheel_ang[i];
      mot_ref_msg.ang_speed_ref[i] = ref_msg;
    }
    pub_mot_ref_->publish(mot_ref_msg);
  }

  static double lin2ang(double linear, double wheel_diameter)
  {
    return 2.0 * linear / wheel_diameter;
  }

  static double ang2lin(double angular, double wheel_diameter)
  {
    return angular * wheel_diameter / 2.0;
  }

  std::string steering_geometry_;
  bool w_ref_max_enabled_;
  double w_ref_max_;
  double dist_front_back_;
  double dist_left_right_;
  std::array<double, 4> wheel_diam_{};
  std::array<bool, 4> wheel_inverted_{};

  rclcpp::Publisher<sdpo_drivers_interfaces::msg::MotRefArray>::SharedPtr pub_mot_ref_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_ref_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
};

}  // namespace sdpo_ros_odom

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sdpo_ros_odom::CmdVelToMotorsRefNode>());
  rclcpp::shutdown();
  return 0;
}
