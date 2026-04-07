#include <algorithm>
#include <mutex>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace sdpo_driver_omnijoy
{

class Omnijoy : public rclcpp::Node
{
public:
  Omnijoy()
  : Node("sdpo_driver_omnijoy")
  {
    axis_linear_x_ = this->declare_parameter<int>("axis_linear_x", 1);
    axis_linear_y_ = this->declare_parameter<int>("axis_linear_y", 2);
    axis_angular_ = this->declare_parameter<int>("axis_angular", 0);
    axis_deadman_ = this->declare_parameter<int>("axis_deadman", 4);
    axis_turbo_ = this->declare_parameter<int>("axis_turbo", 5);
    axis_turbo_up_ = this->declare_parameter<int>("axis_turbo_up", 6);
    axis_turbo_down_ = this->declare_parameter<int>("axis_turbo_down", 7);

    scale_linear_ = this->declare_parameter<double>("scale_linear", 0.1);
    scale_angular_ = this->declare_parameter<double>("scale_angular", 0.2);
    turbo_scale_linear_ =
        this->declare_parameter<double>("turbo_scale_linear", 0.2);
    turbo_max_scale_linear_ =
        this->declare_parameter<double>("turbo_max_scale_linear", 0.4);
    turbo_scale_angular_ =
        this->declare_parameter<double>("turbo_scale_angular", 0.4);

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10,
        std::bind(&Omnijoy::joyCallback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Omnijoy::publish, this));
  }

private:
  static double axisValue(const sensor_msgs::msg::Joy & joy, int index)
  {
    if (index < 0 || static_cast<size_t>(index) >= joy.axes.size()) {
      return 0.0;
    }
    return joy.axes[static_cast<size_t>(index)];
  }

  static bool buttonValue(const sensor_msgs::msg::Joy & joy, int index)
  {
    if (index < 0 || static_cast<size_t>(index) >= joy.buttons.size()) {
      return false;
    }
    return joy.buttons[static_cast<size_t>(index)] != 0;
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    last_published_.angular.z = axisValue(*joy, axis_angular_);
    last_published_.linear.x = axisValue(*joy, axis_linear_x_);
    last_published_.linear.y = axisValue(*joy, axis_linear_y_);

    deadman_pressed_ = buttonValue(*joy, axis_deadman_);
    turbo_pressed_ = buttonValue(*joy, axis_turbo_);
    turbo_up_pressed_ = buttonValue(*joy, axis_turbo_up_);
    turbo_down_pressed_ = buttonValue(*joy, axis_turbo_down_);
  }

  void publish()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (turbo_up_pressed_ && !turbo_down_pressed_) {
      turbo_scale_linear_ =
          std::min(turbo_scale_linear_ + 0.05, turbo_max_scale_linear_);
    } else if (!turbo_up_pressed_ && turbo_down_pressed_) {
      turbo_scale_linear_ = std::max(turbo_scale_linear_ - 0.05, scale_linear_);
    }

    if (deadman_pressed_) {
      geometry_msgs::msg::Twist cmd = last_published_;
      if (turbo_pressed_) {
        cmd.linear.x *= turbo_scale_linear_;
        cmd.linear.y *= turbo_scale_linear_;
        cmd.angular.z *= turbo_scale_angular_;
      } else {
        cmd.linear.x *= scale_linear_;
        cmd.linear.y *= scale_linear_;
        cmd.angular.z *= scale_angular_;
      }
      vel_pub_->publish(cmd);
      zero_twist_published_ = false;
      return;
    }

    if (!zero_twist_published_) {
      vel_pub_->publish(geometry_msgs::msg::Twist());
      zero_twist_published_ = true;
    }
  }

  int axis_linear_x_;
  int axis_linear_y_;
  int axis_angular_;
  int axis_deadman_;
  int axis_turbo_;
  int axis_turbo_up_;
  int axis_turbo_down_;

  double scale_linear_;
  double scale_angular_;
  double turbo_scale_linear_;
  double turbo_max_scale_linear_;
  double turbo_scale_angular_;

  geometry_msgs::msg::Twist last_published_;
  std::mutex mutex_;
  bool deadman_pressed_ = false;
  bool turbo_pressed_ = false;
  bool turbo_up_pressed_ = false;
  bool turbo_down_pressed_ = false;
  bool zero_twist_published_ = false;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sdpo_driver_omnijoy

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sdpo_driver_omnijoy::Omnijoy>());
  rclcpp::shutdown();
  return 0;
}
