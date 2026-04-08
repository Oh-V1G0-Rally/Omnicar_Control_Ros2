#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "sdpo_ros_odom/OdomWhROS2.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sdpo_ros_odom::OdomWhROS2>());
  rclcpp::shutdown();
  return 0;
}
