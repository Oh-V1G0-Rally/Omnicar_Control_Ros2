#include <memory>

#include "sdpo_ratf_ros_localization/SdpoRatfROSLocalizationROS.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sdpo_ratf_ros_localization::SdpoRatfROSLocalizationROS>());
  rclcpp::shutdown();
  return 0;
}
