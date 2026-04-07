#include "sdpo_ratf_driver/SdpoRatfDriverROS2.h"





int main(int argc, char* argv[])
{

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<sdpo_ratf_driver::SdpoRatfDriverROS2>());

  rclcpp::shutdown();

  return 0;

} // int main(int argc, char* argv[])
