#include "sdpo_ratf_driver/SdpoRatfTuneDriverROS1.h"

int main(int argc, char* argv[])
{

  ros::init(argc, argv, "sdpo_ratf_driver");

  sdpo_ratf_driver::SdpoRatfTuneDriverROS1 node;

  ros::spin();

  ros::shutdown();

  return 0;

} // int main(int argc, char* argv[])
