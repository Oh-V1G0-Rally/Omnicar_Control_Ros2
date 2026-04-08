#pragma once

namespace sdpo_ratf_ros_localization {

struct SdpoRatfBeacons
{
  int num_pts = 0;
  double x = 0.0;
  double y = 0.0;
  double dist = 0.0;
  double ang = 0.0;
};

}  // namespace sdpo_ratf_ros_localization
