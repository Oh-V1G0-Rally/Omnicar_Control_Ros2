#ifndef __ROS_API_H__
#define __ROS_API_H__

#include <string>

struct LaserScanSetting
{
  std::string frame_id;
  bool laser_scan_dir;
  bool enable_angle_crop_func;
  double angle_crop_min;
  double angle_crop_max;
  double range_min;
  double range_max;
};

struct LaserPose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
  std::string base_frame_id;
};

#endif
