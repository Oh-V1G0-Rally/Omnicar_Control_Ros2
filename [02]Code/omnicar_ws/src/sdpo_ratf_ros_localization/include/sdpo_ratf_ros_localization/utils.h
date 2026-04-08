#pragma once

#include <cmath>

namespace sdpo_ratf_ros_localization {

inline double dist(double x, double y)
{
  return std::sqrt(x * x + y * y);
}

inline double normAngRad(double angle)
{
  angle = std::fmod(angle + M_PI, M_PI * 2.0);
  if (angle < 0.0) {
    angle += M_PI * 2.0;
  }
  return angle - M_PI;
}

inline double deg2rad(double angle)
{
  return angle * M_PI / 180.0;
}

inline double rad2deg(double angle)
{
  return angle * 180.0 / M_PI;
}

}  // namespace sdpo_ratf_ros_localization
