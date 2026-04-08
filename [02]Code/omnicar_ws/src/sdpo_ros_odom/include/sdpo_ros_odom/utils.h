#pragma once

#include <cmath>
#include <vector>

namespace sdpo_ros_odom {

inline double normAngRad(double angle)
{
  angle = fmod(angle + M_PI, M_PI * 2.0);
  if (angle < 0) {
    angle += M_PI * 2.0;
  }
  return angle - M_PI;
}

struct OdomPose2D
{
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
};

struct OdomVel2D
{
  double v_r = 0.0;
  double vn_r = 0.0;
  double w_r = 0.0;

  double v = 0.0;
  double vn = 0.0;
  double w = 0.0;
};

struct OdomDelta2D
{
  double x_delta = 0.0;
  double y_delta = 0.0;
  double th_delta = 0.0;

  double u_delta = 0.0;
  double v_delta = 0.0;
  double w_delta = 0.0;
};

std::vector<size_t> idx2valueVector(const std::vector<size_t> & vec_ini);

}  // namespace sdpo_ros_odom
