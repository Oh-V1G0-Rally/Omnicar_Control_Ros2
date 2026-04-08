#pragma once

#include <cstdint>

namespace sdpo_ros_odom {

struct OdomWhMotorDrive
{
  bool inverted = false;
  int32_t enc_ticks_delta = 0;
  double ticks_per_rev = 1.0;

  double ang_delta = 0.0;
  double dist_delta = 0.0;

  double w_r = 0.0;
  double w = 0.0;

  double v_r = 0.0;
  double v = 0.0;

  double wh_d = 1.0;

  inline double lin2ang(const double & linear) const
  {
    return 2.0 * linear / wh_d;
  }

  inline double ang2lin(const double & angular) const
  {
    return angular * wh_d / 2.0;
  }

  void setEncTicksDelta(const int32_t & delta_ticks);
  void setEncTicksDelta(const int32_t & delta_ticks, const double & ticks_rev);
  void setDistDelta(const double & delta_dist);
  void setDistDelta(const double & delta_dist, const double & ticks_rev);
  void setW(const double & w_curr);
  void setWr(const double & w_ref);
  void setV(const double & v_curr);
  void setVr(const double & v_ref);
};

}  // namespace sdpo_ros_odom
