#pragma once

#include <string>
#include <vector>

#include "sdpo_ros_odom/OdomWhMotorDrive.h"
#include "sdpo_ros_odom/utils.h"

namespace sdpo_ros_odom {

class OdomWh
{
public:
  OdomPose2D pose;
  OdomVel2D vel;
  OdomDelta2D odo;

  std::vector<OdomWhMotorDrive> mot;
  std::vector<size_t> mot_idx;
  bool w_r_max_enabled = false;
  double w_r_max = 0.0;

  std::vector<double> rob_l;

  OdomWh() = default;
  virtual ~OdomWh() = default;

  void setMotorWRefMax(const bool & enable, const double & w_ref_max)
  {
    w_r_max_enabled = enable;
    if (w_r_max_enabled) {
      w_r_max = w_ref_max;
    }
  }

  void setMotorDriveEncTicksDelta(
    const size_t & idx, const int32_t & delta_ticks, const double & ticks_rev)
  {
    mot[mot_idx[idx]].setEncTicksDelta(delta_ticks, ticks_rev);
  }

  void setMotorDriveW(const size_t & idx, const double & w_curr)
  {
    mot[mot_idx[idx]].setW(w_curr);
  }

  double getMotorDriveWr(const size_t & idx)
  {
    return mot[mot_idx[idx]].w_r;
  }

  void setVelRef(const double & v_ref, const double & vn_ref, const double & w_ref)
  {
    vel.v_r = v_ref;
    vel.vn_r = vn_ref;
    vel.w_r = w_ref;

    updateVelRefInv();

    if (w_r_max_enabled) {
      scaleMotorsDriveWr();
    }
  }

  void update()
  {
    updateVel();
    updatePose();
  }

  virtual std::string getMotorDriveIdxStr(const size_t & idx) = 0;
  virtual void updateVelRef() = 0;
  virtual void updateVel() = 0;
  virtual void updateVelRefInv() = 0;
  virtual void updateOdomDelta() = 0;

protected:
  void updatePose()
  {
    updateOdomDelta();

    pose.x += odo.x_delta;
    pose.y += odo.y_delta;
    pose.th += odo.th_delta;
  }

  void scaleMotorsDriveWr()
  {
    double curr_w_r_max = 0.0;

    for (const auto & m : mot) {
      if (std::abs(m.w_r) > curr_w_r_max) {
        curr_w_r_max = std::abs(m.w_r);
      }
    }

    if ((curr_w_r_max > w_r_max) && (w_r_max != 0.0)) {
      const double scale = w_r_max / curr_w_r_max;

      for (auto & m : mot) {
        m.setWr(m.w_r * scale);
      }

      updateVelRef();
    }
  }
};

}  // namespace sdpo_ros_odom
