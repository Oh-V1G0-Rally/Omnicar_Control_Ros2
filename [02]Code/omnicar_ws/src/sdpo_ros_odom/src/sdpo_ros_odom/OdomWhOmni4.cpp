#include "sdpo_ros_odom/OdomWhOmni4.h"

#include <cmath>
#include <stdexcept>

namespace sdpo_ros_odom {

OdomWhOmni4::OdomWhOmni4(
  const std::vector<size_t> & wh_idx, const std::vector<double> & wh_d,
  const std::vector<bool> & wh_inv, const std::vector<double> & rob_len)
{
  if ((wh_idx.size() != 4) || (wh_d.size() != 4) || (wh_inv.size() != 4) || (rob_len.size() != 2)) {
    throw std::invalid_argument("OdomWhOmni4 expects 4 wheels and 2 robot dimensions");
  }

  mot_idx = idx2valueVector(wh_idx);
  mot.resize(wh_d.size());
  rob_l.resize(rob_len.size());

  mot[kWhIdxFL].wh_d = wh_d[kWhIdxFL];
  mot[kWhIdxFR].wh_d = wh_d[kWhIdxFR];
  mot[kWhIdxBL].wh_d = wh_d[kWhIdxBL];
  mot[kWhIdxBR].wh_d = wh_d[kWhIdxBR];

  mot[kWhIdxFL].inverted = wh_inv[kWhIdxFL];
  mot[kWhIdxFR].inverted = wh_inv[kWhIdxFR];
  mot[kWhIdxBL].inverted = wh_inv[kWhIdxBL];
  mot[kWhIdxBR].inverted = wh_inv[kWhIdxBR];

  rob_l[kRobLenIdxF2B] = rob_len[kRobLenIdxF2B];
  rob_l[kRobLenIdxL2R] = rob_len[kRobLenIdxL2R];
}

std::string OdomWhOmni4::getMotorDriveIdxStr(const size_t & idx)
{
  switch (mot_idx[idx]) {
    case kWhIdxFL: return "FL";
    case kWhIdxFR: return "FR";
    case kWhIdxBL: return "BL";
    case kWhIdxBR: return "BR";
    default: return "nan";
  }
}

void OdomWhOmni4::updateVel()
{
  vel.v = (mot[kWhIdxFL].v - mot[kWhIdxFR].v + mot[kWhIdxBL].v - mot[kWhIdxBR].v) / 4.0;
  vel.vn = (-mot[kWhIdxFL].v - mot[kWhIdxFR].v + mot[kWhIdxBL].v + mot[kWhIdxBR].v) / 4.0;
  vel.w = -(mot[kWhIdxFL].v + mot[kWhIdxFR].v + mot[kWhIdxBL].v + mot[kWhIdxBR].v) /
    (2.0 * (rob_l[kRobLenIdxF2B] + rob_l[kRobLenIdxL2R]));
}

void OdomWhOmni4::updateVelRef()
{
  vel.v_r = (mot[kWhIdxFL].v_r - mot[kWhIdxFR].v_r + mot[kWhIdxBL].v_r - mot[kWhIdxBR].v_r) / 4.0;
  vel.vn_r = (-mot[kWhIdxFL].v_r - mot[kWhIdxFR].v_r + mot[kWhIdxBL].v_r + mot[kWhIdxBR].v_r) / 4.0;
  vel.w_r = -(mot[kWhIdxFL].v_r + mot[kWhIdxFR].v_r + mot[kWhIdxBL].v_r + mot[kWhIdxBR].v_r) /
    (2.0 * (rob_l[kRobLenIdxF2B] + rob_l[kRobLenIdxL2R]));
}

void OdomWhOmni4::updateVelRefInv()
{
  const double robot_span = rob_l[kRobLenIdxF2B] + rob_l[kRobLenIdxL2R];
  mot[kWhIdxFL].setVr(vel.v_r - vel.vn_r - robot_span * vel.w_r * 0.5);
  mot[kWhIdxFR].setVr(-vel.v_r - vel.vn_r - robot_span * vel.w_r * 0.5);
  mot[kWhIdxBL].setVr(vel.v_r + vel.vn_r - robot_span * vel.w_r * 0.5);
  mot[kWhIdxBR].setVr(-vel.v_r + vel.vn_r - robot_span * vel.w_r * 0.5);
}

void OdomWhOmni4::updateOdomDelta()
{
  const double robot_span = rob_l[kRobLenIdxF2B] + rob_l[kRobLenIdxL2R];

  odo.u_delta = (
    mot[kWhIdxFL].dist_delta - mot[kWhIdxFR].dist_delta +
    mot[kWhIdxBL].dist_delta - mot[kWhIdxBR].dist_delta) / 4.0;
  odo.v_delta = (
    -mot[kWhIdxFL].dist_delta - mot[kWhIdxFR].dist_delta +
    mot[kWhIdxBL].dist_delta + mot[kWhIdxBR].dist_delta) / 4.0;
  odo.w_delta = -(
    mot[kWhIdxFL].dist_delta + mot[kWhIdxFR].dist_delta +
    mot[kWhIdxBL].dist_delta + mot[kWhIdxBR].dist_delta) / (2.0 * robot_span);

  if (odo.w_delta == 0.0) {
    odo.x_delta = odo.u_delta * cos(pose.th) - odo.v_delta * sin(pose.th);
    odo.y_delta = odo.u_delta * sin(pose.th) + odo.v_delta * cos(pose.th);
  } else {
    odo.x_delta =
      (odo.u_delta * sin(odo.w_delta) + odo.v_delta * (cos(odo.w_delta) - 1.0)) *
      cos(pose.th + odo.w_delta / 2.0) / odo.w_delta -
      (odo.u_delta * (1.0 - cos(odo.w_delta)) + odo.v_delta * sin(odo.w_delta)) *
      sin(pose.th + odo.w_delta / 2.0) / odo.w_delta;
    odo.y_delta =
      (odo.u_delta * sin(odo.w_delta) + odo.v_delta * (cos(odo.w_delta) - 1.0)) *
      sin(pose.th + odo.w_delta / 2.0) / odo.w_delta +
      (odo.u_delta * (1.0 - cos(odo.w_delta)) + odo.v_delta * sin(odo.w_delta)) *
      cos(pose.th + odo.w_delta / 2.0) / odo.w_delta;
  }

  odo.th_delta = odo.w_delta;
}

}  // namespace sdpo_ros_odom
