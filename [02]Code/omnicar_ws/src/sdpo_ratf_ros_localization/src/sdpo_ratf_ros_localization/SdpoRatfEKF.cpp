#include "sdpo_ratf_ros_localization/SdpoRatfEKF.h"

#include <cmath>

#include "sdpo_ratf_ros_localization/utils.h"

namespace sdpo_ratf_ros_localization {

SdpoRatfEKF::SdpoRatfEKF()
: mode_(SdpoRatfEKFMode::kSdpoRatfEKFFusion), use_last_(false)
{
  XR = Eigen::Vector3d::Zero();
  xr_last_ = Eigen::Vector3d::Zero();
  cov_q_ = Eigen::Matrix3d::Zero();
  cov_r_ = Eigen::Matrix2d::Zero();
  cov_p_ = Eigen::Matrix3d::Zero();
  grad_f_x_ = Eigen::Matrix3d::Identity();
  grad_f_q_ = Eigen::Matrix3d::Zero();
  grad_f_q_(2, 2) = 1.0;
  grad_h_x_ = Eigen::Matrix<double, 2, 3>::Zero();
  grad_h_x_(1, 2) = -1.0;
}

void SdpoRatfEKF::initCovP(const double & p00_ini, const double & p11_ini, const double & p22_ini)
{
  cov_p_ = Eigen::Matrix3d::Zero();
  cov_p_(0, 0) = p00_ini;
  cov_p_(1, 1) = p11_ini;
  cov_p_(2, 2) = p22_ini;
}

void SdpoRatfEKF::initCovQ(const double & q00, const double & q11, const double & q22)
{
  cov_q_(0, 0) = q00;
  cov_q_(1, 1) = q11;
  cov_q_(2, 2) = q22;
}

void SdpoRatfEKF::initCovR(const double & r00, const double & r11)
{
  cov_r_(0, 0) = r00;
  cov_r_(1, 1) = r11;
}

void SdpoRatfEKF::initPose(const double & x_ini, const double & y_ini, const double & th_ini)
{
  XR(0) = x_ini;
  XR(1) = y_ini;
  XR(2) = th_ini;
  xr_last_ = XR;
}

Eigen::Vector3d SdpoRatfEKF::getRobotPose() const
{
  return XR;
}

Eigen::Matrix3d SdpoRatfEKF::getCovP() const
{
  return cov_p_;
}

void SdpoRatfEKF::setUseLastParam(bool use_last)
{
  use_last_ = use_last;
}

void SdpoRatfEKF::setMode(const SdpoRatfEKFMode & mode_enum)
{
  mode_ = mode_enum;
}

void SdpoRatfEKF::setMode(const std::string & mode_str)
{
  if (mode_str == kSdpoRatfEKFModeOdomWhOnlyStr) {
    mode_ = SdpoRatfEKFMode::kSdpoRatfEKFOdomWhOnly;
  } else if (mode_str == kSdpoRatfEKFModeSensOnlyStr) {
    mode_ = SdpoRatfEKFMode::kSdpoRatfEKFSensOnly;
  } else {
    mode_ = SdpoRatfEKFMode::kSdpoRatfEKFFusion;
  }
}

SdpoRatfEKF::SdpoRatfEKFMode SdpoRatfEKF::getMode() const
{
  return mode_;
}

std::string SdpoRatfEKF::getModeStr() const
{
  if (mode_ == SdpoRatfEKFMode::kSdpoRatfEKFOdomWhOnly) {
    return kSdpoRatfEKFModeOdomWhOnlyStr;
  }
  if (mode_ == SdpoRatfEKFMode::kSdpoRatfEKFSensOnly) {
    return kSdpoRatfEKFModeSensOnlyStr;
  }
  return kSdpoRatfEKFModeFusionStr;
}

void SdpoRatfEKF::predict(const double & dd, const double & ddn, const double & dth)
{
  if (mode_ != SdpoRatfEKFMode::kSdpoRatfEKFFusion &&
    mode_ != SdpoRatfEKFMode::kSdpoRatfEKFOdomWhOnly)
  {
    return;
  }

  updateOdom(dd, ddn, dth);

  grad_f_x_(0, 2) = -dd * std::sin(XR(2)) - ddn * std::cos(XR(2));
  grad_f_x_(1, 2) = dd * std::cos(XR(2)) - ddn * std::sin(XR(2));

  grad_f_q_(0, 0) = std::cos(XR(2));
  grad_f_q_(0, 1) = -std::sin(XR(2));
  grad_f_q_(1, 0) = std::sin(XR(2));
  grad_f_q_(1, 1) = std::cos(XR(2));

  cov_p_ = grad_f_x_ * cov_p_ * grad_f_x_.transpose() +
    grad_f_q_ * cov_q_ * grad_f_q_.transpose();
}

void SdpoRatfEKF::update(const SdpoRatfBeacons & beacon_obs, const std::array<double, 2> & beacon_gt)
{
  if (mode_ != SdpoRatfEKFMode::kSdpoRatfEKFFusion &&
    mode_ != SdpoRatfEKFMode::kSdpoRatfEKFSensOnly)
  {
    return;
  }

  Eigen::Vector2d diff = Eigen::Vector2d::Zero();
  diff(0) = beacon_gt[0] - XR(0);
  diff(1) = beacon_gt[1] - XR(1);
  const double d_beacon = std::sqrt(std::pow(diff(0), 2) + std::pow(diff(1), 2));

  const Eigen::Vector3d & state = use_last_ ? xr_last_ : XR;
  grad_h_x_(0, 0) = -(beacon_gt[0] - state(0)) / d_beacon;
  grad_h_x_(1, 0) = (beacon_gt[1] - state(1)) / std::pow(d_beacon, 2);
  grad_h_x_(0, 1) = -(beacon_gt[1] - state(1)) / d_beacon;
  grad_h_x_(1, 1) = -(beacon_gt[0] - state(0)) / std::pow(d_beacon, 2);

  const Eigen::Matrix<double, 3, 2> kf = cov_p_ * grad_h_x_.transpose() *
    (grad_h_x_ * cov_p_ * grad_h_x_.transpose() + cov_r_).inverse();

  cov_p_ = (Eigen::Matrix3d::Identity() - kf * grad_h_x_) * cov_p_;

  Eigen::Vector2d innovation = Eigen::Vector2d::Zero();
  innovation(0) = beacon_obs.dist - d_beacon;
  innovation(1) = normAngRad(
    beacon_obs.ang - normAngRad(std::atan2(beacon_gt[1] - XR(1), beacon_gt[0] - XR(0)) - XR(2)));

  XR = XR + kf * innovation;
}

void SdpoRatfEKF::updateOdom(const double & dd, const double & ddn, const double & dth)
{
  XR(0) += dd * std::cos(XR(2)) - ddn * std::sin(XR(2));
  XR(1) += dd * std::sin(XR(2)) + ddn * std::cos(XR(2));
  XR(2) = normAngRad(XR(2) + dth);
  xr_last_ = XR;
}

}  // namespace sdpo_ratf_ros_localization
