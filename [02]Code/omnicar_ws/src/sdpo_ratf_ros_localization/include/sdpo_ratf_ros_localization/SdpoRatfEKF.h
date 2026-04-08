#pragma once

#include <array>
#include <string>

#include <eigen3/Eigen/Eigen>

#include "sdpo_ratf_ros_localization/SdpoRatfBeacons.h"

namespace sdpo_ratf_ros_localization {

const std::string kSdpoRatfEKFModeOdomWhOnlyStr = "OdomWhOnly";
const std::string kSdpoRatfEKFModeSensOnlyStr = "SensOnly";
const std::string kSdpoRatfEKFModeFusionStr = "Fusion";

class SdpoRatfEKF
{
public:
  enum class SdpoRatfEKFMode
  {
    kSdpoRatfEKFOdomWhOnly,
    kSdpoRatfEKFSensOnly,
    kSdpoRatfEKFFusion
  };

  SdpoRatfEKF();

  void initCovP(const double & p00_ini, const double & p11_ini, const double & p22_ini);
  void initCovQ(const double & q00, const double & q11, const double & q22);
  void initCovR(const double & r00, const double & r11);
  void initPose(const double & x_ini, const double & y_ini, const double & th_ini);

  Eigen::Vector3d getRobotPose() const;
  Eigen::Matrix3d getCovP() const;

  void setUseLastParam(bool use_last);
  void setMode(const SdpoRatfEKFMode & mode_enum);
  void setMode(const std::string & mode_str);
  SdpoRatfEKFMode getMode() const;
  std::string getModeStr() const;

  void predict(const double & dd, const double & ddn, const double & dth);
  void update(const SdpoRatfBeacons & beacon_obs, const std::array<double, 2> & beacon_gt);

  Eigen::Vector3d XR;

private:
  void updateOdom(const double & dd, const double & ddn, const double & dth);

  SdpoRatfEKFMode mode_;
  Eigen::Vector3d xr_last_;
  Eigen::Matrix3d cov_q_;
  Eigen::Matrix3d cov_p_;
  Eigen::Matrix3d grad_f_x_;
  Eigen::Matrix3d grad_f_q_;
  Eigen::Matrix2d cov_r_;
  Eigen::Matrix<double, 2, 3> grad_h_x_;
  bool use_last_;
};

}  // namespace sdpo_ratf_ros_localization
