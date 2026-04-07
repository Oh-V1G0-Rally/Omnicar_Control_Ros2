#pragma once

#include <ros/ros.h>
#include <sdpo_drivers_interfaces/MotDataArrayROS1.h>
#include <sdpo_drivers_interfaces/MotRefArrayROS1.h>
#include <sdpo_drivers_interfaces/SetMotorsPWM.h>
#include <std_srvs/SetBool.h>

#include "sdpo_ratf_driver/Robot5dpoRatfTune.h"





namespace sdpo_ratf_driver
{



class SdpoRatfTuneDriverROS1
{

 private:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  ros::Publisher pub_mot_data_;
  ros::Publisher pub_switch_1_;
  ros::Publisher pub_switch_2_;
  ros::Subscriber sub_mot_ref_;

  ros::ServiceServer srv_motors_pwm_;
  ros::ServiceServer srv_solenoid_1_;
  ros::ServiceServer srv_solenoid_2_;

  ros::Timer serial_port_timer_;

  ros::Time sample_time_;

  Robot5dpoRatfTune rob_;

  double encoder_res_;
  double gear_reduction_;
  std::string serial_port_name_;

  int mot_ctrl_freq_;
  int max_mot_pwm_;

  bool serial_comms_first_fault_;



 public:

  SdpoRatfTuneDriverROS1();
  ~SdpoRatfTuneDriverROS1() = default;

 private:

  void getParam();

  void checkSerialComms();

  void run();

  void pubMotData();
  void pubSwitch();
  void subMotRef(
      const sdpo_drivers_interfaces::MotRefArrayROS1::ConstPtr& msg);

  bool srvMotorsPWM(sdpo_drivers_interfaces::SetMotorsPWM::Request& request,
                    sdpo_drivers_interfaces::SetMotorsPWM::Response& response);

  bool srvSolenoid1(std_srvs::SetBool::Request& request,
                    std_srvs::SetBool::Response& response);
  bool srvSolenoid2(std_srvs::SetBool::Request& request,
                    std_srvs::SetBool::Response& response);

};// class SdpoRatfTuneDriverROS1



} // namespace sdpo_ratf_driver
