#pragma once

#include <ros/ros.h>
#include <sdpo_drivers_interfaces/MotEncArrayROS1.h>
#include <sdpo_drivers_interfaces/MotRefArrayROS1.h>
#include <std_srvs/SetBool.h>

#include "sdpo_ratf_driver/Robot5dpoRatf.h"





namespace sdpo_ratf_driver
{



const double kWatchdogMotWRef = 0.2;





class SdpoRatfDriverROS1
{

 private:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  ros::Publisher pub_mot_enc_;
  ros::Publisher pub_switch_1_;
  ros::Publisher pub_switch_2_;
  ros::Subscriber sub_mot_ref_;

  ros::ServiceServer srv_solenoid_1_;
  ros::ServiceServer srv_solenoid_2_;

  ros::Timer serial_port_timer_;

  ros::Time sample_time_;

  Robot5dpoRatf rob_;

  double encoder_res_;
  double gear_reduction_;
  std::string serial_port_name_;

  int mot_ctrl_freq_;
  int max_mot_pwm_;

  bool serial_comms_first_fault_;



 public:

  SdpoRatfDriverROS1();
  ~SdpoRatfDriverROS1() = default;

 private:

  void getParam();

  void checkSerialComms();

  void run();

  void pubMotEnc();
  void pubSwitch();
  void subMotRef(
      const sdpo_drivers_interfaces::MotRefArrayROS1::ConstPtr& msg);

  bool srvSolenoid1(std_srvs::SetBool::Request& request,
                    std_srvs::SetBool::Response& response);
  bool srvSolenoid2(std_srvs::SetBool::Request& request,
                    std_srvs::SetBool::Response& response);

};// class SdpoRatfDriverROS1



} // namespace sdpo_ratf_driver
