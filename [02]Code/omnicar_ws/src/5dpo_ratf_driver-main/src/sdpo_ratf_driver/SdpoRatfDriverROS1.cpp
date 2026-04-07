#include "sdpo_ratf_driver/SdpoRatfDriverROS1.h"

#include <std_msgs/Bool.h>





namespace sdpo_ratf_driver
{



SdpoRatfDriverROS1::SdpoRatfDriverROS1()
    : nh_priv_("~") , serial_comms_first_fault_(true)
{

  try
  {
    getParam();
  }
  catch (std::exception& e)
  {
    ROS_FATAL("[%s] Error reading the node parameters (%s)",
              ros::this_node::getName().c_str(),
              e.what());

    ros::shutdown();

    return;
  }

  sample_time_ = ros::Time::now();

  pub_mot_enc_ = nh_.advertise
      <sdpo_drivers_interfaces::MotEncArrayROS1>("motors_enc", 10);
  pub_switch_1_ = nh_.advertise<std_msgs::Bool>("switch_1_state", 10);
  pub_switch_2_ = nh_.advertise<std_msgs::Bool>("switch_2_state", 10);

  sub_mot_ref_ = nh_.subscribe
      <sdpo_drivers_interfaces::MotRefArrayROS1>(
          "motors_ref", 1,
          &SdpoRatfDriverROS1::subMotRef, this);



  rob_.setSerialPortName(serial_port_name_);
  rob_.openSerial();

  rob_.run = std::bind(&SdpoRatfDriverROS1::run, this);
  rob_.init();



  srv_solenoid_1_ = nh_.advertiseService("set_solenoid_1_state",
      &SdpoRatfDriverROS1::srvSolenoid1, this);
  srv_solenoid_2_ = nh_.advertiseService("set_solenoid_2_state",
      &SdpoRatfDriverROS1::srvSolenoid2, this);



  serial_port_timer_ = nh_.createTimer(ros::Duration(1),
      boost::bind(&SdpoRatfDriverROS1::checkSerialComms, this));


} // SdpoRatfDriverROS1::SdpoRatfDriverROS1()





void SdpoRatfDriverROS1::getParam()
{

  nh_priv_.param("encoder_res", encoder_res_, 64.0);
  nh_priv_.param("gear_reduction", gear_reduction_, 43.8);
  nh_priv_.param<std::string>(
      "serial_port_name", serial_port_name_, "/dev/ttyACM0");

  nh_priv_.param("mot_ctrl_freq", mot_ctrl_freq_, 100);
  nh_priv_.param("max_mot_pwm"  , max_mot_pwm_  , 1023);

  for (auto& m : rob_.mot)
  {
    m.encoder_res    = encoder_res_;
    m.gear_reduction = gear_reduction_;
    m.mot_ctrl_freq  = mot_ctrl_freq_;
    m.max_mot_pwm    = static_cast<int16_t>(max_mot_pwm_);
  }

  ROS_INFO("[%s] Encoder resolution: %lf (ticks/rev)",
           ros::this_node::getName().c_str(),
           rob_.mot[0].encoder_res);

  ROS_INFO("[%s] Gear reduction ratio: %lf (n:1)",
           ros::this_node::getName().c_str(),
           rob_.mot[0].gear_reduction);

  ROS_INFO("[%s] Motor Controller Frequency: %d (Hz)",
           ros::this_node::getName().c_str(),
           rob_.mot[0].mot_ctrl_freq);

  ROS_INFO("[%s] Motor Maximum PWM: %d (0..1023)",
           ros::this_node::getName().c_str(),
           rob_.mot[0].max_mot_pwm);

  ROS_INFO("[%s] Serial port: %s",
           ros::this_node::getName().c_str(),
           serial_port_name_.c_str());

} // void SdpoRatfDriverROS1::getParam()





void SdpoRatfDriverROS1::checkSerialComms()
{

  if (!rob_.isSerialOpen())
  {
    if (serial_comms_first_fault_)
    {
      serial_comms_first_fault_ = false;

      ROS_INFO("[%s] Couldn't open the serial port %s. Will retry every second.",
               ros::this_node::getName().c_str(),
               serial_port_name_.c_str());
    }

    rob_.mtx_.lock();
    rob_.stopMotors();
    rob_.mtx_.unlock();

    rob_.closeSerial();
    rob_.openSerial();

    if (rob_.isSerialOpen())
    {
      serial_comms_first_fault_ = true;

      ROS_INFO("[%s] Opened serial port %s.",
               ros::this_node::getName().c_str(),
               serial_port_name_.c_str());

      rob_.init();
    }
  }

} // void SdpoRatfDriverROS1::checkSerialComms()





void SdpoRatfDriverROS1::run()
{

  try
  {
    if (ros::Duration(ros::Time::now() - sample_time_) >
        ros::Duration(kWatchdogMotWRef))
    {
      rob_.mtx_.lock();
      rob_.stopMotors();
      rob_.mtx_.unlock();
    }
  }
  catch (std::exception& e)
  {
    ROS_WARN("[%s] Not possible to check the driver timeout condition (%s)",
             ros::this_node::getName().c_str(),
             e.what());

    sample_time_ = ros::Time::now();

    return;
  }

  pubMotEnc();
  pubSwitch();

} // void SdpoRatfDriverROS1::run()





void SdpoRatfDriverROS1::pubMotEnc()
{

  sdpo_drivers_interfaces::MotEncArrayROS1 msg;



  msg.stamp = ros::Time::now();
  msg.mot_enc.resize(4);



  rob_.mtx_.lock();

  for (int i = 0; i < 4; i++)
  {

    msg.mot_enc[i].enc_delta = rob_.mot[i].getEncTicksDeltaPub();

    msg.mot_enc[i].ticks_per_rev =
        rob_.mot[i].encoder_res * rob_.mot[i].gear_reduction;

    msg.mot_enc[i].ang_speed = rob_.mot[i].w;

  }

  rob_.mtx_.unlock();



  pub_mot_enc_.publish(msg);

} // void SdpoRatfDriverROS1::pubMotEnc()





void SdpoRatfDriverROS1::pubSwitch()
{
  std_msgs::Bool msg_1, msg_2;

  rob_.mtx_.lock();
  msg_1.data = rob_.switch_1_state;
  msg_2.data = rob_.switch_2_state;
  rob_.mtx_.unlock();

  pub_switch_1_.publish(msg_1);
  pub_switch_2_.publish(msg_2);
} // void SdpoRatfDriverROS1::pubSwitch()





void SdpoRatfDriverROS1::subMotRef(
    const sdpo_drivers_interfaces::MotRefArrayROS1::ConstPtr& msg)
{

  if (msg->ang_speed_ref.size() >= 4)
  {
    rob_.mtx_.lock();
    for (int i = 0; i < 4; i++)
    {
      rob_.mot[i].w_r = msg->ang_speed_ref[i].ref;
    }
    rob_.mtx_.unlock();

    sample_time_ = msg->stamp;
  }

} // void SdpoRatfDriverROS1::subMotRef(const sdpo_drivers_interfaces::MotRefArrayROS1::ConstPtr& msg)





bool SdpoRatfDriverROS1::srvSolenoid1(std_srvs::SetBool::Request& request,
                                      std_srvs::SetBool::Response& response)
{
  rob_.mtx_.lock();
  rob_.solenoid_1_state = request.data;
  rob_.mtx_.unlock();

  response.success = true;
  response.message = "";
  return true;
} // bool SdpoRatfDriverROS1::srvSolenoid1(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)





bool SdpoRatfDriverROS1::srvSolenoid2(std_srvs::SetBool::Request& request,
                                      std_srvs::SetBool::Response& response)
{
  rob_.mtx_.lock();
  rob_.solenoid_2_state = request.data;
  rob_.mtx_.unlock();

  response.success = true;
  response.message = "";
  return true;
} // bool SdpoRatfDriverROS1::srvSolenoid2(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)



} // namespace sdpo_ratf_driver
