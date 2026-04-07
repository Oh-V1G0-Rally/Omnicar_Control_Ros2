#include "sdpo_ratf_driver/Robot5dpoRatfTune.h"





namespace sdpo_ratf_driver
{



Robot5dpoRatfTune::Robot5dpoRatfTune() : Robot5dpoRatf()
{

} // Robot5dpoRatfTune::Robot5dpoRatfTune()





Robot5dpoRatfTune::Robot5dpoRatfTune(std::string serial_port_name)
    : Robot5dpoRatf(serial_port_name)
{

} // Robot5dpoRatfTune::Robot5dpoRatfTune(std::string serial_port_name)





void Robot5dpoRatfTune::stopMotors()
{
  for(auto& m : mot)
  {
    m.w_r = 0;
    m.pwm = 0;
  }
} // void Robot5dpoRatfTune::stopMotors()





void Robot5dpoRatfTune::sendSerialData()
{
  mtx_.lock();

  for (uint8_t i = 0; i < 4; i++)
  {
    serial_cfg_->channel_K = mot[i].pwm & 0xFFFF;
    serial_cfg_->channel_K = serial_cfg_->channel_K | ((i & 0x03) << 24);
    serial_async_->writeString(SendChannel('K'));
  }

  serial_cfg_->channel_L = solenoid_1_state? 1 : 0;
  serial_cfg_->channel_M = solenoid_2_state? 1 : 0;

  mtx_.unlock();

  serial_async_->writeString(SendChannel('L'));
}



} // namespace sdpo_ratf_driver
