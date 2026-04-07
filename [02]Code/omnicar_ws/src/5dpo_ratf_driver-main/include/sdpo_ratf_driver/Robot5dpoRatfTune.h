#pragma once

#include "sdpo_ratf_driver/Robot5dpoRatf.h"





namespace sdpo_ratf_driver
{



class Robot5dpoRatfTune : public Robot5dpoRatf
{

 public:

  Robot5dpoRatfTune();
  Robot5dpoRatfTune(std::string serial_port_name);

  void stopMotors() final;

 protected:

  void sendSerialData() final;

};// class Robot5dpoRatfTune : public Robot5dpoRatf



} // namespace sdpo_ratf_driver
