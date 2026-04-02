#ifndef ROBOT_H
#define ROBOT_H

#include "robot_config.h"
#include "Encoder.h"
#include "Motor.h"
#include "CtrlPID.h"
#include "MRAC.h"





class Robot
{
 public:
  uint32_t dt;
  Encoder enc[kNumMot];
  Motor mot[kNumMot];
  CtrlPID pid[kNumMot];
  MRAC mrac[kNumMot];
  double dist2line, angle2line;
  int bad_count;
  double wheelbase = 0.095;
  double x = 0;
  double y = 0;
  double theta = 0;

  void (*serialWriteChannel)(char channel, int32_t value);

 public:
  float tomaW;
  void init(void (*serialWriteChannelFunction)(char c, int32_t v));

  void update(uint32_t &delta);
  void send(void);
  void stop(void);

  void setMotorWref(uint8_t index, float new_w_r);
  void setMotorPWM(uint8_t index, int16_t pwm);

 private:
  void initEnc();
  void initCtrlPID(uint8_t index);
  void initMRAC(uint8_t index);
  void updateState(uint32_t ticks_left, uint32_t ticks_right);
};





#endif
