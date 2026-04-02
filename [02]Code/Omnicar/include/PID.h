#ifndef PID_H
#define PID_H

#include <Arduino.h>

#include "robot_config.h"

class CtrlPID
{
 public:

  bool active;
  bool enable_cross_coupling;
  float kp, ki, kd, kf;
  float w, w_ref;
  float w2, w2_ref;   // the other motor
  float w_error_int;  // differential integral error
  float kp_cc;        // cross coupling controller gain
  float ki_cc;        // cross coupling controller gain
  float e, e_prev, e_sum;
  float dt;
  float u, u_max;

  // float hamm_vd, hamm_v0;

 public:

  void update(float new_w);
  void reset(void);

  void enable(bool e);

 private:

  // void hammerstein(float &mmot);
};

#endif