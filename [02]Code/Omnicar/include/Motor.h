#pragma once

#include <Arduino.h>

#include "robot_config.h"

/**
 * @brief Motor driver data structure (Cytron-based drivers)
 *
 * Interface: PWM, direction
 * - Cytron MDD10A 10Amp 5V-30V DC Motor Driver (2 Channels)
 *   - https://www.cytron.io/p-10amp-5v-30v-dc-motor-driver-2-channels
 * - PWM defines the duty cycle
 * - direction defines the direction of the motor
 */
struct Motor
{
 public:

  bool m_enable = false;  //!< enable / disable motor
  uint8_t m_idx;          //!< motor index
  int32_t m_pwm = 0;      //!< pwm value (-kMotPWMMax .. kMotPWMMax)

 public:

  /**
   * @brief initialize the motor driver
   *
   * - set PWM and dir pins to OUTPUT
   * - initialize the ESP32 LED Control for PWM automatic generation
   * - enable motors + set PWM to 0
   *
   * @param[in] idx motor index
   */
  void init(uint8_t idx);

  /**
   * @brief set PWM value
   *
   * - check saturation
   * - limit PWM change if enabled
   * - set PWM duty cycle and direction output pin
   *
   * @param[in] pwm pwm value (-kMotPWMMax .. kMotPWMMax)
   */
  void setPWM(int32_t pwm);

  /**
   * @brief stop motor (ignore possible delta maximum variation mechanism)
   */
  inline void stop()
  {
    m_pwm = 0;

    digitalWrite(kMotDirPin[m_idx], 0);
    ledcWrite(kMotPWMCh[m_idx], 0);
  }
};
