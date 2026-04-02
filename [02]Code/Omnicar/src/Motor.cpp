#include "Motor.h"
#include "robot_config.h"

void Motor::init(uint8_t idx)
{
  m_idx = idx;

  pinMode(kMotDirPin[m_idx], OUTPUT);
  pinMode(kMotPWMPin[m_idx], OUTPUT);

  ledcSetup(kMotPWMCh[m_idx], kMotPWMFreq, kMotPWMRes);
  ledcAttachPin(kMotPWMPin[m_idx], kMotPWMCh[m_idx]);

  m_enable = true;
  m_pwm = 0;

  setPWM(0);
}





void Motor::setPWM(int32_t new_pwm)
{
  // Saturation
  if (new_pwm > kMotPWMMax)
  {
    new_pwm = kMotPWMMax;
  }
  else if (new_pwm < -kMotPWMMax)
  {
    new_pwm = -kMotPWMMax;
  }

  // Reset if disabled
  if (!m_enable)
  {
    new_pwm = 0;
  }

  // Limit PWM change
  if (kMotPWMDeltaMaxEnabled)
  {
    if (new_pwm - m_pwm > kMotPWMDeltaMax)
    {
      new_pwm = m_pwm + kMotPWMDeltaMax;
    }
    else if (new_pwm - m_pwm < -kMotPWMDeltaMax)
    {
      new_pwm = m_pwm - kMotPWMDeltaMax;
    }
  }

  // Set pwm
  if (m_enable)
  {
    if (new_pwm >= 0)
    {
      if (kMotPWMInvert[m_idx])
      {
        digitalWrite(kMotDirPin[m_idx], 1);
      }
      else
      {
        digitalWrite(kMotDirPin[m_idx], 0);
      }

      ledcWrite(kMotPWMCh[m_idx], new_pwm);
    }
    else
    {
      if (kMotPWMInvert[m_idx])
      {
        digitalWrite(kMotDirPin[m_idx], 0);
      }
      else
      {
        digitalWrite(kMotDirPin[m_idx], 1);
      }

      ledcWrite(kMotPWMCh[m_idx], (uint32_t)(-new_pwm));
    }
  }
  else
  {
    ledcWrite(kMotPWMCh[m_idx], new_pwm);
  }

  // Save pwm value
  m_pwm = new_pwm;
}
