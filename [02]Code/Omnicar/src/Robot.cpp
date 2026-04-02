#include "Robot.h"

// Fix: Define missing constants and flags
bool PID_enable = true;
const int kRobotSensSwitchPin = 35; // Default input pin if not defined

// // Fix: Declare external encoder update function (assumed defined in main.cpp)
// extern void updateEncodersState();

// Fix: ESP32 Hardware Timer handle and ISR
hw_timer_t * encoder_timer = NULL;

void IRAM_ATTR onEncoderTimer() {
  updateEncodersState();
}


void Robot::init(void (*serialWriteChannelFunction)(char c, int32_t v))
{
  uint8_t i;

  // Serial write channels function
  serialWriteChannel = serialWriteChannelFunction;

  // General inputs / outputs
  // - solenoid
  //pinMode(kRobotSensSwitchPin, INPUT_PULLUP);
  //pinMode(kRobotActSolenoidPin, OUTPUT);
  //digitalWrite(kRobotActSolenoidPin, 0);

  // Encoders
  initEnc();
  updateEncodersState();

  for (i = 0; i < kNumMot; i++)
  {
    encoders[i].delta = 0;
  }

  // Fix: Replace Timer1 with ESP32 Hardware Timer
  // Timer 0, prescaler 80 (1MHz clock), count up
  encoder_timer = timerBegin(0, 80, true);
  //timerAttachInterrupt(encoder_timer, &updateEncodersState, true);
  timerAttachInterrupt(encoder_timer, &onEncoderTimer, true); // Usa il wrapper locale IRAM
  timerAlarmWrite(encoder_timer, 100, true); // calls every 50 us (safer than 5us)
  timerAlarmEnable(encoder_timer);

  // Motors
  for (i = 0; i < kNumMot; i++)
  {
    // Inizializza passando l'indice del motore (0-3)
    mot[i].init(i);
  }

  // Controllers
  for (i = 0; i < kNumMot; i++)
  {
    if (PID_enable)
      initCtrlPID(i);
    else
      initMRAC(i);
  }
}





void Robot::update(uint32_t &delta)
{
  uint8_t i;
  dt = delta;

  // // Encoders
  // for (i = 0; i < kNumMot; i++)
  // {
  //   enc[i].updateTick();
  // }

  // updateState(enc[0].odo, enc[1].odo);

  // Salva tick precedenti prima di updateTick()
    int32_t prev_tick[kNumMot];
    for (i = 0; i < kNumMot; i++) prev_tick[i] = enc[i].tick;

    for (i = 0; i < kNumMot; i++) enc[i].updateTick();

    int32_t dtick_left  = enc[0].tick - prev_tick[0];
    int32_t dtick_right = enc[1].tick - prev_tick[1];
    updateState(dtick_left, dtick_right); // passa delta, non odo assoluto

  // Controllers
  for (i = 0; i < kNumMot; i++)
  {
    if (PID_enable)
      pid[i].update(enc[i].odo * kEncImp2MotW);
    else
       mrac[i].compute(enc[i].odo * kEncImp2MotW);
  }
  tomaW=enc[1].odo *kEncImp2MotW;
  // Actuators
  for (i = 0; i < kNumMot; i++)
  {
    if (pid[i].active)
    {
      mot[i].setPWM( round( kMotV2MotPWM * pid[i].m ) );
    }
    else if(mrac[i].active)
    {
      mot[i].setPWM( round( kMotV2MotPWM * mrac[i].u ) );
    }
  }
}





void Robot::send(void)
{
  for (int idx = 0; idx < kNumMot; idx++)
  {
    (*serialWriteChannel)('g'+idx, enc[idx].tick);
  }

  (*serialWriteChannel)('k', dt);

  (*serialWriteChannel)('s', (digitalRead(kRobotSensSwitchPin) << 0));
  (*serialWriteChannel)('z', pid[0].w);
}





void Robot::stop(void)
{
  uint8_t i;

  for (i = 0; i < kNumMot; i++)
  {
    setMotorPWM(i, 0);
  }
}





void Robot::setMotorWref(uint8_t index, float new_w_r)
{
  if (PID_enable)
  {
    pid[index].enable(true);
    mrac[index].enable(false);
    pid[index].w_ref = new_w_r;
  }
  else
  {
    pid[index].enable(false);
    mrac[index].enable(true);
    mrac[index].r = new_w_r;
  }
  
  
  
}





void Robot::setMotorPWM(uint8_t index, int16_t pwm)
{
  pid[index].enable(false);
  mrac[index].enable(false);
  mot[index].setPWM(pwm);
}





void Robot::initEnc()
{
  for (int idx = 0; idx < kNumMot; idx++)
  {
    pinMode(kMotEncPinA[idx], INPUT_PULLUP);
    pinMode(kMotEncPinB[idx], INPUT_PULLUP);
  }
}





void Robot::initCtrlPID(uint8_t index)
{
  pid[index].active = false;
  pid[index].kp = kMotCtrlKc;

  if (kMotCtrlTi == 0)
  {
    pid[index].ki = 0;
  }
  else
  {
    pid[index].ki = kMotCtrlKc / kMotCtrlTi;
  }

  pid[index].kd = 0;
  pid[index].kf = kMotCtrlKf;
  pid[index].dt = kMotCtrlTime;

  pid[index].m_max = kMotVmax;

  pid[index].hamm_vd = 0;
  pid[index].hamm_v0 = 0;

  pid[index].reset();
}

void Robot::initMRAC(uint8_t index)
{
  mrac[index].init(kMotModelKp,kMotModelTau,kMotCtrlTime);
  pid[index].active = false;
  mrac[index].m_max = kMotVmax;

  mrac[index].hamm_vd = 0;
  mrac[index].hamm_v0 = 0;

  //pid[index].reset();
}

void Robot::updateState(uint32_t ticks_left, uint32_t ticks_right)
{

  // Fix: Use wheel diameter from config instead of hardcoded 0.05
  double d1 = double(ticks_left)/kMotEncRes*2*M_PI*kRobotWhD[0]/2;
  double d2 = double(ticks_right)/kMotEncRes*2*M_PI*kRobotWhD[0]/2;
  double delta_d = (d2+d1)/2;
  double delta_theta = (d2-d1)/(2*wheelbase);
  x = x + delta_d*cos(theta + delta_theta/2);
  y = y + delta_d*sin(theta + delta_theta/2);
  theta = theta + delta_theta;


}
