#include <Arduino.h>
#include "robot_config.h"
#include "Encoder.h"

#pragma GCC optimize ("O3")





const int8_t kEncoderTable[16] =
{
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0
};

//void updateEncodersState(void)

void IRAM_ATTR updateEncodersState(void)
{
  uint8_t b = 0, new_state;

  for (int idx = 0; idx < kNumMot; idx++)
  {
    if (digitalRead(kMotEncPinA[idx])) b |= (1 << (idx * 2));
    if (digitalRead(kMotEncPinB[idx])) b |= (1 << ((idx * 2) + 1));
  }

  for (int idx = 0; idx < kNumMot; idx++)
  {
    new_state = b & 0b00000011;
    encoders[idx].updateDelta(new_state);
    b = (b >> 2);
  }
}





void Encoder::updateDelta(uint8_t &new_state)
{
  delta += kEncoderTable[state | new_state];
  state = new_state << 2;
}





void Encoder::updateTick(void)
{
  tick_last = tick;
  noInterrupts();
  odo = delta;
  delta = 0;
  interrupts();
  tick += odo;
}
