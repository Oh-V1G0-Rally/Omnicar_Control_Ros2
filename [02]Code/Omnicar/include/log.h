#pragma once

#include <Arduino.h>

#include "robot_config.h"

/*******************************************************************************
 * @brief Serial debug communication macro helpers (print, println)
 ******************************************************************************/
#ifdef SERIAL_DBG_ENABLED

#define SERIAL_INIT Serial.begin(256000);
#define SERIAL_PRINT(x) Serial.print(x)
#define SERIAL_PRINT_BASE(x, y) Serial.print(x, y)
#define SERIAL_PRINTLN(x) Serial.println(x)
#define SERIAL_PRINTLN_BASE(x, y) Serial.println(x, y)

#else

#define SERIAL_INIT
#define SERIAL_PRINT(x)
#define SERIAL_PRINT_BASE(x, y)
#define SERIAL_PRINTLN(x)
#define SERIAL_PRINTLN_BASE(x, y)

#endif

/*******************************************************************************
 * @brief Serial time measuring macro helpers
 ******************************************************************************/

#ifdef SERIAL_TIME_DBG_ENABLED

#ifndef SERIAL_DBG_ENABLED
#undef SERIAL_INIT
#define SERIAL_INIT Serial.begin(256000);
#endif

#define SERIAL_LOG_TIME(description, cmd) \
  {                                       \
    unsigned long start = micros();       \
    cmd;                                  \
    unsigned long stop = micros();        \
    unsigned long delta = stop - start;   \
    Serial.print(description);            \
    Serial.println(delta);                \
  }

#else

#define SERIAL_LOG_TIME(description, cmd) cmd

#endif
