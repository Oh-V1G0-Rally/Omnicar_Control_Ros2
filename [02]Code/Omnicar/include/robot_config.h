#ifndef ROBOTCONFIG_H
#define ROBOTCONFIG_H

#include <Arduino.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

/******************************************************************************
 * Configuration Mode
 ******************************************************************************/
#define CONFIG_ROS            //!< Firmware communicates via ROS2/Serial

/******************************************************************************
 * Robot Geometry & Kinematics (Omnidirectional 4WD)
 ******************************************************************************/
constexpr int kNumMot = 4;
const float kRobotL[] = { 
  0.185,    //!< d_x: front-back wheel distance / 2 (m)
  0.220     //!< d_y: left-right wheel distance / 2 (m)
};

const float kRobotWhD[] = { 0.060, 0.060, 0.060, 0.060 }; //!< Wheel diameters (m)
const float kRobotBattVnom = 11.1f;                       //!< Battery 3S1P

/*******************************************************************************
 * @brief MQTT configuration
 *
 * Please create the hotspot in your PC with the following configuration:
 * - Wi-Fi
 *   - SSID   : "esp32-5dpo-hotspot" (kMQTTWiFiSSID)
 *   - Mode   : Access Point
 *   - Band   : B/G (2.4GHz)
 *   - Channel: 3 (2422MHz) (check WiFi analyser to see the best channel)
 * - Wi-Fi Security
 *   - Security: WPA/WPA2 Personal
 *   - Password: 5dpo5dpo (kMQTTWiFiPass)
 ******************************************************************************/
constexpr char kMQTTWiFiSSID[] =
    "C2SR_Lab_Exp";                    //!< Wi-Fi Hotspot SSID
constexpr char kMQTTWiFiPass[] = "c2sr-robotics";   //!< Wi-Fi password
constexpr char kMQTTServerIP[] = "192.168.31.144";  //!< MQTT server IP (PC)

/******************************************************************************
 * Pinout Mapping - OMNICAR ESP32
 ******************************************************************************/
 
// Fallback per schede ESP32 che non definiscono LED_BUILTIN nativamente
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// --- MOTORS (PWM & DIRECTION) ---
// Utilizziamo l'API LEDC di ESP32 per PWM ad alta risoluzione
// constexpr uint32_t kMotPWMFreq = 20000;     //!< REAL:
constexpr uint32_t kMotPWMFreq = 19500;     //!< DEBUG: 50Hz per test oscilloscopio (era 19500)
constexpr uint8_t kMotPWMRes = 12;       //!< Resolution (12 bits = 0-4095)
const uint8_t kMotPWMPin[] = { 13, 14, 26, 33 }; // FL, FR, RL, RR
const uint8_t kMotDirPin[] = { 12, 27, 25, 32 }; // FL, FR, RL, RR
const uint8_t kMotPWMCh[]  = { 0, 1, 2, 3 };     // Canali LEDC indipendenti

// --- ENCODERS ---
// Nota: 34, 35, 36, 39 richiedono pull-up esterni!
const uint8_t kMotEncPinA[] = { 16, 5, 19, 22 }; // FL, FR, RL, RR
const uint8_t kMotEncPinB[] = { 17, 18, 21, 23 }; // FL, FR, RL, RR
// const uint8_t kMotEncPin[kNumMot][2] = {
//  {16, 17}, {5, 18}, {19, 21}, {22, 23}
//};

// --- PERIPHERALS ---
//const uint8_t kI2C_SDA = 21; // Attenzione: condiviso con Encoder RR A se non rimappato
//const uint8_t kI2C_SCL = 22; // Attenzione: condiviso con Encoder RR B se non rimappato

/******************************************************************************
 * Motor & Control Parameters
 ******************************************************************************/
const float kMotNgear  = 18.75f;      //!< Gear reduction ratio
// #TOCHECK 
//const float kMotEncRes = 64.0f * 4.0f; //!< Quad pulses per revolution
const float kMotEncRes = 64.0f; //!< Quad pulses per revolution

// PWM Parametrization (ESP32 Specific)
constexpr int32_t kMotPWMMax   = (1 << kMotPWMRes) - 1;

// Parametri aggiuntivi per la nuova implementazione Motor
const bool kMotPWMInvert[] = { false, false, false, false }; // Inversione direzione per ogni motore
const bool kMotPWMDeltaMaxEnabled = false;                    // Abilita limitazione accelerazione
const int kMotPWMDeltaMax = 1000;                             // Variazione massima PWM per ciclo

// Low Level Controller (Timing)
const unsigned long kMotCtrlFreq = 50UL;               //!< Loop a 50Hz (minimo ROS)
const float kMotCtrlTime = 1.0f / kMotCtrlFreq;
const unsigned long kMotCtrlTimeUs = 1000000UL / kMotCtrlFreq;
const unsigned long kMotCtrlTimeout = 100UL;           //!< Watchdog (ms)
const bool kMotCtrlTimeoutEnable = true;

const unsigned long kMotCtrlLEDOkFreq = 4UL;  //!< heartbeat LED frequency (Hz)
const unsigned long kMotCtrlLEDOkCount = 1000000UL / kMotCtrlLEDOkFreq / kMotCtrlTimeUs / 2;

// Motor Model (from previous Arduino calibration)
const float kMotModelKp  = 5.0970f;  //!< Gain (rad.s^-1 / V)
const float kMotModelTau = 0.0900f;  //!< Time constant (s)
const float kMotModelLag = 0.0000;   //!< lag lag (s)
const float kMotVmax     = 11.1f;    //!< Max battery voltage
constexpr float kMotWmin = -50.0f;  //!< minimum motor angular velocity (rad/s)


// PI Gains (Derived via IMC Tuning) 
const float kMotCtrlTauCl = kMotModelTau / 0.5f;                            //IMC desired time constant for the closed-loop (s))
const float kMotCtrlKcKp = kMotModelTau / (kMotCtrlTauCl + kMotModelLag);   //IMC tunning: Kc_PI * Kp_plant
const float kMotCtrlKc    = (kMotModelTau / kMotCtrlTauCl) / kMotModelKp;   //PI proportional gain (V / rad.s^(-1))
const float kMotCtrlTi    = kMotModelTau;                                   //PI integration time (s)
const float kMotCtrlKf    = 0.0f / kMotModelKp;                             // Feed-Forward gain

/******************************************************************************
 * Conversion Constants
 ******************************************************************************/
// Ticks to Motor Angular Speed (rad/s)
// Costante "Legacy" per tempo fisso (potrebbe essere rimossa in futuro)
const float kEncImp2MotW = (2.0f * PI * 1000000.0f) / (kMotCtrlTimeUs * kMotNgear * kMotEncRes);

// NUOVA: Conversione Ticks -> Radianti (Indipendente dal tempo)
const float kEncImp2Rad = (2.0f * PI) / (kMotNgear * kMotEncRes);

// Volts to PWM (0..1023)
const float kMotV2MotPWM = kMotPWMMax * 1.0 / kRobotBattVnom;

// --- SPI CUSTOM CONFIGURATION ---
// Remapping necessario perché Encoder RL occupa i pin 18 (SCK) e 19 (MISO)
const int8_t kSpiSCK  = 17; // Pin alternativo (TX2)
const int8_t kSpiMISO = 16; // Pin alternativo (RX2)
const int8_t kSpiMOSI = 23; // Default VSPI MOSI (OK)
const int8_t kSpiCS   = 5;  // Default VSPI CS (OK)

#endif
