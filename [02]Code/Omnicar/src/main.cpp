#include <Arduino.h>
#include <channels.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "Adafruit_MotorShield.h"
#include "robot_config.h"
#include "Robot.h"
#include "MotorAutotuner.h"
// #include <WebServer.h>
// #include "SPIFFS.h"

/******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************/
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

unsigned long current_micros = 0, previous_micros = 0;
unsigned long last_motor_update_millis = 0;
bool timeout = false;
channels_t serial_channels;
uint8_t builtin_led_state;

Robot robot;
Encoder *encoders = robot.enc;
MotorAutotuner tuner(&robot);
// File logFile; // File per il salvataggio offline
// WebServer server(80); // Oggetto per il web server sulla porta 80
bool is_boot_test_pending = false;       // Flag per il countdown del test
unsigned long boot_test_start_millis = 0; // Timestamp inizio countdown
bool is_pid_test_running = false;         // Stato esecuzione test PID
unsigned long pid_test_start_time = 0;    // Tempo inizio test PID


/******************************************************************************
 * FUNCTIONS HEADERS
 ******************************************************************************/
void processSerialPacket(char channel, uint32_t value, channels_t& obj);
void serialWrite(uint8_t b);
void serialWriteChannel(char channel, int32_t value);
void serialRead();
void checkMotorsTimeout();
void setupWiFi();
void setupOTA();
// void setupWebServer();
void handleOfflineTest();
void handleSerialInput();

/******************************************************************************
 * IMPLEMENT
 ******************************************************************************/
#pragma region SETUP_LOOP_RASPBERRY_PI - STD
void setup() {
  // //DEBUG
  // // Spegnimento forzato del Built-in LED
  // builtin_led_state = LOW;
  // pinMode(LED_BUILTIN, OUTPUT);
  // digitalWrite(LED_BUILTIN, builtin_led_state);

  // Robot
  robot.init(serialWriteChannel);

  // Serial communication
  Serial.begin(115200);
  serial_channels.init(processSerialPacket, serialWrite);
  serialWriteChannel('r', 0);

  setupWiFi();
  setupOTA();

  //MY - Debug
  //Serial.println("--- OMNICAR MOTOR TEST START ---");
  //Serial.flush(); // Ensure message is sent before robot.init() potentially crashes

  // Inizializza il robot passando la funzione di callback per la seriale
  //Serial.println("Robot initialized.");


  // Test PWM motors
  /*robot.setMotorPWM(0, 0);
  robot.setMotorPWM(1, 0);
  robot.setMotorPWM(2, 0);
  robot.setMotorPWM(3, 0);*/

  // Initialization
  current_micros = micros();
  previous_micros = current_micros;
  last_motor_update_millis = millis();
}

void loop() {
  ArduinoOTA.handle();

  // static unsigned long blink_led_decimate = 0;
  uint32_t delta;

  serialRead();

  current_micros = micros();
  delta = current_micros - previous_micros;
 
  if (delta > kMotCtrlTimeUs) {
    if (false) {
      checkMotorsTimeout();
    }

    if (!timeout) {
      previous_micros = current_micros;
      
      // Update and send data
      robot.update(delta);
      robot.send();

      // Debug (Serial Monitor)
      //serialWrite('\n');

      // // Blink LED
      // blink_led_decimate++;
      // if (blink_led_decimate >= kMotCtrlLEDOkCount) {
      //   if (builtin_led_state == LOW) {
      //     builtin_led_state = HIGH;
      //   } else {
      //     builtin_led_state = LOW;
      //   }
      //   digitalWrite(LED_BUILTIN, builtin_led_state);
      //   blink_led_decimate = 0;
      // }
    }
  }

}
#pragma endregion

#pragma region SETUP_LOOP_RASPBERRY_PI - TEST
// void setup() {
//   // Built-in LED
//   /*builtin_led_state = LOW;
//   pinMode(LED_BUILTIN, OUTPUT);
//   digitalWrite(LED_BUILTIN, builtin_led_state);*/

//   // Serial communication
//   Serial.begin(115200);
//   serial_channels.init(processSerialPacket, serialWrite);

//   // --- WIFI & OTA SETUP ---
//   // È FONDAMENTALE connettersi al WiFi PRIMA di inizializzare OTA
//   setupWiFi();
//   setupOTA();

//   Serial.println("WiFi configured.");

//   //MY - Debug
//   Serial.println("--- OMNICAR MOTOR TEST START ---");
//   Serial.flush(); // Ensure message is sent before robot.init() potentially crashes

//   // Inizializza il robot passando la funzione di callback per la seriale
//   robot.init(serialWriteChannel);
//   Serial.println("Robot initialized.");

//   // Reset signal
//   serialWriteChannel('r', 0);

//   // Initialization
//   current_micros = micros();
//   previous_micros = current_micros;
//   last_motor_update_millis = millis();
// }

// void loop() {
//     // La gestione degli aggiornamenti OTA deve essere eseguita il più frequentemente possibile.
//     ArduinoOTA.handle();

//     current_micros = micros();
//     uint32_t delta;

//     serialRead();

//     current_micros = micros();
//     delta = current_micros - previous_micros;

//     // 1. Controllo della temporizzazione basato su kMotCtrlTime (es. 20ms = 50Hz)
//     if (delta>= kMotCtrlTimeUs) {
//         previous_micros = current_micros;

//         // Disabilitato forzatamente per permettere il test manuale (senza comandi ROS in entrata)
//         if (false && kMotCtrlTimeoutEnable) {
//           checkMotorsTimeout();
//         }

//         if (!timeout) {
//           robot.update(delta);

//           // robot.enc[i].odo contiene i tick accumulati dall'ultima lettura
//           for (int i = 0; i < kNumMot; i++) {
//               serial_channels.send('0' + i, robot.enc[i].odo);
//               //Serial.printf("0" + i, robot.enc[i].odo);
//           }
//         }
//       }
//     // Gestione comandi in entrata (da ROS verso ESP32)
//     handleSerialInput();
//     // La chiamata a ArduinoOTA.handle() è stata spostata all'inizio del loop per garantirne l'esecuzione.
// }
#pragma endregion

#pragma region SETUP_ESP32
// void setup() {
//   // Inizializza la seriale per il monitoraggio
//   Serial.begin(115200);
//   //while (!Serial) delay(10); // Attendi l'apertura del monitor seriale
//   serial_channels.init(processSerialPacket, serialWrite);
  
//   // --- SPIFFS SETUP ---
//   // if (!SPIFFS.begin(true)) {
//   //   Serial.println("SPIFFS Mount Failed");
//   // }
//   // Configura il pulsante BOOT (GPIO 0)
//   pinMode(0, INPUT_PULLUP);

//   // --- WIFI & OTA SETUP ---
//   setupWiFi();
//   setupOTA();

//   // --- WEB SERVER SETUP ---
//   // setupWebServer();

//   Serial.println("--- OMNICAR MOTOR TEST START ---");
//   Serial.println("Initializing robot hardware...");
//   Serial.flush(); // Ensure message is sent before robot.init() potentially crashes

//   // Inizializza il robot passando la funzione di callback per la seriale
//   robot.init(serialWriteChannel);

//   Serial.println("Robot initialized.");
//   Serial.println("PID Control Mode Ready.");
  
//   // Reset signal
//   serialWriteChannel('r', 0);

//   // Initialization
//   current_micros = micros();
//   previous_micros = current_micros;
//   last_motor_update_millis = millis();
// }
#pragma endregion

#pragma region LOOP_PID
// // LOOP con MotorAutotuner
// void loop() {
//   ArduinoOTA.handle();
//   // server.handleClient(); // Gestisce le richieste web in arrivo

//   // 1. GESTIONE TRIGGER TEST (Pulsante BOOT)
//   handleOfflineTest(); // Controlla pressione tasto (non bloccante)

//   // Setpoint
//   const float target_speed = 20.0f; // Target: 15 rad/s (ca. 150 rpm)

//   // Gestione Countdown 5 secondi
//   if (is_boot_test_pending) {
//     if (millis() - boot_test_start_millis > 5000) {
//       is_boot_test_pending = false;
      
//       // AVVIO TEST E LOGGING
//       // logFile = SPIFFS.open("/log.csv", "w");
//       if (true) { // Condizione sempre vera per avviare il test
//         // Scriviamo l'intestazione CSV manualmente (compatibile con plot_motor_response.py)
//         // logFile.println("Time_ms;Ref_RadS;Spd_M0;Spd_M1;Spd_M2;Spd_M3");
//         Serial.println("Time_ms;Ref_RadS;Spd_M0;Spd_M1;Spd_M2;Spd_M3"); // Invio diretto su seriale
        
//         is_pid_test_running = true;
//         pid_test_start_time = millis();
//         previous_micros = micros(); // Reset timing per il primo ciclo
        
//         // --- CONFIGURAZIONE SETPOINT PID ---
//         for (int i = 0; i < kNumMot; i++) {
//           robot.setMotorWref(i, target_speed);
//         }
//         Serial.printf("PID Test Started (Ref: %.4f rad/s\n)", target_speed);
//       }
//     }
//   }
  
//   // 2. ESECUZIONE TEST PID (Se attivo)
//   if (is_pid_test_running) {
//     // Controllo ABORT manuale su seriale (tasto 'x')
//     if (Serial.available() > 0 && (Serial.peek() == 'x' || Serial.peek() == 'X')) {
//         Serial.read();
//         is_pid_test_running = false;
//         robot.stop();
//         return;
//     }
    
//     current_micros = micros();
//     uint32_t delta = current_micros - previous_micros;

//     if (delta > kMotCtrlTimeUs) {
//       previous_micros = current_micros;
      
//       // Esegue il ciclo di controllo PID del robot
//       robot.update(delta);
      
//       // Salvataggio dati su file CSV
//       // if (logFile) {
//       unsigned long t = millis() - pid_test_start_time;
//       // Log: Tempo;Riferimento;Velocità Reali M0..M3
//       Serial.printf("%lu;%.4f", t, target_speed); 
//       for(int i=0; i<kNumMot; i++) {
//           // Calcolo velocità in rad/s: Ticks * CostanteConversione
//           // Usa il tempo dinamico (delta) per loggare la velocità corretta
//           // float speed = ((float)robot.enc[i].odo * kEncImp2Rad) / (delta / 1000000.0f);
//           float speed = ((float)robot.enc[i].odo * kEncImp2MotW);
//           Serial.printf(";%.4f", speed);
//       }
//       Serial.println();
//       // }

//       // Stop automatico dopo 4 secondi
//       if (millis() - pid_test_start_time > 4000) {
//          is_pid_test_running = false;
//          robot.stop();
//       }
//     }
//   }
//   // 3. CHIUSURA LOG (Se test appena finito)
//   // else if (logFile) {
//   //   logFile.close();
//   //   Serial.println("\n--- TEST FINITO (Streaming Completato) ---");
//   //   // Serial.println("Usa il comando 'r' per scaricare i dati.");
//   // }
//   // 4. PID CONTROL LOOP (Normale funzionamento remoto)
//   // else {
//   //     serialRead(); // Legge pacchetti binari (Serial Channels) solo se non in test

//   //     current_micros = micros();
//   //     uint32_t delta = current_micros - previous_micros;

//   //     Serial.println("Secondo IF");

//   //     if (delta > kMotCtrlTimeUs) {
//   //       if (kMotCtrlTimeoutEnable) {
//   //         checkMotorsTimeout();
//   //       }

//   //       if (!timeout) {
//   //         previous_micros = current_micros;
          
//   //         // Update PID e invio telemetria
//   //         robot.update(delta);
//   //         robot.send(); 
//   //       }
//   //     }
//   // }
// }

#pragma endregion

#pragma region LOOP_MANUALE
// // LOOP MANUALE
// void loop() {
//   uint32_t dt = 0; // Variabile delta time per robot.update
//   int idx_mot = 0;

//   // Richiedi input utente
//   Serial.println("\n=========================================");
//   Serial.println("   TEST MANUALE ENCODER (Ruota a mano)");
//   Serial.println("   Inserisci indice motore (0-3) per iniziare.");
//   Serial.println("=========================================");

//   while (Serial.available() == 0) {
//     delay(10);
//   }
  
//   idx_mot = Serial.parseInt();
//   // Pulisci il buffer seriale (rimuovi newline)
//   while (Serial.available()) { Serial.read(); }

//   if (idx_mot >= 0 && idx_mot < kNumMot) {
//     Serial.printf("--> MONITORAGGIO MOTORE %d ATTIVO\n", idx_mot);
//     Serial.println("    Ruota la ruota manualmente.");
//     Serial.println("    Invia un qualsiasi carattere per uscire e cambiare motore.");
    
//     // Assicurati che il motore sia spento (libero)
//     robot.setMotorPWM(idx_mot, 0);

//     // Loop finché non si riceve input seriale
//     while (Serial.available() == 0) {
//       robot.update(dt); // Aggiorna i contatori degli encoder (trasferisce da delta a tick)
//       Serial.printf("MOT[%d] | Ticks: %d | Odo: %d\n", 
//                     idx_mot, robot.enc[idx_mot].tick, robot.enc[idx_mot].odo);
//       delay(20); // Stampa ogni 100ms
//     }

//     // Pulisci il buffer all'uscita
//     while (Serial.available()) { Serial.read(); }
//     Serial.println("--> Uscita monitoraggio.");

//   } else {
//     Serial.printf("Indice %d non valido! Inserire 0, 1, 2 o 3.\n", idx_mot);
//   }
// }
#pragma endregion

#pragma region LOOP_TEST_SINGOLO
// // LOOP SINGOLO su richiesta
// void loop() {
//   const int PWM_test = 4000; // PWM moderato per il test (circa 35%)
//   uint32_t dt = 0; // Variabile delta time per robot.update
//   unsigned long start_time;
//   int idx_mot = 0;

//   // Richiedi input utente
//   Serial.println("\n--- Inserisci indice motore (0-3) per avviare il test: ---");
//   while (Serial.available() == 0) {
//     ArduinoOTA.handle(); // Permette l'upload mentre si attende l'input
//     delay(10);
//   }
  
//   idx_mot = Serial.parseInt();
//   // Pulisci il buffer seriale (rimuovi newline)
//   while (Serial.available()) { Serial.read(); }

//   if (idx_mot >= 0 && idx_mot < kNumMot) {

//     // --- AVANTI ---
//     Serial.printf("Motor %d: FORWARD (+%d)\n", idx_mot, PWM_test);
//     robot.setMotorPWM(idx_mot, PWM_test); 
    
//     // Esegui per 3 secondi monitorando l'encoder
//     start_time = millis();
//     while(millis() - start_time < 3000) {
//       ArduinoOTA.handle(); // Permette l'upload durante il movimento
//       robot.update(dt); // Aggiorna i contatori degli encoder (trasferisce da delta a tick)
//       Serial.printf("MOT[%d] >> PWM: %d | Ticks: %d | Odo: %d\n", 
//                     idx_mot, PWM_test, robot.enc[idx_mot].tick, robot.enc[idx_mot].odo);
//       delay(100); // Stampa ogni 100ms
//     }

//     // --- STOP ---
//     robot.setMotorPWM(idx_mot, 0);
//     delay(500); // Breve pausa
//     Serial.printf("MOT[%d] STOPPED. Final Ticks: %d\n", idx_mot, robot.enc[idx_mot].tick);
//     delay(500);

//     // --- INDIETRO ---
//     Serial.printf("Motor %d: BACKWARD (-%d)\n", idx_mot, PWM_test);
//     robot.setMotorPWM(idx_mot, -PWM_test); 
    
//     // Esegui per 3 secondi monitorando l'encoder
//     start_time = millis();
//     while(millis() - start_time < 3000) {
//       ArduinoOTA.handle(); // Permette l'upload durante il movimento
//       robot.update(dt);
//       Serial.printf("MOT[%d] >> PWM: %d | Ticks: %d | Odo: %d\n", 
//                     idx_mot, -PWM_test, robot.enc[idx_mot].tick, robot.enc[idx_mot].odo);
//       delay(100);
//     }

//     // --- STOP ---
//     robot.setMotorPWM(idx_mot, 0);
//     delay(1000);
//     Serial.println("--- Test Complete. ---");
//   } else {
//     Serial.printf("Indice %d non valido! Inserire 0, 1, 2 o 3.\n", idx_mot);
//   }
// }
#pragma endregion

#pragma region LOOP_TEST_SEQUENZA_TUTTI_MOTORI
// void loop() {
//   const int PWM_test = 1500; // PWM moderato per il test (circa 35%)
//   uint32_t dt = 0; // Variabile delta time per robot.update
//   unsigned long start_time;

//   for (int i = 0; i < kNumMot; i++) {
//     // --- AVANTI ---
//     Serial.printf("Motor %d: FORWARD (+%d)\n", i, PWM_test);
//     robot.setMotorPWM(i, PWM_test); 
    
//     // Esegui per 3 secondi monitorando l'encoder
//     start_time = millis();
//     while(millis() - start_time < 3000) {
//       robot.update(dt); // Aggiorna i contatori degli encoder (trasferisce da delta a tick)
//       Serial.printf("MOT[%d] >> PWM: %d | Ticks: %d | Odo: %d\n", 
//                     i, PWM_test, robot.enc[i].tick, robot.enc[i].odo);
//       delay(100); // Stampa ogni 100ms
//     }

//     // --- STOP ---
//     robot.setMotorPWM(i, 0);
//     delay(500); // Breve pausa
//     Serial.printf("MOT[%d] STOPPED. Final Ticks: %d\n", i, robot.enc[i].tick);
//     delay(500);

//     // --- INDIETRO ---
//     Serial.printf("Motor %d: BACKWARD (-%d)\n", i, PWM_test);
//     robot.setMotorPWM(i, -PWM_test); 
    
//     // Esegui per 3 secondi monitorando l'encoder
//     start_time = millis();
//     while(millis() - start_time < 3000) {
//       robot.update(dt);
//       Serial.printf("MOT[%d] >> PWM: %d | Ticks: %d | Odo: %d\n", 
//                     i, -PWM_test, robot.enc[i].tick, robot.enc[i].odo);
//       delay(100);
//     }

//     // --- STOP ---
//     robot.setMotorPWM(i, 0);
//     delay(1000);
//   }

//   Serial.println("--- Sequence Complete. Restarting... ---");
//   delay(3000);
// }
#pragma endregion

#pragma region LOOP_TUNING
// // LOOP con MotorAutotuner

// void loop() {
//   ArduinoOTA.handle();
//   server.handleClient(); // Gestisce le richieste web in arrivo

//   // 1. GESTIONE TRIGGER TEST (Pulsante BOOT)
//   handleOfflineTest(); // Controlla pressione tasto (non bloccante)

//   // Gestione Countdown 5 secondi
//   if (is_boot_test_pending) {
//     if (millis() - boot_test_start_millis > 5000) {
//       is_boot_test_pending = false;
      
//       // AVVIO TEST E LOGGING
//       logFile = SPIFFS.open("/log.csv", "w");
//       if (logFile) {
//         tuner.setOutput(&logFile);
        
//         // --- CONFIGURAZIONE TEST ---
//         // Qui definisci quale test eseguire al termine del countdown.
//         // Configurazione: Preheat=0ms, Step1=2000ms, Step2=2000ms
//         tuner.start(0b1111, 1500, 0, 2000, 2000); 
//       }
//     }
//   }

//   // Se il tuner è attivo, esegui il suo ciclo di update (non-blocking)
//   if (tuner.isRunning()) {
//     // Controllo ABORT: Se premi 'x', ferma tutto immediatamente
//     // (Solo se connesso via seriale)
//     if (Serial.available() > 0) {
//       char c = Serial.peek(); // Guarda il carattere senza rimuoverlo subito
//       if (c == 'x' || c == 'X') {
//         Serial.read(); // Rimuovi dal buffer
//         tuner.stop();
//         return; // Ricomincia il loop
//       }
//     }
//     tuner.update();
//   }
//   // Se il tuner ha finito ed era aperto un file di log, chiudilo
//   else if (logFile) {
//     logFile.close();
//     tuner.setOutput(&Serial); // Ripristina output su Serial
//     Serial.println("\n--- LOG SALVATO SU SPIFFS ---");
//     Serial.println("Usa il comando 'r' per scaricare i dati.");
//   }
//   // Altrimenti attendi comandi seriali
//   else {
//     handleSerialInput();
//   }
// }


#pragma endregion

/******************************************************************************
 * FUNCTIONS IMPLEMENTATIONS
 ******************************************************************************/
void processSerialPacket(char channel, uint32_t value, channels_t& obj) {

  //if(channel == 'G') Serial.println("DEBUG: Ricevuto G!");
  
  uint8_t mot_i;
  int16_t pwm;

  // Reset watchdog
  if ((channel == 'G') || (channel == 'K')) {
    last_motor_update_millis = millis();
  }

  // Process incomming serial packet
  switch (channel) {
    // - reference angular speed
    case 'G':
    case 'H':
    case 'I':
    case 'J':
      mot_i = channel - 'G';
      // set reference angular speed for the motors
#ifdef CONFIG_LAZARUS
      robot.setMotorWref(mot_i, ((int32_t) value) * kEncImp2MotW );
#endif
#ifdef CONFIG_ROS
      robot.setMotorWref(mot_i, *((float*) &value) );
#endif
      break;
  
    // - PWM
    case 'K':
      mot_i = (value >> 24) & 0x03;
      pwm = value & 0xFFFF;
      robot.setMotorPWM(mot_i, pwm);
      break;

    // - solenoid
    case 'L':
      //digitalWrite(kRobotActSolenoidPin, value);
      break;
  }
}

void serialWrite(uint8_t b) {
  Serial.write(b);
}

void serialWriteChannel(char channel, int32_t value) {
  serial_channels.send(channel, value);
}

void serialRead() {
  uint8_t serial_byte;

  if (Serial.available() > 0) {
    serial_byte = Serial.read();
    serial_channels.StateMachine(serial_byte);
  }
}

void checkMotorsTimeout() {
  if (millis() - last_motor_update_millis > kMotCtrlTimeout) {
    timeout = true;

    robot.stop();
    //digitalWrite(kRobotActSolenoidPin, 0);

    // DEBUG
    // builtin_led_state = LOW;
    // digitalWrite(LED_BUILTIN, builtin_led_state);

  } else {
    if (timeout) {
      robot.init(serialWriteChannel);
    }

    timeout = 0;
  }
}

void setupWiFi() {
  // Utilizza le credenziali definite in robot_config.h
  Serial.printf("\nConnecting to WiFi: %s\n", kMQTTWiFiSSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(kMQTTWiFiSSID, kMQTTWiFiPass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nWiFi Connected! IP: %s\n", WiFi.localIP().toString().c_str());
}

void setupOTA() {
  ArduinoOTA.setHostname("Omnicar-ESP32");
  ArduinoOTA.begin();
}

// void setupWebServer() {
//   // Pagina principale che mostra il link per il download
//   server.on("/", HTTP_GET, []() {
//       String html = "<html><head><title>Omnicar Datalogger</title>";
//       html += "<style>body { font-family: sans-serif; text-align: center; padding-top: 50px; } a { font-size: 1.2em; }</style>";
//       html += "</head><body>";
//       html += "<h1>Omnicar Datalogger Wireless</h1>";
//       if (SPIFFS.exists("/log.csv")) {
//           File f = SPIFFS.open("/log.csv", "r");
//           html += "<p>File di log trovato (" + String(f.size()) + " bytes).</p>";
//           html += "<p><a href='/log.csv' download='log.csv'><strong>Scarica log.csv</strong></a></p>";
//           f.close();
//       } else {
//           html += "<p>Nessun file di log trovato. Esegui prima un test con il pulsante BOOT.</p>";
//       }
//       html += "</body></html>";
//       server.send(200, "text/html", html);
//   });

//   // Handler per servire il file CSV quando viene richiesto
//   server.on("/log.csv", HTTP_GET, []() {
//       if (SPIFFS.exists("/log.csv")) {
//           File file = SPIFFS.open("/log.csv", "r");
//           server.streamFile(file, "text/csv"); // Invia il file al browser
//           file.close();
//       } else {
//           server.send(404, "text/plain", "File non trovato");
//       }
//   });

//   server.begin();
//   Serial.println("Web server attivo! Accedi da: http://" + WiFi.localIP().toString());
// }

void handleOfflineTest() {
    // --- GESTIONE PULSANTE BOOT (OFFLINE MODE) ---
    // Se premi il tasto BOOT (GPIO 0) a terra, avvia il countdown
    if (digitalRead(0) == LOW) {
      // Impedisci riavvio se già in test o in countdown
      if (!is_boot_test_pending && !tuner.isRunning()) {
        delay(50); // Debounce minimo
        if (digitalRead(0) == LOW) {
          is_boot_test_pending = true;
          boot_test_start_millis = millis();
          Serial.println("BOOT Pressed: Avvio test tra 5 secondi...");
        }
      }
    }
}

void handleSerialInput() {
    if (Serial.available() > 0) {
      char c = Serial.read();
      // Invia 't' o 'T' per avviare il tuning
      if (c == 't' || c == 'T') {
        tuner.setOutput(&Serial); // Assicura output su Serial
        Serial.println("\n--- SETUP AUTOTUNER ---");
        Serial.print("Inserisci Maschera Motori (Binario, es. 0001=M0, 0101=M0+M2, 1111=All): ");
        while (!Serial.available()) {
          ArduinoOTA.handle(); // Permette OTA anche durante l'attesa
          delay(10);
        }
        String maskStr = Serial.readStringUntil('\n');
        maskStr.trim(); // Rimuove spazi e newline
        int mask = strtol(maskStr.c_str(), NULL, 2); // Converte da stringa binaria a int
        
        Serial.print("\nPWM step (es. 4095): ");
        while (!Serial.available()) {
          ArduinoOTA.handle(); // Permette OTA anche durante l'attesa
          //delay(10);
        }
        int pwm = Serial.parseInt();

        // Pulisci buffer
        while (Serial.available()) Serial.read();
        
        // Avvia tuning con maschera diretta
        tuner.start((uint8_t)mask, pwm, 0, 2000, 2000);
      }
      
      // --- COMANDO 'r': Leggi log salvato ---
      // else if (c == 'r' || c == 'R') {
      //   Serial.println("\n--- LETTURA LOG DA SPIFFS ---");
      //   if (SPIFFS.exists("/log.csv")) {
      //       File f = SPIFFS.open("/log.csv", FILE_READ);
      //       while (f.available()) Serial.write(f.read());
      //       f.close();
      //       Serial.println("\n--- FINE FILE ---");
      //   } else {
      //       Serial.println("Nessun file di log trovato.");
      //   }
      // }
    }
}