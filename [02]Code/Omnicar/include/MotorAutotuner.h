#ifndef MOTOR_AUTOTUNER_H
#define MOTOR_AUTOTUNER_H

#include <Arduino.h>
#include "Robot.h"
#include "robot_config.h"

class MotorAutotuner {
public:
    enum State {
        INIT,
        PREHEAT,
        PAUSE,
        STEP1,
        STEP2,
        FINISHED
    };

    MotorAutotuner(Robot* robotRef) : robot(robotRef), state(INIT), activeMotors(0), out(&Serial) {}

    void setOutput(Print* outputStream) {
        out = outputStream;
    }

    // Avvia la sequenza di tuning
    // MODIFICATO: Accetta maschera motori e durate configurabili
    void start(uint8_t motorMask, int pwmValue, unsigned long preheatMs, unsigned long step1Ms, unsigned long step2Ms) {
        activeMotors = motorMask;
        targetPWM = pwmValue;
        durPreheat = preheatMs;
        durStep1 = step1Ms;
        durStep2 = step2Ms;

        // Se preheat è 0, salta direttamente alla pausa (o step1)
        state = (durPreheat > 0) ? PREHEAT : PAUSE;
        stateStartTime = millis();
        lastUpdateMicros = micros();
        
        out->printf("--- START Tuning. Mask: 0x%02X, PWM: %d ---\n", activeMotors, targetPWM);
        out->printf("Timings: Preheat=%lu, Step1=%lu, Step2=%lu\n", durPreheat, durStep1, durStep2);
        
        if (state == PREHEAT) {
            out->println("State: PREHEAT");
            setMotorsPWM(targetPWM);
        } else {
            out->println("State: PAUSE (Skipped PREHEAT)");
            setMotorsPWM(0);
        }
    }

    // Ferma forzatamente il tuning e resetta lo stato
    void stop() {
        setMotorsPWM(0);
        state = INIT;
        out->println("\n--- TUNING ABORTED BY USER ---");
    }

    // Metodo non-blocking da chiamare nel loop
    void update() {
        unsigned long currentMillis = millis();
        unsigned long currentMicros = micros();
        
        switch (state) {
            case INIT:
                // In attesa
                break;

            case PREHEAT:
                if (currentMillis - stateStartTime > durPreheat) {
                    state = PAUSE;
                    stateStartTime = currentMillis;
                    
                    // Ferma il motore per la pausa
                    setMotorsPWM(0);
                    out->println("PREHEAT Finito. Inizio PAUSA (1s)...");
                }
                break;

            case PAUSE:
                if (currentMillis - stateStartTime > 100) {
                    state = STEP1;
                    stateStartTime = currentMillis; // Reset tempo a 0 per il grafico
                    lastUpdateMicros = currentMicros;
                    
                    // --- FIX: RESET REALE DEGLI ENCODER ---
                    uint32_t dummy_dt = 0;
                    robot->update(dummy_dt);
                    
                    // Applica PWM per STEP 1 (una volta sola)
                    setMotorsPWM(targetPWM / 2);

                    out->println("PAUSA Finita.");
                    out->println("State: TEST START (Step 1: 50% -> Step 2: 100%)");
                    // Intestazione CSV per 4 motori
                    out->println("Time_ms;PWM_Ref;Spd_M0;Spd_M1;Spd_M2;Spd_M3"); 
                    // Riga iniziale a 0
                    out->println("0;0;0.0000;0.0000;0.0000;0.0000");
                }
                break;

            case STEP1:
                // Log dati a 200Hz
                runControlLoop(currentMillis, currentMicros, targetPWM / 2);

                // Dopo 2 secondi passa allo Step 2
                if (currentMillis - stateStartTime > durStep1) {
                    state = STEP2;
                    // NON resettiamo stateStartTime, così il tempo prosegue lineare nel CSV
                    
                    // Applica PWM per STEP 2 (una volta sola)
                    setMotorsPWM(targetPWM);
                }
                break;

            case STEP2:
                // Log dati a 200Hz
                runControlLoop(currentMillis, currentMicros, targetPWM);

                // Fine dopo Step1 + Step2
                if (currentMillis - stateStartTime > (durStep1 + durStep2)) {
                    state = FINISHED;
                    setMotorsPWM(0);
                    out->println("State: FINISHED");
                }
                break;

            case FINISHED:
                setMotorsPWM(0);
                state = INIT; // Reset automatico
                break;
        }
    }

    bool isRunning() { return state != INIT; }

private:
    // Helper per settare PWM su tutti i motori attivi nella maschera
    void setMotorsPWM(int pwm) {
        for (int i = 0; i < kNumMot; i++) {
            if ((activeMotors >> i) & 1) {
                robot->setMotorPWM(i, pwm);
            }
        }
    }

    // Helper per leggere encoder e stampare seriale alla frequenza corretta
    void runControlLoop(unsigned long currentMillis, unsigned long currentMicros, int currentPWM) {
        // Usa kMotCtrlTimeUs (5000us = 200Hz) definito in robot_config.h
        if (currentMicros - lastUpdateMicros >= kMotCtrlTimeUs) { //kMotCtrlTimeUs= 0.005s = 200Hz
            uint32_t dt_us = currentMicros - lastUpdateMicros;
            float dt_sec = dt_us / 1000000.0f;

            robot->update(dt_us); 

            // Log: Tempo relativo all'inizio del TEST (Step 1)
            out->printf("%lu;%d", currentMillis - stateStartTime, currentPWM);

            // Logga sempre tutti e 4 i motori per coerenza CSV
            for (int i = 0; i < kNumMot; i++) {
                float speed = (float)robot->enc[i].odo * kEncImp2MotW;
                out->printf(";%.4f", speed);
            }
            out->println();

            lastUpdateMicros = currentMicros;
        }
    }

    Robot* robot;
    State state;
    uint8_t activeMotors; // Bitmask
    int targetPWM;
    unsigned long durPreheat, durStep1, durStep2;
    unsigned long stateStartTime;
    unsigned long lastUpdateMicros;
    Print* out;
};

#endif
