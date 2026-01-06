#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "types.h"
#include "radio.h"
#include "imu.h"
#include "pid.h"
#include "motors.h"
#include "esc_calibrate.h"

// On supprime les includes inutiles (Telemetry, Setup Wizard, EEPROM)

DroneState drone;
unsigned long loop_timer;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(I2C_SPEED);
    
    // Plus besoin d'EEPROM.begin()

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    motors_init();
    radio_init();

    // Plus de start_telemetry_task() -> Le Wifi reste éteint !

    Serial.println(F("Attente signal radio..."));
    unsigned long wait_radio = millis();
    while(drone.channel_3 < 900 && millis() - wait_radio < 2000) {
        radio_update(&drone);
        delay(10);
    }

    if(drone.channel_3 > 1900) {
        Serial.println(F("MODE CALIBRATION ESC"));
        drone.current_mode = MODE_CALIBRATION;
        esc_calibrate_init();
    } else {
        Serial.println(F("MODE VOL (SAFE)"));
        drone.current_mode = MODE_SAFE;
        imu_init(); 
        pid_init();
    }
    
    loop_timer = micros();
}

void loop() {
    radio_update(&drone);
    // Plus de setup_loop_monitor()

    if(drone.current_mode == MODE_CALIBRATION) {
        esc_calibrate_loop(&drone);
    } 
    else {
        imu_read(&drone);

        switch(drone.current_mode) {
            case MODE_SAFE:
                motors_stop();
                digitalWrite(PIN_LED, HIGH);
                // Armement (Gaz bas + Yaw Gauche)
                if(drone.channel_3 < 1050 && drone.channel_4 < 1050) drone.current_mode = MODE_PRE_ARM;
                break;

            case MODE_PRE_ARM:
                motors_stop();
                // Validation (Gaz bas + Yaw Centre)
                if(drone.channel_3 < 1050 && drone.channel_4 > 1450) {
                    drone.current_mode = MODE_ARMED;
                    pid_reset_integral();
                    drone.angle_pitch = 0;
                    drone.angle_roll = 0;
                    loop_timer = micros();
                }
                break;

            case MODE_ARMED:
                motors_stop();
                digitalWrite(PIN_LED, LOW);
                // Décollage
                if(drone.channel_3 > 1050) drone.current_mode = MODE_FLYING;
                // Désarmement (Gaz bas + Yaw Droite)
                if(drone.channel_3 < 1050 && drone.channel_4 > 1900) drone.current_mode = MODE_SAFE;
                break;

            case MODE_FLYING:
                pid_compute_setpoints(&drone);
                pid_compute(&drone);
                motors_mix(&drone);
                if(drone.channel_3 < 1050) drone.current_mode = MODE_ARMED;
                break;
        }
        motors_write();
    }

    // --- WATCHDOG KEEPER ---
    // On garde cette sécurité vitale pour éviter le reboot de 10s
    if (micros() - loop_timer < LOOP_TIME_US - 1000) {
        delay(1); 
    }
    while(micros() - loop_timer < LOOP_TIME_US);
    loop_timer = micros();
}