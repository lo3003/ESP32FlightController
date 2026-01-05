#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "config.h"
#include "types.h"
#include "radio.h"
#include "imu.h"
#include "pid.h"
#include "motors.h"
#include "esc_calibrate.h"
#include "telemetry.h"
#include "setup_wizard.h" 

DroneState drone;
unsigned long loop_timer;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(I2C_SPEED);
    EEPROM.begin(512); 
    
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    motors_init();
    radio_init();

    // Démarrage Télémétrie (Wi-Fi)
    start_telemetry_task(&drone);

    // Initialisation
    Serial.println(F("Démarrage..."));
    
    // Attente brève pour le S.BUS
    unsigned long wait_radio = millis();
    while(drone.channel_3 < 900 && millis() - wait_radio < 2000) {
        radio_update(&drone);
        delay(10);
    }

    if(drone.channel_3 > 1900) {
        Serial.println(F("MODE CALIBRATION"));
        drone.current_mode = MODE_CALIBRATION;
        esc_calibrate_init();
    } else {
        Serial.println(F("MODE VOL"));
        drone.current_mode = MODE_SAFE;
        imu_init();
        pid_init();
    }
    
    loop_timer = micros();
}

void loop() {
    // 1. Lecture Radio & Setup
    radio_update(&drone);
    setup_loop_monitor();

    // 2. Logique de vol
    if(drone.current_mode == MODE_CALIBRATION) {
        esc_calibrate_loop(&drone);
    } 
    else {
        imu_read(&drone);

        switch(drone.current_mode) {
            case MODE_SAFE:
                motors_stop();
                digitalWrite(PIN_LED, HIGH);
                if(drone.channel_3 < 1050 && drone.channel_4 < 1050) drone.current_mode = MODE_PRE_ARM;
                break;

            case MODE_PRE_ARM:
                motors_stop();
                if(drone.channel_3 < 1050 && drone.channel_4 > 1450) {
                    drone.current_mode = MODE_ARMED;
                    pid_reset_integral();
                    drone.angle_pitch = 0;
                    drone.angle_roll = 0;
                    // Reset du timer de boucle pour éviter un saut brutal
                    loop_timer = micros(); 
                }
                break;

            case MODE_ARMED:
                motors_stop();
                digitalWrite(PIN_LED, LOW);
                if(drone.channel_3 > 1050) drone.current_mode = MODE_FLYING;
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

    // --- MODIFICATION CRITIQUE POUR STABILITÉ WIFI ---
    // Au lieu de bloquer le processeur à 100% avec un while(),
    // on regarde combien de temps il reste avant le prochain cycle.
    // Si on a plus de 2ms d'avance, on fait un delay(1) pour laisser le Wi-Fi travailler.
    
    unsigned long time_used = micros() - loop_timer;
    
    if (time_used < LOOP_TIME_US) {
        unsigned long time_remaining = LOOP_TIME_US - time_used;
        
        // Si on a le temps, on donne la main au système (Wi-Fi)
        if (time_remaining > 2000) { // S'il reste plus de 2ms
             delay(1); // Pause système de ~1ms (Laisse le Core respirer)
        }
        
        // On finit le reste du temps avec une boucle précise
        while(micros() - loop_timer < LOOP_TIME_US);
    }
    
    loop_timer = micros();
}