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

// --- NOUVEAUX INCLUDES POUR LE WEB ---
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
    digitalWrite(PIN_LED, HIGH); // LED ON = Démarrage

    // 1. Init Hardware
    motors_init();
    radio_init();

    // 2. Démarrage du Serveur Web (Sur le Coeur 0)
    // Cela rend le setup accessible via Wi-Fi immédiatement
    start_telemetry_task(&drone);

    // 3. Détection Calibration ESC (Gaz à fond)
    Serial.println(F("Attente signal radio..."));
    
    // On attend un peu pour être sûr de recevoir des trames S.BUS
    unsigned long wait_radio = millis();
    while(drone.channel_3 < 900 && millis() - wait_radio < 2000) {
        radio_update(&drone);
        delay(10);
    }

    // Si gaz > 1900us au démarrage -> Mode Calibration ESC
    if(drone.channel_3 > 1900) {
        Serial.println(F("!!! MODE CALIBRATION ESC DETECTE !!!"));
        drone.current_mode = MODE_CALIBRATION;
        esc_calibrate_init();
    } else {
        Serial.println(F("Démarrage Mode Vol (Safe)..."));
        drone.current_mode = MODE_SAFE;
        imu_init(); // Calibration du Gyroscope (Ne pas bouger le drone)
        pid_init();
    }
    
    loop_timer = micros();
}

void loop() {
    // 1. Lecture Radio
    // Met à jour les variables globales (raw_channel_x) et la structure drone
    radio_update(&drone);

    // 2. Monitoring Setup Web 
    // Si la page web demande un scan Min/Max, cette fonction capture les valeurs
    setup_loop_monitor();

    // 3. Gestion selon le Mode de Vol
    if(drone.current_mode == MODE_CALIBRATION) {
        // Logique dédiée Calibration ESC (Passe-through gaz)
        esc_calibrate_loop(&drone);
    } 
    else {
        // --- LOGIQUE DE VOL NORMALE ---
        
        // Lecture IMU (Gyro + Accel)
        imu_read(&drone);

        switch(drone.current_mode) {
            case MODE_SAFE:
                motors_stop();
                digitalWrite(PIN_LED, HIGH); // LED Fixe = Sécurité
                
                // Armement : Gaz en bas (<1050) + Yaw à Gauche (<1050)
                if(drone.channel_3 < 1050 && drone.channel_4 < 1050) {
                    drone.current_mode = MODE_PRE_ARM;
                }
                break;

            case MODE_PRE_ARM:
                motors_stop();
                // Attente retour centre du Yaw pour valider l'armement
                if(drone.channel_3 < 1050 && drone.channel_4 > 1450) {
                    drone.current_mode = MODE_ARMED;
                    pid_reset_integral();
                    drone.angle_pitch = 0;
                    drone.angle_roll = 0;
                }
                break;

            case MODE_ARMED:
                motors_stop(); // Moteurs coupés (ou ralenti si vous préférez)
                digitalWrite(PIN_LED, LOW); // LED Eteinte = Armé
                
                // Décollage : Si on monte les gaz
                if(drone.channel_3 > 1050) drone.current_mode = MODE_FLYING;
                
                // Désarmement : Gaz en bas + Yaw à Droite (>1900)
                if(drone.channel_3 < 1050 && drone.channel_4 > 1900) {
                    drone.current_mode = MODE_SAFE;
                }
                break;

            case MODE_FLYING:
                // Calculs de Vol (PID + Mixage)
                pid_compute_setpoints(&drone);
                pid_compute(&drone);
                motors_mix(&drone);
                
                // Si on baisse les gaz à fond, on repasse en mode Armé (Moteurs coupés/ralenti)
                if(drone.channel_3 < 1050) drone.current_mode = MODE_ARMED;
                break;
        }
        
        // Application de la commande aux moteurs (si on n'est pas en Safe/Calib)
        // Note: motors_stop() force déjà les valeurs à 1000, donc motors_write() applique bien l'arrêt.
        motors_write();
    }

    // 4. Gestion de la boucle 250Hz (4000us)
    while(micros() - loop_timer < LOOP_TIME_US);
    loop_timer = micros();
}