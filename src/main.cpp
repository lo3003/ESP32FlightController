#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "config.h"
#include "types.h"
#include "radio.h"
#include "imu.h"
#include "pid.h"
#include "motors.h"
#include "setup_wizard.h"
#include "esc_calibrate.h"

DroneState drone;
unsigned long loop_timer;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(I2C_SPEED);
    EEPROM.begin(512); // Indispensable sur ESP32
    
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH); // LED ON = Booting

    // Init Hardware
    motors_init();
    radio_init();

    // --- PHASE 1 : ECOUTE POUR SETUP (5 sec) ---
    Serial.println(F(">>> ATTENTE COMMANDE 's' (5s) POUR SETUP <<<"));
    unsigned long wait_timer = millis();
    while(millis() - wait_timer < 5000) {
        if(Serial.available() > 0) {
            if(Serial.read() == 's') {
                drone.current_mode = MODE_SETUP;
                run_setup_wizard(); // Bloquant
                ESP.restart();
            }
        }
        // Faire clignoter la LED rapidement
        if((millis() / 100) % 2) digitalWrite(PIN_LED, LOW); else digitalWrite(PIN_LED, HIGH);
        delay(10);
    }
    
    // --- PHASE 2 : DETECTION CALIBRATION ESC ---
    // On lit la radio pour voir si les gaz sont à fond
    // On attend un signal valide d'abord
    Serial.println(F("Attente signal radio..."));
    while(drone.channel_3 < 900) {
        radio_update(&drone);
        delay(10);
    }

    if(drone.channel_3 > 1900) {
        // GAZ A FOND -> MODE CALIBRATION
        Serial.println(F("!!! MODE CALIBRATION ESC DETECTE !!!"));
        drone.current_mode = MODE_CALIBRATION;
        esc_calibrate_init(); // Prépare la calib
    } else {
        // DEMARRAGE NORMAL
        Serial.println(F("Démarrage Mode Vol..."));
        drone.current_mode = MODE_SAFE;
        imu_init(); // Calibre le gyro (ne pas bouger)
        pid_init();
    }
    
    loop_timer = micros();
}

void loop() {
    // 1. Lecture Radio
    radio_update(&drone);

    // 2. Gestion selon le Mode
    if(drone.current_mode == MODE_CALIBRATION) {
        // Logique dédiée Calibration (fichier esc_calibrate.cpp)
        esc_calibrate_loop(&drone);
    } 
    else {
        // Logique de Vol Normale
        imu_read(&drone);

        switch(drone.current_mode) {
            case MODE_SAFE:
                motors_stop();
                digitalWrite(PIN_LED, HIGH); // LED Fixe = Sécurité
                // Armement : Gaz bas + Yaw (ou switch, ici Yaw Gauche/Droite selon tes prefs)
                // YMFC standard : Gaz < 1050 et Yaw < 1050 (Gauche) -> Armement
                if(drone.channel_3 < 1050 && drone.channel_4 < 1050) drone.current_mode = MODE_PRE_ARM;
                break;

            case MODE_PRE_ARM:
                motors_stop();
                // Attente retour centre Yaw
                if(drone.channel_3 < 1050 && drone.channel_4 > 1450) {
                    drone.current_mode = MODE_ARMED;
                    pid_reset_integral();
                    drone.angle_pitch = drone.angle_roll = 0;
                }
                break;

            case MODE_ARMED:
                motors_stop(); // Ou ralenti moteur si désiré
                digitalWrite(PIN_LED, LOW); // LED Eteinte = Armé
                
                if(drone.channel_3 > 1050) drone.current_mode = MODE_FLYING;
                
                // Désarmement : Gaz bas + Yaw Droite
                if(drone.channel_3 < 1050 && drone.channel_4 > 1900) drone.current_mode = MODE_SAFE;
                break;

            case MODE_FLYING:
                pid_compute_setpoints(&drone);
                pid_compute(&drone);
                motors_mix(&drone);
                
                if(drone.channel_3 < 1050) drone.current_mode = MODE_ARMED;
                break;
        }
        
        // Ecriture Moteurs (Mode Vol)
        motors_write();
    }

    // Gestion de la boucle 250Hz (4000us)
    // Important pour la stabilité PID et l'intégration Gyro
    while(micros() - loop_timer < LOOP_TIME_US);
    loop_timer = micros();
}