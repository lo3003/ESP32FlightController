#include <Arduino.h>
#include <EEPROM.h>
#include "setup_wizard.h"
#include "config.h"

// Variables temporaires pour le setup
extern volatile int raw_channel_1, raw_channel_2, raw_channel_3, raw_channel_4;
int center_1, center_2, center_3, center_4;
int min_1, min_2, min_3, min_4;
int max_1, max_2, max_3, max_4;
byte assign_1, assign_2, assign_3, assign_4;

void wait_zero() {
    // Attend que les sticks soient centrés (~1500)
    bool ok = false;
    while(!ok) {
        if(raw_channel_1 > 1480 && raw_channel_1 < 1520 &&
           raw_channel_2 > 1480 && raw_channel_2 < 1520 &&
           raw_channel_3 > 1480 && raw_channel_3 < 1520 &&
           raw_channel_4 > 1480 && raw_channel_4 < 1520) ok = true;
        delay(100);
        Serial.print(".");
    }
    Serial.println("OK");
}

void run_setup_wizard() {
    Serial.println(F("\n\n--- SETUP WIZARD ESP32 ---"));
    Serial.println(F("Placez les sticks au CENTRE."));
    wait_zero();
    
    // 1. Capture Centres
    Serial.println(F("Enregistrement Centres..."));
    center_1 = raw_channel_1;
    center_2 = raw_channel_2;
    center_3 = raw_channel_3;
    center_4 = raw_channel_4;
    
    // 2. Détection Inversions (Simplifié pour ESP32 - On suppose l'ordre des pins correct 1,2,3,4)
    // On garde l'assignation par défaut 1, 2, 3, 4 mais on détecte si inversé
    Serial.println(F("Montez les GAZ (CH3) puis centrez."));
    while(raw_channel_3 < 1800 && raw_channel_3 > 1200) delay(10);
    assign_3 = 3; if(raw_channel_3 < 1500) assign_3 |= 0x80;
    wait_zero();
    
    Serial.println(F("Mettez ROLL a DROITE (CH1) puis centrez."));
    while(raw_channel_1 < 1800 && raw_channel_1 > 1200) delay(10);
    assign_1 = 1; if(raw_channel_1 < 1500) assign_1 |= 0x80;
    wait_zero();

    Serial.println(F("Mettez PITCH vers HAUT (CH2) puis centrez."));
    while(raw_channel_2 < 1800 && raw_channel_2 > 1200) delay(10);
    assign_2 = 2; if(raw_channel_2 < 1500) assign_2 |= 0x80;
    wait_zero();

    Serial.println(F("Mettez YAW a DROITE (CH4) puis centrez."));
    while(raw_channel_4 < 1800 && raw_channel_4 > 1200) delay(10);
    assign_4 = 4; if(raw_channel_4 < 1500) assign_4 |= 0x80;
    wait_zero();

    // 3. Calibration Limites
    Serial.println(F("Bougez TOUS les sticks dans les coins (10 sec)..."));
    min_1 = center_1; max_1 = center_1;
    min_2 = center_2; max_2 = center_2;
    min_3 = center_3; max_3 = center_3;
    min_4 = center_4; max_4 = center_4;
    
    long timer = millis() + 10000;
    while(millis() < timer) {
        if(raw_channel_1 < min_1) min_1 = raw_channel_1; if(raw_channel_1 > max_1) max_1 = raw_channel_1;
        if(raw_channel_2 < min_2) min_2 = raw_channel_2; if(raw_channel_2 > max_2) max_2 = raw_channel_2;
        if(raw_channel_3 < min_3) min_3 = raw_channel_3; if(raw_channel_3 > max_3) max_3 = raw_channel_3;
        if(raw_channel_4 < min_4) min_4 = raw_channel_4; if(raw_channel_4 > max_4) max_4 = raw_channel_4;
        delay(20);
    }
    
    // 4. Ecriture EEPROM
    Serial.println(F("Sauvegarde..."));
    // Centres
    EEPROM.write(0, center_1 & 0xFF); EEPROM.write(1, center_1 >> 8);
    EEPROM.write(2, center_2 & 0xFF); EEPROM.write(3, center_2 >> 8);
    EEPROM.write(4, center_3 & 0xFF); EEPROM.write(5, center_3 >> 8);
    EEPROM.write(6, center_4 & 0xFF); EEPROM.write(7, center_4 >> 8);
    // Max
    EEPROM.write(8, max_1 & 0xFF); EEPROM.write(9, max_1 >> 8);
    EEPROM.write(10, max_2 & 0xFF); EEPROM.write(11, max_2 >> 8);
    EEPROM.write(12, max_3 & 0xFF); EEPROM.write(13, max_3 >> 8);
    EEPROM.write(14, max_4 & 0xFF); EEPROM.write(15, max_4 >> 8);
    // Min
    EEPROM.write(16, min_1 & 0xFF); EEPROM.write(17, min_1 >> 8);
    EEPROM.write(18, min_2 & 0xFF); EEPROM.write(19, min_2 >> 8);
    EEPROM.write(20, min_3 & 0xFF); EEPROM.write(21, min_3 >> 8);
    EEPROM.write(22, min_4 & 0xFF); EEPROM.write(23, min_4 >> 8);
    // Assign
    EEPROM.write(24, assign_1);
    EEPROM.write(25, assign_2);
    EEPROM.write(26, assign_3);
    EEPROM.write(27, assign_4);
    // Gyro (Standard MPU6050)
    EEPROM.write(28, 1); // Roll Axis
    EEPROM.write(29, 2); // Pitch Axis
    EEPROM.write(30, 3); // Yaw Axis
    EEPROM.write(31, 1); // Type
    EEPROM.write(32, GYRO_ADDRESS); 
    // Signature
    EEPROM.write(33, 'J'); EEPROM.write(34, 'M'); EEPROM.write(35, 'B');
    
    EEPROM.commit();
    Serial.println(F("Terminé !"));
}