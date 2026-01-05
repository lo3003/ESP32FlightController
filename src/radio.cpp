#include <Arduino.h>
#include <EEPROM.h>
#include "radio.h"
#include "config.h"
#include "sbus.h" // Librairie Bolder Flight

// Variables Globales (utilisées par main et setup_wizard)
int raw_channel_1 = 1500, raw_channel_2 = 1500, raw_channel_3 = 1000, raw_channel_4 = 1500;
byte eeprom_data[36];

// Création objet S.BUS : Serial2, RX=PIN_SBUS_RX, TX=-1, Inverted=true
bfs::SbusRx sbus(&Serial2, PIN_SBUS_RX, -1, true);
bfs::SbusData data;

void radio_init() {
    // Démarrage du S.BUS
    sbus.Begin();
    
    // Charge la config EEPROM (Calibration)
    for(int i = 0; i < 36; i++) eeprom_data[i] = EEPROM.read(i);
}

// Fonction spéciale pour lire le S.BUS à la demande
// (Indispensable pour le setup_wizard qui tourne en boucle)
void radio_read_raw() {
    if (sbus.Read()) {
        data = sbus.data();
        
        // Mapping S.BUS (172-1811) vers PWM Standard (1000-2000)
        // Note: L'ordre des canaux [0]..[3] dépend de ta radio (AETR ou TAER)
        // Ici je suppose AETR (Standard FrSky/Radiolink) : 0=Roll, 1=Pitch, 2=Thr, 3=Yaw
        // Si c'est inversé, change les indices [x]
        raw_channel_1 = map(data.ch[0], 172, 1811, 1000, 2000); // Roll
        raw_channel_2 = map(data.ch[1], 172, 1811, 1000, 2000); // Pitch
        raw_channel_3 = map(data.ch[2], 172, 1811, 1000, 2000); // Throttle
        raw_channel_4 = map(data.ch[3], 172, 1811, 1000, 2000); // Yaw
    }
}

void radio_update(DroneState *drone) {
    // 1. Lire les nouvelles données
    radio_read_raw();

    // 2. Appliquer la calibration (Centre/Inversion) via l'EEPROM
    drone->channel_1 = convert_receiver_channel(1);
    drone->channel_2 = convert_receiver_channel(2);
    drone->channel_3 = convert_receiver_channel(3);
    drone->channel_4 = convert_receiver_channel(4);
}

// Fonction YMFC originale (Inchangée, elle fait le job de calibration)
int convert_receiver_channel(byte function) {
    byte channel, reverse;
    int low, center, high, actual, difference;

    channel = eeprom_data[function + 23] & 0b00000111;
    if(eeprom_data[function + 23] & 0b10000000) reverse = 1; else reverse = 0;

    int inputs[5];
    inputs[1] = raw_channel_1;
    inputs[2] = raw_channel_2;
    inputs[3] = raw_channel_3;
    inputs[4] = raw_channel_4;
    
    actual = inputs[channel];
    low = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14];
    center = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2];
    high = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];

    if(actual < center){
        if(actual < low) actual = low;
        difference = ((long)(center - actual) * (long)500) / (center - low);
        if(reverse == 1) return 1500 + difference; else return 1500 - difference;
    } else if(actual > center){
        if(actual > high) actual = high;
        difference = ((long)(actual - center) * (long)500) / (high - center);
        if(reverse == 1) return 1500 - difference; else return 1500 + difference;
    } else return 1500;
}