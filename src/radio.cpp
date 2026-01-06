#include <Arduino.h>
#include "radio.h"
#include "config.h"
#include "sbus.h" 

// Variables Globales
int raw_channel_1 = 1500, raw_channel_2 = 1500, raw_channel_3 = 1000, raw_channel_4 = 1500;

// Création objet S.BUS
bfs::SbusRx sbus(&Serial2, PIN_SBUS_RX, -1, true);
bfs::SbusData data;

void radio_init() {
    sbus.Begin();
}

void radio_read_raw() {
    if (sbus.Read()) {
        data = sbus.data();
        
        // --- CALIBRATION SUR MESURE (Mise à jour) ---
        // Bornes S.BUS physiques : env. 310 (Min) et 1690 (Max)
        
        // ROLL : OK (Gauche=Min, Droite=Max)
        raw_channel_1 = map(data.ch[0], 310, 1690, 1000, 2000); 
        
        // PITCH : INVERSÉ (Haut=Max 1920 -> On veut 1000 pour piquer)
        // On garde l'inversion ici
        raw_channel_2 = map(data.ch[1], 1690, 310, 1000, 2000); 
        
        // THROTTLE : NORMAL (Bas=Min 1080 -> On veut 1000)
        // --- CORRECTION ICI : Remis dans le sens normal (310 -> 1690) ---
        raw_channel_3 = map(data.ch[2], 310, 1690, 1000, 2000); 
        
        // YAW : OK (Gauche=Min, Droite=Max)
        raw_channel_4 = map(data.ch[3], 310, 1690, 1000, 2000);
    }
}

void radio_update(DroneState *drone) {
    radio_read_raw();

    drone->channel_1 = raw_channel_1;
    drone->channel_2 = raw_channel_2;
    drone->channel_3 = raw_channel_3;
    drone->channel_4 = raw_channel_4;
}

int convert_receiver_channel(byte function) {
    return 1500;
}