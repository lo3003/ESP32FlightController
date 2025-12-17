#include <Arduino.h>
#include <EEPROM.h>
#include "radio.h"
#include "config.h"

// Variables Interruptions
volatile unsigned long timer_1, timer_2, timer_3, timer_4;
volatile int raw_channel_1, raw_channel_2, raw_channel_3, raw_channel_4;
byte eeprom_data[36];

// Interrupt Service Routines (Une par pin pour ESP32)
void IRAM_ATTR isr_ch1() {
    unsigned long now = micros();
    if(digitalRead(PIN_RADIO_1)) timer_1 = now;
    else raw_channel_1 = now - timer_1;
}
void IRAM_ATTR isr_ch2() {
    unsigned long now = micros();
    if(digitalRead(PIN_RADIO_2)) timer_2 = now;
    else raw_channel_2 = now - timer_2;
}
void IRAM_ATTR isr_ch3() {
    unsigned long now = micros();
    if(digitalRead(PIN_RADIO_3)) timer_3 = now;
    else raw_channel_3 = now - timer_3;
}
void IRAM_ATTR isr_ch4() {
    unsigned long now = micros();
    if(digitalRead(PIN_RADIO_4)) timer_4 = now;
    else raw_channel_4 = now - timer_4;
}

void radio_init() {
    pinMode(PIN_RADIO_1, INPUT); attachInterrupt(PIN_RADIO_1, isr_ch1, CHANGE);
    pinMode(PIN_RADIO_2, INPUT); attachInterrupt(PIN_RADIO_2, isr_ch2, CHANGE);
    pinMode(PIN_RADIO_3, INPUT); attachInterrupt(PIN_RADIO_3, isr_ch3, CHANGE);
    pinMode(PIN_RADIO_4, INPUT); attachInterrupt(PIN_RADIO_4, isr_ch4, CHANGE);

    // Charge la config EEPROM
    for(int i = 0; i < 36; i++) eeprom_data[i] = EEPROM.read(i);
}

void radio_update(DroneState *drone) {
    drone->channel_1 = convert_receiver_channel(1);
    drone->channel_2 = convert_receiver_channel(2);
    drone->channel_3 = convert_receiver_channel(3);
    drone->channel_4 = convert_receiver_channel(4);
}

// Fonction YMFC originale adaptée
int convert_receiver_channel(byte function) {
    byte channel, reverse;
    int low, center, high, actual, difference;

    channel = eeprom_data[function + 23] & 0b00000111;
    if(eeprom_data[function + 23] & 0b10000000) reverse = 1; else reverse = 0;

    // Mapping ESP32 variables -> Channel index
    // Note: Dans le setup YMFC, l'ordre est déterminé. 
    // Ici on map simplement nos lectures interrupt vers un tableau virtuel pour respecter la logique
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