#ifndef RADIO_H
#define RADIO_H
#include "types.h"

void radio_init();
void radio_update(DroneState *drone);
int convert_receiver_channel(byte function);

#endif