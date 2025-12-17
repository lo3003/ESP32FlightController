#include <Arduino.h>
#include "motors.h"
#include "config.h"

int esc_1, esc_2, esc_3, esc_4;

void motors_init() {
    pinMode(PIN_MOTOR_1, OUTPUT);
    pinMode(PIN_MOTOR_2, OUTPUT);
    pinMode(PIN_MOTOR_3, OUTPUT);
    pinMode(PIN_MOTOR_4, OUTPUT);
    motors_stop();
}

void motors_stop() {
    esc_1 = MOTOR_OFF; 
    esc_2 = MOTOR_OFF; 
    esc_3 = MOTOR_OFF; 
    esc_4 = MOTOR_OFF;
}

void motors_write_direct(int m1, int m2, int m3, int m4) {
    esc_1 = m1; esc_2 = m2; esc_3 = m3; esc_4 = m4;
    motors_write();
}

void motors_mix(DroneState *drone) {
    int throttle = drone->channel_3;
    if (throttle > MAX_THROTTLE) throttle = MAX_THROTTLE;

    // Calcul du mixage (Throttle + PID)
    int esc_1_calc = throttle - drone->pid_output_pitch + drone->pid_output_roll - drone->pid_output_yaw; 
    int esc_2_calc = throttle + drone->pid_output_pitch + drone->pid_output_roll + drone->pid_output_yaw; 
    int esc_3_calc = throttle + drone->pid_output_pitch - drone->pid_output_roll - drone->pid_output_yaw; 
    int esc_4_calc = throttle - drone->pid_output_pitch - drone->pid_output_roll + drone->pid_output_yaw; 

    // Gestion de la saturation (si on dépasse le max)
    int max_val = esc_1_calc;
    if(esc_2_calc > max_val) max_val = esc_2_calc;
    if(esc_3_calc > max_val) max_val = esc_3_calc;
    if(esc_4_calc > max_val) max_val = esc_4_calc;

    if(max_val > MAX_THROTTLE) {
        int diff = max_val - MAX_THROTTLE;
        esc_1_calc -= diff; esc_2_calc -= diff;
        esc_3_calc -= diff; esc_4_calc -= diff;
    }

    // Assignation et limites
    esc_1 = constrain(esc_1_calc, MIN_THROTTLE_IDLE, MAX_THROTTLE);
    esc_2 = constrain(esc_2_calc, MIN_THROTTLE_IDLE, MAX_THROTTLE);
    esc_3 = constrain(esc_3_calc, MIN_THROTTLE_IDLE, MAX_THROTTLE);
    esc_4 = constrain(esc_4_calc, MIN_THROTTLE_IDLE, MAX_THROTTLE);
}

// Génération PWM Manuelle (Style YMFC adapté ESP32)
// On utilise digitalWrite qui est rapide sur ESP32
void motors_write() {
    long loop_timer_pwm = micros();
    
    // On met tout à HIGH
    digitalWrite(PIN_MOTOR_1, HIGH);
    digitalWrite(PIN_MOTOR_2, HIGH);
    digitalWrite(PIN_MOTOR_3, HIGH);
    digitalWrite(PIN_MOTOR_4, HIGH);

    long timer_1 = esc_1 + loop_timer_pwm;
    long timer_2 = esc_2 + loop_timer_pwm;
    long timer_3 = esc_3 + loop_timer_pwm;
    long timer_4 = esc_4 + loop_timer_pwm;
    
    // On attend le moment de mettre à LOW
    // Note: C'est bloquant, comme sur l'Arduino Original.
    while(digitalRead(PIN_MOTOR_1) || digitalRead(PIN_MOTOR_2) || digitalRead(PIN_MOTOR_3) || digitalRead(PIN_MOTOR_4)) {
        long now = micros();
        if(timer_1 <= now) digitalWrite(PIN_MOTOR_1, LOW);
        if(timer_2 <= now) digitalWrite(PIN_MOTOR_2, LOW);
        if(timer_3 <= now) digitalWrite(PIN_MOTOR_3, LOW);
        if(timer_4 <= now) digitalWrite(PIN_MOTOR_4, LOW);
    }
}