#include <Arduino.h>
#include "motors.h"
#include "config.h"

// Variables globales pour les impulsions moteurs
int esc_1, esc_2, esc_3, esc_4;

// Configuration des canaux PWM ESP32 (0 à 15 disponibles)
#define PWM_CH1 0
#define PWM_CH2 1
#define PWM_CH3 2
#define PWM_CH4 3

void motors_init() {
    // Configuration du Timer LEDC (Hardware PWM)
    // Fréquence définie dans config.h (250Hz), Résolution 14 bits (0-16383)
    ledcSetup(PWM_CH1, ESC_FREQ, 14);
    ledcSetup(PWM_CH2, ESC_FREQ, 14);
    ledcSetup(PWM_CH3, ESC_FREQ, 14);
    ledcSetup(PWM_CH4, ESC_FREQ, 14);

    // Attachement des Pins aux canaux
    ledcAttachPin(PIN_MOTOR_1, PWM_CH1);
    ledcAttachPin(PIN_MOTOR_2, PWM_CH2);
    ledcAttachPin(PIN_MOTOR_3, PWM_CH3);
    ledcAttachPin(PIN_MOTOR_4, PWM_CH4);

    motors_stop();
}

void motors_stop() {
    esc_1 = MOTOR_OFF; 
    esc_2 = MOTOR_OFF; 
    esc_3 = MOTOR_OFF; 
    esc_4 = MOTOR_OFF;
    motors_write(); // Applique immédiatement
}

void motors_mix(DroneState *drone) {
    int raw_throttle = drone->channel_3;

    // Sécurité basse
    if(raw_throttle < 1000) raw_throttle = 1000;

    // --- RÉINTÉGRATION VFINAL : ADOUCISSEMENT GAZ ---
    // Réduction de la plage à 85% pour maniabilité 
    int throttle = 1000 + (raw_throttle - 1000) * 0.85;

    // Sécurité Max définie dans config.h
    if (throttle > MAX_THROTTLE_FLIGHT) throttle = MAX_THROTTLE_FLIGHT;

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

    if(max_val > MAX_THROTTLE_FLIGHT) {
        int diff = max_val - MAX_THROTTLE_FLIGHT;
        esc_1_calc -= diff; esc_2_calc -= diff;
        esc_3_calc -= diff; esc_4_calc -= diff;
    }

    // Assignation et limites (1080 est le ralenti mini armé VFinal)
    // On utilise les constantes de config.h si possible, sinon valeurs par défaut
    esc_1 = constrain(esc_1_calc, MIN_THROTTLE_IDLE, MAX_THROTTLE_FLIGHT);
    esc_2 = constrain(esc_2_calc, MIN_THROTTLE_IDLE, MAX_THROTTLE_FLIGHT);
    esc_3 = constrain(esc_3_calc, MIN_THROTTLE_IDLE, MAX_THROTTLE_FLIGHT);
    esc_4 = constrain(esc_4_calc, MIN_THROTTLE_IDLE, MAX_THROTTLE_FLIGHT);
}

// Génération PWM Hardware (Non-Bloquante)
void motors_write() {
    // Conversion us -> Duty Cycle 14 bits
    // Période 250Hz = 4000us. 
    // Duty = (Pulse_us / 4000) * 16383
    
    // Note: On utilise des calculs entiers pour la rapidité
    ledcWrite(PWM_CH1, (esc_1 * 16383) / 20000);
    ledcWrite(PWM_CH2, (esc_2 * 16383) / 20000);
    ledcWrite(PWM_CH3, (esc_3 * 16383) / 20000);
    ledcWrite(PWM_CH4, (esc_4 * 16383) / 20000);
}

// --- LA FONCTION MANQUANTE ---
void motors_write_direct(int m1, int m2, int m3, int m4) {
    esc_1 = m1;
    esc_2 = m2;
    esc_3 = m3;
    esc_4 = m4;
    motors_write();
}