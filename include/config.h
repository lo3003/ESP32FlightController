#ifndef CONFIG_H
#define CONFIG_H

// --- PINOUT (ESP32 30-PIN) ---
#define PIN_MOTOR_1    13  // Avant Droit (CCW)
#define PIN_MOTOR_2    27  // Arrière Droit (CW)
#define PIN_MOTOR_3    26  // Arrière Gauche (CCW)
#define PIN_MOTOR_4    25  // Avant Gauche (CW)

#define PIN_RADIO_1    14  // Roll
#define PIN_RADIO_2    4   // Pitch
#define PIN_RADIO_3    16  // Throttle
#define PIN_RADIO_4    17  // Yaw

#define PIN_LED        2

// --- I2C ---
#define GYRO_ADDRESS   0x68
#define I2C_SPEED      400000

// --- PARAMETRES DE VOL ---
// PID Roll
#define PID_P_ROLL     1.3
#define PID_I_ROLL     0.04
#define PID_D_ROLL     18.0
#define PID_MAX_ROLL   400

// PID Pitch
#define PID_P_PITCH    1.3
#define PID_I_PITCH    0.04
#define PID_D_PITCH    18.0
#define PID_MAX_PITCH  400

// PID Yaw
#define PID_P_YAW      4.0
#define PID_I_YAW      0.02
#define PID_D_YAW      0.0
#define PID_MAX_YAW    400

// --- REGLAGES MOTEURS ---
#define MAX_THROTTLE        1800
#define MIN_THROTTLE_IDLE   1100 // Vitesse ralenti armé
#define MOTOR_OFF           1000

// --- SYSTEME ---
#define LOOP_TIME_US   4000 // 250Hz

#endif