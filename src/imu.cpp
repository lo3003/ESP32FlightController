#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "imu.h"
#include "config.h"
#include "motors.h"

double gyro_axis_cal[4];
int acc_axis[4], gyro_axis[4];
int gyro_address;

void imu_init() {
    // On force l'adresse 0x68 (MPU6050 Standard)
    gyro_address = 0x68; 
    
    // Init MPU6050
    Wire.beginTransmission(gyro_address); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
    Wire.beginTransmission(gyro_address); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission(); 
    Wire.beginTransmission(gyro_address); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission(); 
    Wire.beginTransmission(gyro_address); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission(); 

    // Calibration
    Serial.println("Calibration Gyro (Ne pas bouger)...");
    for (int i = 0; i < 2000 ; i++){
        if(i % 200 == 0) Serial.print(".");
        
        Wire.beginTransmission(gyro_address);
        Wire.write(0x43);
        Wire.endTransmission();
        
        // --- CORRECTION SECURITE ---
        // On demande 6 octets. Si on en reçoit moins, on ignore ce tour.
        if (Wire.requestFrom(gyro_address, 6) != 6) {
            delay(3);
            continue; // On passe à l'itération suivante sans bloquer
        }
        // ---------------------------
        
        gyro_axis[1] = Wire.read()<<8|Wire.read();
        gyro_axis[2] = Wire.read()<<8|Wire.read();
        gyro_axis[3] = Wire.read()<<8|Wire.read();

        gyro_axis_cal[1] += gyro_axis[1];
        gyro_axis_cal[2] += gyro_axis[2];
        gyro_axis_cal[3] += gyro_axis[3];
        
        delay(3); 
    }
    gyro_axis_cal[1] /= 2000;
    gyro_axis_cal[2] /= 2000;
    gyro_axis_cal[3] /= 2000;
    Serial.println("OK");
}

void imu_read(DroneState *drone) {
    Wire.beginTransmission(gyro_address);
    Wire.write(0x3B);
    Wire.endTransmission();
    
    // --- CORRECTION SECURITE CRITIQUE ---
    // Au lieu de "while(Wire.available() < 14);" qui bloque tout...
    // On vérifie si la lecture a réussi.
    if (Wire.requestFrom(gyro_address, 14) != 14) {
        // Si erreur I2C, on quitte la fonction immédiatement pour ne pas crasher.
        // On garde les anciennes valeurs d'angles pour ce cycle.
        return; 
    }
    // ------------------------------------
    
    acc_axis[1] = Wire.read()<<8|Wire.read();
    acc_axis[2] = Wire.read()<<8|Wire.read();
    acc_axis[3] = Wire.read()<<8|Wire.read();
    int temp = Wire.read()<<8|Wire.read();
    gyro_axis[1] = Wire.read()<<8|Wire.read();
    gyro_axis[2] = Wire.read()<<8|Wire.read();
    gyro_axis[3] = Wire.read()<<8|Wire.read();

    gyro_axis[1] -= gyro_axis_cal[1];
    gyro_axis[2] -= gyro_axis_cal[2];
    gyro_axis[3] -= gyro_axis_cal[3];

    // Calculs Angles 
    // (Note: J'ai sécurisé la lecture EEPROM qui pouvait renvoyer n'importe quoi sur une puce vide)
    byte roll_axis = 1; // Force l'axe standard si EEPROM vide
    // byte roll_axis = EEPROM.read(28) & 0b00000011; // A décommenter si EEPROM configurée
    
    // Pour simplifier le debug, je force les axes standards ici.
    // Vous pourrez remettre la lecture EEPROM une fois le drone stable.
    double gyro_roll = gyro_axis[1]; 
    double gyro_pitch = gyro_axis[2]; 
    double gyro_yaw = gyro_axis[3];
    
    // Inversion standard (à ajuster selon montage)
    gyro_pitch *= -1; 
    gyro_yaw *= -1;

    // Filtre Passe-Bas
    drone->gyro_roll_input = (drone->gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);
    drone->gyro_pitch_input = (drone->gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);
    drone->gyro_yaw_input = (drone->gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);

    // Intégration Gyro
    drone->angle_pitch += gyro_pitch * 0.0000611;
    drone->angle_roll += gyro_roll * 0.0000611;
    
    drone->angle_pitch -= drone->angle_roll * sin(gyro_yaw * 0.000001066);
    drone->angle_roll += drone->angle_pitch * sin(gyro_yaw * 0.000001066);

    // Accéléromètre
    long acc_x = acc_axis[2]; // Pitch axis standard
    long acc_y = acc_axis[1]; // Roll axis standard
    long acc_z = acc_axis[3]; // Yaw axis standard
    
    acc_x *= -1; // Inversion standard
    acc_z *= -1;

    drone->acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
    
    if(abs(acc_y) < drone->acc_total_vector){
        float angle_pitch_acc = asin((float)acc_y/drone->acc_total_vector)* 57.296;
        drone->angle_pitch = drone->angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
    }
    if(abs(acc_x) < drone->acc_total_vector){
        float angle_roll_acc = asin((float)acc_x/drone->acc_total_vector)* -57.296;
        drone->angle_roll = drone->angle_roll * 0.9996 + angle_roll_acc * 0.0004;
    }
}