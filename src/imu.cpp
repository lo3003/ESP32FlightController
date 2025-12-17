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
    gyro_address = EEPROM.read(32);
    if(gyro_address == 0) gyro_address = 0x68; // Sécurité si EEPROM vide

    // Init MPU6050
    Wire.beginTransmission(gyro_address); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
    Wire.beginTransmission(gyro_address); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission(); // 500dps
    Wire.beginTransmission(gyro_address); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission(); // +/-8g
    Wire.beginTransmission(gyro_address); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission(); // Filter 42Hz

    // Calibration
    Serial.println("Calibration Gyro (Ne pas bouger)...");
    for (int i = 0; i < 2000 ; i++){
        if(i % 200 == 0) Serial.print(".");
        
        Wire.beginTransmission(gyro_address);
        Wire.write(0x43); // Registre Gyro
        Wire.endTransmission();
        Wire.requestFrom(gyro_address, 6);
        while(Wire.available() < 6);
        
        gyro_axis[1] = Wire.read()<<8|Wire.read();
        gyro_axis[2] = Wire.read()<<8|Wire.read();
        gyro_axis[3] = Wire.read()<<8|Wire.read();

        gyro_axis_cal[1] += gyro_axis[1];
        gyro_axis_cal[2] += gyro_axis[2];
        gyro_axis_cal[3] += gyro_axis[3];
        
        // Simuler activité ESC pour éviter les bips
        // motors_write_direct(1000, 1000, 1000, 1000); 
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
    Wire.requestFrom(gyro_address, 14);
    while(Wire.available() < 14);
    
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

    // Calculs Angles (Code YMFC port)
    // Utilisation des configs EEPROM pour l'orientation
    byte roll_axis = EEPROM.read(28) & 0b00000011;
    bool roll_inv = EEPROM.read(28) & 0b10000000;
    
    byte pitch_axis = EEPROM.read(29) & 0b00000011;
    bool pitch_inv = EEPROM.read(29) & 0b10000000;
    
    byte yaw_axis = EEPROM.read(30) & 0b00000011;
    bool yaw_inv = EEPROM.read(30) & 0b10000000;

    double gyro_roll = gyro_axis[roll_axis]; if(roll_inv) gyro_roll *= -1;
    double gyro_pitch = gyro_axis[pitch_axis]; if(pitch_inv) gyro_pitch *= -1;
    double gyro_yaw = gyro_axis[yaw_axis]; if(yaw_inv) gyro_yaw *= -1;

    // Filtre Passe-Bas Simple
    drone->gyro_roll_input = (drone->gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);
    drone->gyro_pitch_input = (drone->gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);
    drone->gyro_yaw_input = (drone->gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);

    // Intégration Gyro
    drone->angle_pitch += gyro_pitch * 0.0000611;
    drone->angle_roll += gyro_roll * 0.0000611;
    
    // Transfert d'angles (Yaw)
    drone->angle_pitch -= drone->angle_roll * sin(gyro_yaw * 0.000001066);
    drone->angle_roll += drone->angle_pitch * sin(gyro_yaw * 0.000001066);

    // Accéléromètre
    long acc_x = acc_axis[pitch_axis]; if(pitch_inv) acc_x *= -1;
    long acc_y = acc_axis[roll_axis]; if(roll_inv) acc_y *= -1;
    long acc_z = acc_axis[yaw_axis]; if(yaw_inv) acc_z *= -1;

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