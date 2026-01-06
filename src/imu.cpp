#include <Arduino.h>
#include <Wire.h>
#include "imu.h"
#include "config.h"

double gyro_axis_cal[4];
int acc_axis[4], gyro_axis[4];
// Adresse I2C codée en dur
const int gyro_address = 0x68; 

void imu_init() {
    Wire.beginTransmission(gyro_address); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
    Wire.beginTransmission(gyro_address); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission(); 
    Wire.beginTransmission(gyro_address); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission(); 
    Wire.beginTransmission(gyro_address); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission(); 

    Serial.println("Calibration Gyro...");
    for (int i = 0; i < 2000 ; i++){
        if(i % 200 == 0) Serial.print(".");
        
        Wire.beginTransmission(gyro_address); Wire.write(0x43); Wire.endTransmission();
        
        // Sécurité I2C (Anti-Crash)
        if (Wire.requestFrom(gyro_address, 6) != 6) { delay(3); continue; }
        
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
    Wire.beginTransmission(gyro_address); Wire.write(0x3B); Wire.endTransmission();
    if (Wire.requestFrom(gyro_address, 14) != 14) return;
    
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

    // --- CONFIGURATION HARDCODÉE (STANDARD) ---
    // Roll = Axe Y du capteur (Standard MPU6050)
    // Pitch = Axe X du capteur
    // Yaw = Axe Z
    // A modifier ici si votre gyroscope est tourné de 90°
    double gyro_roll = gyro_axis[2];   // Axe Y
    double gyro_pitch = gyro_axis[1];  // Axe X
    double gyro_yaw = gyro_axis[3];    // Axe Z
    
    gyro_pitch *= -1; 
    gyro_yaw *= -1;

    drone->gyro_roll_input = (drone->gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);
    drone->gyro_pitch_input = (drone->gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);
    drone->gyro_yaw_input = (drone->gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);

    drone->angle_pitch += gyro_pitch * 0.0000611;
    drone->angle_roll += gyro_roll * 0.0000611;
    drone->angle_pitch -= drone->angle_roll * sin(gyro_yaw * 0.000001066);
    drone->angle_roll += drone->angle_pitch * sin(gyro_yaw * 0.000001066);

    long acc_x = acc_axis[1]; // Pitch (X)
    long acc_y = acc_axis[2]; // Roll (Y)
    long acc_z = acc_axis[3]; // Yaw (Z)
    
    acc_x *= -1; 
    acc_z *= -1;

    drone->acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
    
    if(abs(drone->acc_total_vector) < 100) return; // Sécurité Division

    if(abs(acc_y) < drone->acc_total_vector){
        float angle_pitch_acc = asin((float)acc_y/drone->acc_total_vector)* 57.296;
        drone->angle_pitch = drone->angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
    }
    if(abs(acc_x) < drone->acc_total_vector){
        float angle_roll_acc = asin((float)acc_x/drone->acc_total_vector)* -57.296;
        drone->angle_roll = drone->angle_roll * 0.9996 + angle_roll_acc * 0.0004;
    }
}