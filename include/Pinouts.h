#ifndef __PINOUTS_H__
#define __PINOUTS_H__

// User Interface
#define GPS_LOCK_LED 20
#define USER_BUTTON 2

// ************** For Version 3 of the Motherboard ***********
// Motor A: pins 3,4 L
#define MOTOR_A_DIRECTION 4 // IN1
#define MOTOR_A_SPEED 3     // IN2
// Motor B: pins 5,6 R
#define MOTOR_B_DIRECTION 6 // IN1
#define MOTOR_B_SPEED 5     // IN2
// Motor C: pins 23,22 V
#define MOTOR_C_DIRECTION 22 // IN1 // Teensy pin A08
#define MOTOR_C_SPEED 23     // IN2 // Teensy pin A09

// Motor D: pins 14, 15 L
#define MOTOR_D_DIRECTION 15 // IN1 // Teensy pin A01
#define MOTOR_D_SPEED 14     // IN2 // Teensy pin A00 ! Should be 14

// Motor E: pins 28, 29 R
#define MOTOR_E_DIRECTION 29 // IN1
#define MOTOR_E_SPEED 28     // IN2

// Motor F: pins 24, 33 V
#define MOTOR_F_DIRECTION 33 // IN1
#define MOTOR_F_SPEED 24     // IN2 // Teensy pin A10
// **********************************************************

// Error Flags
#define ERROR_FLAG_A 7
#define ERROR_FLAG_B 8
#define ERROR_FLAG_C 9

// Depth Control
#define PRESSURE_SENSOR_PIN 20 // Teensy pin A12 // change back to 26

// Flow Sensor
#define FLOW_SENSOR_PIN 26 // Teensy pin A06 // break 20

// Rudder Servo
#define RUDDER_SERVO_PIN 25 // Teensy  pin A11
#define FREQ_READER_PIN 32  //! Should be 32

/*
 * Note: the Gyro is connected to I2C Bus 1:
 *  SCL: 16 (Teensy Pin A2)
 *  SDA: 17 (Teensy Pin A3)
 */

/*
 * Note: The Motherboard IMU is connected to I2C Bus 0:
 *  SCL: 19 (Teensy Pin A5)
 *  SDA: 18 (Teensy Pin A4)
 */

#endif
