// In summer 2021 the motherboard had to be updated with new H-bridges. The new
// H-bridges needed to be driven by an H-bridge driver IN1 - Direction, IN2 -
// SPEED, rather than the Half-bridge driver for the old ones, switch IN1 & IN2
// for PWM and LOW to change direction. The code below, along with the .h file
// and the Pinouts.h file were changed. Erik Spjut August 2021.
#include "MotorDriver.h"
MotorDriver::MotorDriver() {
  for (int m = 0; m < NUM_MOTORS; m++) {
    motorValues[m] = 0;
    pwmValues[m] = 0;
    pwmDir[m] = 0;
    digitalWrite(motorPins[m][DIRECTION_PIN], 0);
    analogWrite(motorPins[m][SPEED_PIN], 0);
  }
}

void MotorDriver::init(void) {
  for (int m = 0; m < NUM_MOTORS; m++) {
    pinMode(motorPins[m][SPEED_PIN], OUTPUT);
    pinMode(motorPins[m][DIRECTION_PIN], OUTPUT);
  }
}

void MotorDriver::apply(void) {
  // determine direction and magnitude of spin required:
  for (int m = 0; m < NUM_MOTORS; m++) {
    pwmDir[m] = (motorValues[m] >= 0);
    pwmValues[m] = (motorValues[m] < 0) ? -motorValues[m] : motorValues[m];
    if (motorValues[m]) { // correct for deadzone if not zero
      pwmValues[m] =
          pwmValues[m] - MOTOR_DEADZONE * pwmValues[m] / 255 + MOTOR_DEADZONE;
    }
  }

  // write this information to motors
  for (int m = 0; m < NUM_MOTORS; m++) { // using pwmDir as 0 or 1
    digitalWrite(motorPins[m][DIRECTION_PIN], pwmDir[m]);
    analogWrite(motorPins[m][SPEED_PIN], pwmValues[m]);
  }
}

void MotorDriver::drive(int motorA_power, int motorB_power, int motorC_power,
                        int motorD_power, int motorE_power) {
  motorValues[0] = motorA_power; // M1
  motorValues[1] = motorB_power; // M2
  motorValues[2] = motorC_power; // M3
  motorValues[3] = motorD_power; // M4
  motorValues[4] = motorE_power; // M5
  apply();
  printState();
}

String MotorDriver::printState(void) {
  String printString =
      "[Motors]: PWMA: " + String(pwmDir[0] ? " " : "-") +
      String(pwmValues[0]) + " PWMB: " + String(pwmDir[1] ? " " : "-") +
      String(pwmValues[1]) + " PWMC: " + String(pwmDir[2] ? " " : "-") +
      String(pwmValues[2]) + " PWMD: " + String(pwmDir[3] ? " " : "-") +
      String(pwmValues[3]) + " PWME: " + String(pwmDir[4] ? " " : "-") +
      String(pwmValues[4]);
  return printString;
}

std::string MotorDriver::logData(void) {
  std::string data = "";
  for (int m = 0; m < NUM_MOTORS; m++) {
    data += std::to_string(pwmValues[m]) + ",";
    data = (pwmDir[m] ? "" : "-") + data;
  }
  return data;
}