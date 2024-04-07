#include "ServoDriver.h"
#include <cmath>

ServoDriver::ServoDriver() {}

void ServoDriver::init(void) {
  pinMode(RUDDER_SERVO_PIN, OUTPUT);
  analogWriteFrequency(RUDDER_SERVO_PIN, 50);
  apply();
}

void ServoDriver::apply(void) {
  analogWrite(RUDDER_SERVO_PIN, round(servoOut / 20.0 * 255.0));
}

void ServoDriver::drive(float angle) {
  setServoOut(angle);
  apply();
  printState();
}

void ServoDriver::setServoOut(float angle) {
  // angle = angle * 1.0 / 3.0;
  servoOut = 0.634011 * angle + 1.16311;
  // servoOut = angle;
  servoOut = constrain(servoOut, 0.6, 1.83);
}

String ServoDriver::printState(void) {
  String printString = "[Servo]: Angle: " + String(servoOut);
  return printString;
}

std::string ServoDriver::logData(void) {
  std::string data = std::to_string(servoOut) + ",";
  return data;
}