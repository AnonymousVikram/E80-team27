#include "ServoDriver.h"
#include "Printer.h"
#include <cmath>
extern Printer printer;

ServoDriver::ServoDriver() : DataSource("angle", "float") { setServoOut(0); }

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

size_t ServoDriver::writeDataBytes(unsigned char *buffer, size_t idx) {
  int *data_slot = (int *)&buffer[idx];
  data_slot[0] = servoOut;
  return idx + sizeof(int);
}