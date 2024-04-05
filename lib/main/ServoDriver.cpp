#include "ServoDriver.h"
#include "Printer.h"
extern Printer printer;

ServoDriver::ServoDriver() : DataSource("angle", "int") { setServoOut(0); }

void ServoDriver::init(void) { pinMode(RUDDER_SERVO_PIN, OUTPUT); }

void ServoDriver::apply(void) { analogWrite(RUDDER_SERVO_PIN, servoOut); }

void ServoDriver::drive(int angle) {
  setServoOut(angle);
  apply();
  printState();
}

void ServoDriver::setServoOut(int angle) {
  servoOut = (int)((1.5 + (float)angle / 90.0 * 0.5) / 20.0 * 255);

  if (servoOut < 12.75)
    servoOut = 12.75;
  if (servoOut > 25.5)
    servoOut = 25.5;
}