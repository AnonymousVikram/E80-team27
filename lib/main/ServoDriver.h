#ifndef SERVO_DRIVER_H_INCLUDED
#define SERVO_DRIVER_H_INCLUDED

#include "Pinouts.h"
#include <Arduino.h>
#include <string>

class ServoDriver {
public:
  ServoDriver();

  void init(void);

  void apply(void);

  String printState(void);

  void drive(float angle);

  float servoOut = 0;
  std::string logData(void);

  std::string headers = "Servo Angle [rad]";

private:
  void setServoOut(float angle);
};
#endif