#ifndef SERVO_DRIVER_H_INCLUDED
#define SERVO_DRIVER_H_INCLUDED

#include "DataSource.h"
#include "Pinouts.h"
#include <Arduino.h>

class ServoDriver : public DataSource {
public:
  ServoDriver();

  void init(void);

  void apply(void);

  String printState(void);

  void drive(int angle);

  int servoOut = 0;

  size_t writeDataBytes(unsigned char *buffer, size_t idx);

private:
  void setServoOut(int angle);
};