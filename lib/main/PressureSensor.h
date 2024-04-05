#ifndef PRESSURE_SENSOR_H_INCLUDED
#define PRESSURE_SENSOR_H_INCLUDED

#include "DataSource.h"
#include <Arduino.h>
#include <Wire.h>

class PressureSensor : public DataSource {
public:
  PressureSensor(void);

  // Starts the connection to the sensor
  void init(void);

  // Reads data from the sensor
  void read(void);

  // Latest reported pressure data is stored here
  float depth;

  // prints state to serial
  String printPressure(void);

  // from DataSource
  size_t writeDataBytes(unsigned char *buffer, size_t idx);

  int lastExecutionTime = -1;

private:
  float pressure_cal_slope = 1.0F;
  float pressure_cal_offset = 0.0F;
};
#endif