#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include "DataSource.h"
#include <Arduino.h>
#include <Wire.h>

typedef struct {
  float z; // [m]
} pressure_state_t;

class PressureSensor : public DataSource {
public:
  PressureSensor(void);

  // Starts the connection to the sensor
  void init(void);

  // Reads data from the sensor
  void read(void);

  // Latest reported pressure data is stored here
  pressure_state_t state;

  // prints state to serial
  String printPressure(void);

  // from DataSource
  size_t writeDataBytes(unsigned char *buffer, size_t idx);

  int lastExecutionTime = -1;

private:
  float pressure_cal_slope = 0.0F;
  float pressure_cal_offset = 0.0F;
};
#endif