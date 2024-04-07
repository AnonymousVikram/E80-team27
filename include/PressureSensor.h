#ifndef PRESSURE_SENSOR_H_INCLUDED
#define PRESSURE_SENSOR_H_INCLUDED

#include <Arduino.h>
#include <Wire.h>
#include <string>

class PressureSensor {
public:
  PressureSensor(void);

  // Starts the connection to the sensor
  void init(void);

  // Reads data from the sensor
  void read(void);

  // Latest reported pressure data is stored here
  float depth = 0;

  // prints state to serial
  String printPressure(void);

  int lastExecutionTime = -1;

  std::string headers = "Pressure Sensor Depth [m]";

  std::string logData(void);

private:
  float pressure_cal_slope = -5.0;
  float pressure_cal_offset = -2.04F;
};
#endif