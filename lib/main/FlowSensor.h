#ifndef FLOW_SENSOR_H_INCLUDED
#define FLOW_SENSOR_H_INCLUDED

#include <Arduino.h>
#include <Wire.h>
#include <string>

class FlowSensor {
public:
  FlowSensor(void);

  // Starts the connection to the sensor
  void init(void);

  // Reads data from the sensor
  void read(void);

  // Latest reported flow data is stored here
  float velocity = 0;

  // prints state to serial
  String printFlow(void);

  int lastExecutionTime = -1;

  std::string headers = "Flow Sensor Velocity [m/s]";

  std::string logData(void);

private:
  float flow_cal_slope = 1.0F;
  float flow_cal_offset = 0.0F;
  float pipe_cs_area = 0.5F * 0.0254F; // cross sectional area of pipe [m^2]

  // convert rate in L/min to velocity in m/s
  float Lmin_to_mps(float rate);
};
#endif