#ifndef ROBOT_CONTROL_H_INCLUDED
#define ROBOT_CONTROL_H_INCLUDED

#define proximity_threshold 0.1

#include "DataSource.h"
#include "MotorDriver.h"
#include "Pinouts.h"
#include "Printer.h"
#include "ServoDriver.h"
#include "StateEstimator.h"
#include <Arduino.h>

extern Printer printer;
extern MotorDriver motorDriver;
extern ServoDriver servoDriver;
extern StateEstimator stateEstimator;

class RobotControl : public DataSource {
public:
  RobotControl(void);
  void init(const int numWaypoints, const float waypoints[][3]);
  void update(void);

  String printString(void);
  String printWaypoint(void);

  size_t writeDataBytes(unsigned char *buffer, size_t idx);

  int lastExecutionTime = -1;

private:
  int numWaypoints;
  int currentWaypoint = 0;
  float waypoints[][3];
  float distanceToWaypoint(float x, float y, float z, float *heading);
  void updateWaypoint(void);
  

  // control values
  int motorPowers[NUM_MOTORS] = {0, 0, 0, 0, 0, 0};
  int rudderAngle = 0;
  int throttle = 0;

  // control gains
  float KpThrottle = 1000;
  float KdThrottle = 0.1;
  float KpYaw = 1000;
  float KdYaw = 0.1;

  // control state
  
};
#endif