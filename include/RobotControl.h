#ifndef ROBOT_CONTROL_H_INCLUDED
#define ROBOT_CONTROL_H_INCLUDED

#define proximity_threshold 0.1
#define yaw_threshold 0.05
#define depth_threshold 0.1
#define waitTime 5000

#include "MotorDriver.h"
#include "Pinouts.h"
#include "Printer.h"
#include "ServoDriver.h"
#include "StateEstimator.h"
#include <Arduino.h>
#include <string>

extern Printer printer;
extern MotorDriver motorDriver;
extern ServoDriver servoDriver;
extern StateEstimator stateEstimator;

#define NUMWAYPOINTS 3

class RobotControl {
public:
  RobotControl(void);
  void init(int numWaypoints, float waypoints[][3]);
  void update(void);

  String printString(void);
  String printWaypoint(void);

  int lastExecutionTime = -1;

  std::string headers =
      "Waypoint X [m], Waypoint Y [m], Waypoint Z [m], Waiting [bool]";

  std::string logData(void);

private:
  state_t *state = &stateEstimator.curState;

  int numWaypoints;
  int currentWaypoint = 0;
  float distanceToWaypoint(float x, float y, float z);
  float xyDistanceToWaypoint(float x, float y);
  float zDistanceToWaypoint(float z);
  float deltaYawToWaypoint(float x, float y, float *heading);
  float deltaRoll(float roll);
  void updateWaypoint(float dist);
  float angleDiff(float angle);

  // control values
  int motorPowers[NUM_MOTORS];

  // control gains
  float diveKp = 1000;
  float diveKd = 100;
  float throttleKp = 1000;
  float throttleKd = 100;
  float yawKp = 5;
  float yawKd = 100;
  float rollKp = 1000;
  float rollKd = 100;

  bool waiting = false;
  int waitBeg = 0;

  float waypoints[NUMWAYPOINTS][3];
};
#endif