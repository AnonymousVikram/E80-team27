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

class RobotControl {
public:
  RobotControl(void);
  void init(const int numWaypoints, const float waypoints[][3]);
  void update(void);

  String printString(void);
  String printWaypoint(void);

  int lastExecutionTime = -1;

  std::string headers =
      "Motor 1 Command [us],Motor 2 Command [us],Motor 3 Command [us],Motor 4 "
      "Command [us],Motor 5 Command [us],Motor 6 Command [us], Rudder Command "
      "[us], Waypoint X [m], Waypoint Y [m], Waypoint Z [m], Waiting [bool]";

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
  int motorPowers[NUM_MOTORS] = {0, 0, 0, 0, 0, 0};

  // control gains
  float diveKp = 1000;
  float diveKd = 0.1;
  float throttleKp = 1000;
  float throttleKd = 0.1;
  float yawKp = 1000;
  float yawKd = 0.1;
  float rollKp = 1000;
  float rollKd = 0.1;

  bool waiting = false;
  int waitBeg = 0;

  float waypoints[][3];
};
#endif