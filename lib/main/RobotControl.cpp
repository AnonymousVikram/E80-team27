#include "RobotControl.h"
#include "MotorDriver.h"
#include "Printer.h"
#include "ServoDriver.h"
#include "StateEstimator.h"
#include <math.h>

extern Printer printer;
extern MotorDriver motor_driver;
extern ServoDriver rudder;
extern StateEstimator stateEstimator;

RobotControl::RobotControl(void)
    : DataSource("motorA,motorB,motorC,motorD,motorE,motorF",
                 "int,int,int,int,int,int") {
  for (int m = 0; m < NUM_MOTORS; m++) {
    motorPowers[m] = 0;
  }
}

void RobotControl::init(const int numWaypoints, const float waypoints[][3]) {
  this->numWaypoints = numWaypoints;
  for (int i = 0; i < numWaypoints; i++) {
    this->waypoints[i][0] = waypoints[i][0];
    this->waypoints[i][1] = waypoints[i][1];
    this->waypoints[i][2] = waypoints[i][2];
  }
}

void RobotControl::update(void) {

  // update the state
  stateEstimator.updateState();

  // get the current waypoint
  float x = state->x;
  float y = state->y;
  float z = state->z;
  float roll = state->roll;
  float heading = state->yaw;

  float distance = distanceToWaypoint(x, y, z);
  updateWaypoint(distance);

  // update the waypoint if we are close enough

  // calculate the desired heading
  float desiredHeading = atan2(waypoints[currentWaypoint][1] - y,
                               waypoints[currentWaypoint][0] - x);

  // calculate the heading error
  float headingError = angleDiff(desiredHeading - heading);

  // calculate the forward error
  float forwardError = distance * cos(headingError);

  // calculate the depth error
  float depthError = waypoints[currentWaypoint][2] - z;

  // Dive Control
  float diveError = depthError;
  float diveRate = stateEstimator.curState.z - stateEstimator.prevState.z;
  float diveControl = diveKp * diveError + diveKd * diveRate;

  // Yaw Control
  float yawError = headingError;
  float yawRate = stateEstimator.curState.yaw - stateEstimator.prevState.yaw;
  float yawControl = yawKp * yawError + yawKd * yawRate;

  // Throttle Control
  float throttleError = forwardError;
  float throttleRate = stateEstimator.curState.x - stateEstimator.prevState.x;
  float throttleControl =
      throttleKp * throttleError + throttleKd * throttleRate;

  // Roll Control
  float rollError = roll;
  float rollRate = stateEstimator.curState.roll - stateEstimator.prevState.roll;
  float rollControl = rollKp * rollError + rollKd * rollRate;

  // Update the motor powers
  if (diveError > depth_threshold) {
    motorPowers[0] = 0;
    motorPowers[1] = 0;
    motorPowers[2] = diveControl;
    motorPowers[3] = 0;
    motorPowers[4] = 0;
    motorPowers[5] = diveControl;
  } else if (yawError > yaw_threshold) {
    motorPowers[0] = -yawControl;
    motorPowers[1] = yawControl;
    motorPowers[2] = diveControl;
    motorPowers[3] = -yawControl;
    motorPowers[4] = yawControl;
    motorPowers[5] = diveControl;
  } else {
    // combine everything to navigate to the waypoint
    // motor powers 0 and 3 are the left motors
    // motor powers 1 and 4 are the right motors
    // motor powers 2 and 5 are the vertical motors
    motorPowers[0] = throttleControl - rollControl - yawControl;
    motorPowers[1] = throttleControl + rollControl + yawControl;
    motorPowers[2] = throttleControl + diveControl;
    motorPowers[3] = throttleControl + rollControl - yawControl;
    motorPowers[4] = throttleControl - rollControl + yawControl;
    motorPowers[5] = throttleControl + diveControl;
  }

  if (waiting) {
    // use the motors to hold the current position
    float ax = stateEstimator.curState.x - stateEstimator.prevState.x;
    float az = stateEstimator.curState.z - stateEstimator.prevState.z;
    motorPowers[0] = -throttleKp * ax;
    motorPowers[1] = -throttleKp * ax;
    motorPowers[2] = -diveKp * az;
    motorPowers[3] = -throttleKp * ax;
    motorPowers[4] = -throttleKp * ax;
    motorPowers[5] = -diveKp * az;
  }

  // limit the motor powers
  for (int m = 0; m < NUM_MOTORS; m++) {
    motorPowers[m] = constrain(motorPowers[m], -250, 250);
  }

  // update the motor powers
  motor_driver.drive(motorPowers[0], motorPowers[1], motorPowers[2],
                     motorPowers[3], motorPowers[4], motorPowers[5]);
}

String RobotControl::printString(void) {
  String output = "[RobotControl]: ";
  // output += "Current State: " + stateEstimator.printState() + "; ";
  output += "Current Motor Powers: ";
  for (int m = 0; m < NUM_MOTORS; m++) {
    output += String(motorPowers[m]) + ", ";
  }
  output += "Current Servo Angle: " + String(rudder.servoOut) + ";";
  return output;
}

String RobotControl::printWaypoint(void) {
  String output = "[RobotControl]: ";
  output += "Current Waypoint: " + String(currentWaypoint) + "; ";
  output += "Current Waypoint: ";
  output += "x: " + String(waypoints[currentWaypoint][0]) + ", ";
  output += "y: " + String(waypoints[currentWaypoint][1]) + ", ";
  output += "z: " + String(waypoints[currentWaypoint][2]) + "; ";
  output += "Waiting: " + String(waiting) + ";";
  return output;
}

size_t RobotControl::writeDataBytes(unsigned char *buffer, size_t idx) {
  int *int_slot = (int *)&buffer[idx];
  for (int m = 0; m < NUM_MOTORS; m++) {
    int_slot[m] = motorPowers[m];
  }
  return idx + NUM_MOTORS * sizeof(int);
}

float RobotControl::distanceToWaypoint(float x, float y, float z) {
  float dx = waypoints[currentWaypoint][0] - x;
  float dy = waypoints[currentWaypoint][1] - y;
  float dz = waypoints[currentWaypoint][2] - z;
  return sqrt(dx * dx + dy * dy + dz * dz);
}

float RobotControl::xyDistanceToWaypoint(float x, float y) {
  float dx = waypoints[currentWaypoint][0] - x;
  float dy = waypoints[currentWaypoint][1] - y;
  return sqrt(dx * dx + dy * dy);
}

float RobotControl::zDistanceToWaypoint(float z) {
  return waypoints[currentWaypoint][2] - z;
}

float RobotControl::deltaYawToWaypoint(float x, float y, float *heading) {
  float dx = waypoints[currentWaypoint][0] - x;
  float dy = waypoints[currentWaypoint][1] - y;
  *heading = atan2(dy, dx);
  return angleDiff(*heading - stateEstimator.curState.yaw);
}

float RobotControl::deltaRoll(float roll) {
  return angleDiff(roll - stateEstimator.curState.roll);
}

void RobotControl::updateWaypoint(float dist) {
  // check if we've reached the last waypoint
  if (currentWaypoint == numWaypoints - 1) {
    waiting = true;
    return;
  }
  // update waiting if the time has elapsed
  if (waiting) {
    if (millis() - waitBeg > waitTime) {
      waiting = false;
      currentWaypoint++;
    }
    return;
  }

  // check if we've reached the current waypoint
  if (dist < proximity_threshold) {
    waitBeg = millis();
    waiting = true;
  }
}

float RobotControl::angleDiff(float angle) {
  while (angle > M_PI) {
    angle -= 2 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2 * M_PI;
  }
  return angle;
}
