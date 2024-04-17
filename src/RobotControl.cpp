#include "RobotControl.h"
#include "FloatFormatter.h"
#include "MotorDriver.h"
#include "ServoDriver.h"
#include "StateEstimator.h"
#include <math.h>

extern MotorDriver motor_driver;
extern ServoDriver rudder;
extern StateEstimator stateEstimator;
extern FloatFormatter formatter;

RobotControl::RobotControl(void) {
  for (int m = 0; m < NUM_MOTORS; m++) {
    motorPowers[m] = 0;
  }
}

void RobotControl::init(int inNumWaypoints, float inWaypoints[][3]) {
  numWaypoints = inNumWaypoints;

  for (int i = 0; i < numWaypoints; i++) {
    for (int j = 0; j < 3; j++) {
      waypoints[i][j] = inWaypoints[i][j];
    }
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

  float distance = distanceToWaypoint(x, y);
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
  float yawError = heading;
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
  if (diveError > depth_threshold || waiting) { // if diving
    motorPowers[0] = 0;
    motorPowers[1] = 0;
    motorPowers[2] = diveControl;
    motorPowers[3] = 0;
    motorPowers[4] = diveControl;
  } else { // navigate
    // combine everything to navigate to the waypoint
    // motor powers 0 and 3 are the left motors
    // motor powers 1 and 4 are the right motors
    // motor powers 2 and 5 are the vertical motors
    motorPowers[0] = throttleControl;
    motorPowers[1] = throttleControl;
    motorPowers[2] = diveControl + rollControl;
    motorPowers[3] = throttleControl;
    motorPowers[4] = diveControl - rollControl;
  }

  // constrain the motor powers
  for (int m = 0; m < NUM_MOTORS; m++) {
    motorPowers[m] = constrain(motorPowers[m], -240, 240);
  }

  yawControl = constrain(yawControl, -PI / 3, PI / 3);
  rudder.drive(yawControl);

  // update the motor powers
  motor_driver.drive(motorPowers[0], motorPowers[1], motorPowers[2],
                     motorPowers[3], motorPowers[4]);
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

std::string RobotControl::logData(void) {
  return formatter.format(waypoints[currentWaypoint][0]) + "," +
         formatter.format(waypoints[currentWaypoint][1]) + "," +
         formatter.format(waypoints[currentWaypoint][2]) + "," +
         formatter.format(waiting) + ",";
}

float RobotControl::distanceToWaypoint(float x, float y) {
  float dx = waypoints[currentWaypoint][0] - x;
  float dy = waypoints[currentWaypoint][1] - y;
  return sqrt(dx * dx + dy * dy);
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
