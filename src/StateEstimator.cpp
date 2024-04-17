#include "StateEstimator.h"
#include "FloatFormatter.h"
#include "FreqReader.h"
#include "PressureSensor.h"
#include "SensorGyro.h"
#include <math.h>

extern FloatFormatter formatter;

// extern SensorIMU imu;
extern FreqReader freqReader;
extern PressureSensor pSensor;
extern SensorGyro gyro;

inline float angleDiff(float a) {
  while (a < -PI)
    a += 2 * PI;
  while (a > PI)
    a -= 2 * PI;
  return a;
}

StateEstimator::StateEstimator(void) {}

void StateEstimator::init(void) {
  prevState.x = 0;
  prevState.y = 0;
  prevState.z = 0;
  prevState.roll = 0;
  prevState.pitch = 0;
  prevState.yaw = 0;

  curState.x = 0;
  curState.y = 0;
  curState.z = 0;
  curState.roll = 0;
  curState.pitch = 0;
  curState.yaw = 0;
}

void StateEstimator::updateState(void) {
  // update the state based on velocity from flow sensor, depth from pressure
  // sensor, and roll and yaw from gyro
  float v = freqReader.velocity;
  float depth = pSensor.depth;

  // Gyro Vals
  float roll = gyro.orientation.roll;
  float pitch = gyro.orientation.pitch;
  float yaw = gyro.orientation.yaw;

  float ax = gyro.state.accelX;
  float ay = gyro.state.accelY;
  // float az = gyro.state.accelZ;

  // IMU Vals
  // float ax2 = imu.state.accelX;
  // float ay2 = imu.state.accelY;
  // float az2 = imu.state.accelZ;

  // float roll2 = imu.state.roll;
  // float yaw2 = imu.state.heading;

  // Update state
  prevState = curState;
  curState.x = prevState.x + v * cos(yaw) * dt;
  curState.y = prevState.y + v * sin(yaw) * dt;
  curState.z = depth;
  curState.roll = roll;
  curState.pitch = pitch;
  curState.yaw = yaw;
}

String StateEstimator::printState(void) {
  String currentState = "[StateEstimator]: ";
  int decimals = 2;
  currentState += "State: x: " + String(curState.x, decimals) + " [m], ";
  currentState += "y: " + String(curState.y, decimals) + " [m], ";
  currentState += "z: " + String(curState.z, decimals) + " [m], ";
  currentState += "roll: " + String(curState.roll, decimals) + " [rad], ";
  currentState += "pitch: " + String(curState.pitch, decimals) + " [rad], ";
  currentState += "yaw: " + String(curState.yaw, decimals) + " [rad]; ";
  return currentState;
}

std::string StateEstimator::logData(void) {
  return formatter.format(curState.x) + "," + formatter.format(curState.y) +
         "," + formatter.format(curState.z) + "," +
         formatter.format(curState.roll) + "," +
         formatter.format(curState.pitch) + "," +
         formatter.format(curState.yaw) + ",";
}