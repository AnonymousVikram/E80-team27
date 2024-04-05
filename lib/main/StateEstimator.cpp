#include "StateEstimator.h"
#include "FlowSensor.h"
#include "PressureSensor.h"
#include "Printer.h"
#include "SensorGyro.h"
#include "SensorIMU.h"
#include "TimingOffsets.h"
#include <math.h>

extern Printer printer;
extern SensorIMU imu;
extern FlowSensor flowSensor;
extern PressureSensor pressureSensor;
extern SensorGyro gyro;

inline float angleDiff(float a) {
  while (a < -PI)
    a += 2 * PI;
  while (a > PI)
    a -= 2 * PI;
  return a;
}

StateEstimator::StateEstimator(void)
    : DataSource("x,y,z,roll,pitch,yaw",
                 "float,float,float,float,float") // from DataSource
{}

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
  float v = flowSensor.velocity;
  float depth = pressureSensor.depth;

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
  curState.x = prevState.x + v * cos(yaw) * dt + ax * dt * dt / 2;
  curState.y = prevState.y + v * sin(yaw) * dt + ay * dt * dt / 2;
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

size_t StateEstimator::writeDataBytes(unsigned char *buffer, size_t idx) {
  float *data_slot = (float *)&buffer[idx];
  data_slot[0] = curState.x;
  data_slot[1] = curState.y;
  data_slot[2] = curState.z;
  data_slot[3] = curState.roll;
  data_slot[4] = curState.pitch;
  data_slot[5] = curState.yaw;
  return idx + 6 * sizeof(float);
}
