#include "StateEstimator.h"
#include "Printer.h"
#include <math.h>

extern Printer printer;

inline float angleDiff(float a) {
  while (a < -PI)
    a += 2 * PI;
  while (a > PI)
    a -= 2 * PI;
  return a;
}

StateEstimator::StateEstimator(void)
    : DataSource("x,y,z,roll,pitch,yaw", "float,float,float,float,float") // from DataSource
{}

void StateEstimator::init(void) {
  state.x = 0;
  state.y = 0;
  state.z = 0;
  state.roll = 0;
  state.pitch = 0;
  state.yaw = 0;

  next_state.x = 0;
  next_state.y = 0;
  next_state.z = 0;
  next_state.roll = 0;
  next_state.pitch = 0;
  next_state.yaw = 0;

  control_vals.vx = 0;
  control_vals.vy = 0;
  control_vals.vz = 0;
  control_vals.v = 0;
  control_vals.ax = 0;
  control_vals.ay = 0;
  control_vals.az = 0;
}

// function to apply haversine to gps coordinates to get distance and heading

void StateEstimator::haversine(float lat1, float lon1, float lat2, float lon2,
                                 float *distance, float *heading) {
  // convert to radians
  lat1 = lat1 * PI / 180;
  lon1 = lon1 * PI / 180;
  lat2 = lat2 * PI / 180;
  lon2 = lon2 * PI / 180;

  // haversine formula
  float dlat = lat2 - lat1;
  float dlon = lon2 - lon1;
  float a =
      pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
  *distance = 2 * RADIUS_OF_EARTH_M * asin(sqrt(a));

  // heading formula
  float y = sin(dlon) * cos(lat2);
  float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);
  *heading = atan2(y, x);
}

void StateEstimator::polarToCartesian(float distance, float heading, float *x,
                                        float *y) {
  *x = distance * cos(heading);
  *y = distance * sin(heading);
}


String StateEstimator::printState(void) {
  String currentState = "";
  int decimals = 2;
  if (!gpsAcquired) {
    currentState += "XY_State: Waiting to acquire more satellites...";
  } else {
    currentState += "XY_State: x: ";
    currentState += String(state.x, decimals);
    currentState += "[m], ";
    currentState += "y: ";
    currentState += String(state.y, decimals);
    currentState += "[m], ";
    currentState += "yaw: ";
    currentState += String(state.yaw, decimals);
    currentState += "[rad]; ";
  }
  return currentState;
}

size_t StateEstimator::writeDataBytes(unsigned char *buffer, size_t idx) {
  float *data_slot = (float *)&buffer[idx];
  data_slot[0] = state.x;
  data_slot[1] = state.y;
  data_slot[2] = state.z;
  data_slot[3] = state.roll;
  data_slot[4] = state.pitch;
  data_slot[5] = state.yaw;
  return idx + 6 * sizeof(float);
}
