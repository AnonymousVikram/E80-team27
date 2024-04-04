#include "XYStateEstimator.h"
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

XYStateEstimator::XYStateEstimator(void)
    : DataSource("x,y", "float,float") // from DataSource
{}

void XYStateEstimator::init(void) {
  state.x = 0;
  state.y = 0;
  state.yaw = 0;
}

// function to apply haversine to gps coordinates to get distance and heading

void XYStateEstimator::haversine(float lat1, float lon1, float lat2, float lon2,
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

void XYStateEstimator::polarToCartesian(float distance, float heading, float *x,
                                        float *y) {
  *x = distance * cos(heading);
  *y = distance * sin(heading);
}

void XYStateEstimator::updateState(imu_state_t *imu_state_p,
                                   gps_state_t *gps_state_p) {
  if (gps_state_p->num_sat >= N_SATS_THRESHOLD) {
    gpsAcquired = 1;

    // set the values of state.x, state.y, and state.yaw
    // It can make use of the constants RADIUS_OF_EARTH, origin_lat, origin_lon
    // (see XYStateEstimator.h) You can access the current GPS latitude and
    // longitude readings with gps_state_p->lat and gps_state_p->lon You can
    // access the current imu heading with imu_state_p->heading Also note that
    // math.h is already included so you have access to trig functions [rad]

    ///////////////////////////////////////////////////////////////////
    // INSERT YAW, X and Y CALCULATION HERE
    //////////////////////////////////////////////////////////////////

    // Calculate the distance from the origin to the current GPS location
    float newLat = gps_state_p->lat;
    float newLon = gps_state_p->lon;
    float newHead = imu_state_p->heading; // check if the heading needs to be
                                          // fused with the GPS heading

    float distance, heading;
    haversine(origin_lat, origin_lon, newLat, newLon, &distance, &heading);

    // Calculate the x and y coordinates of the current GPS location
    float x, y;
    polarToCartesian(distance, heading, &x, &y);

    // Update the state
    state.x = x;
    state.y = y;
    state.yaw = newHead;

    // // The following formulae are derived from
    // //
    // https://robotics.stackexchange.com/questions/10450/conversion-gps-longitude-latitude-to-x-y-of-local-reference-frame
    // float newX = (RADIUS_OF_EARTH_M * cos(newLat) * cos(newLon) - x0) *
    //                  cos(initial_heading - PI / 2) -
    //              (RADIUS_OF_EARTH_M * cos(newLat) * sin(newLon) - y0) *
    //                  sin(initial_heading - PI / 2);
    // float newY = (RADIUS_OF_EARTH_M * cos(newLat) * cos(newLon) - x0) *
    //                  sin(initial_heading - PI / 2) +
    //              (RADIUS_OF_EARTH_M * cos(newLat) * sin(newLon) - y0) *
    //                  cos(initial_heading - PI / 2);

  } else {
    gpsAcquired = 0;
  }
}

String XYStateEstimator::printState(void) {
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

size_t XYStateEstimator::writeDataBytes(unsigned char *buffer, size_t idx) {
  float *data_slot = (float *)&buffer[idx];
  data_slot[0] = state.x;
  data_slot[1] = state.y;
  return idx + 2 * sizeof(float);
}
