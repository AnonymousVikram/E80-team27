#ifndef STATE_ESTIMATOR_HEADER
#define STATE_ESTIMATOR_HEADER

#include <Arduino.h>

#include "DataSource.h"
#include <SensorGPS.h>
#include <SensorIMU.h>

#define RADIUS_OF_EARTH_M 6371000 // [m]

typedef struct {
  float x = 0;   // x position in global frame [m]
  float y = 0;   // y position in global frame [m]
  float z = 0;   // z position in global frame [m]
  float roll = 0; // roll in global frame [rad]
  float pitch = 0; // pitch in global frame [rad]
  float yaw = 0; // yaw in global frame [rad] CCW from magnetic east
} state_t;

typedef struct {
    float vx = 0; // x velocity in global frame [m/s]
    float vy = 0; // y velocity in global frame [m/s]
    float vz = 0; // z velocity in global frame [m/s]
    float v = 0; // velocity magnitude in global frame [m/s]
    float ax = 0; // x acceleration in global frame [m/s^2]
    float ay = 0; // y acceleration in global frame [m/s^2]
    float az = 0; // z acceleration in global frame [m/s^2]
} state_pred_t;


/*
 * StateEstimator class keeps track of the robot's state,
 * incorporating measurements of the system outputs from the various
 * sensors like IMU and GPS, as well as the control inputs to the system.
 */
class StateEstimator : public DataSource {
public:
  StateEstimator(void);

  // init
  void init(void);

  // haversine function
  void haversine(float lat1, float lon1, float lat2, float lon2, float *distance, float *heading);
  void polarToCartesian(float distance, float heading, float *x, float *y);
  // State Access
  state_t state;
  state_t next_state;
  state_pred_t control_vals;

  void updateState(imu_state_t *imu_state_p, gps_state_t *gps_state_p);
  String printState(void);

  // from DataSource
  size_t writeDataBytes(unsigned char *buffer, size_t idx);

  int lastExecutionTime = -1;

private:
  // set coordinates of chosen origin below
  const float origin_lat = 34.106465;
  const float origin_lon = -117.712488;
  const float initial_heading = 0; // [rad]

  const float x0 = RADIUS_OF_EARTH_M * cos(origin_lat) * cos(origin_lon);
  const float y0 = RADIUS_OF_EARTH_M * cos(origin_lat) * sin(origin_lon);
  
  bool dive;
  bool gpsAcquired;
};

#endif
