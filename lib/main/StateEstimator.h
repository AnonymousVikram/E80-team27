#ifndef STATE_ESTIMATOR_HEADER
#define STATE_ESTIMATOR_HEADER

#include <Arduino.h>

#include "DataSource.h"
#include <SensorGPS.h>
#include <SensorIMU.h>
#include "TimingOffsets.h"

// #define RADIUS_OF_EARTH_M 6371000 // [m]

typedef struct {
  float x = 0;     // x position in global frame [m]
  float y = 0;     // y position in global frame [m]
  float z = 0;     // z position in global frame [m]
  float roll = 0;  // roll in global frame [rad]
  float pitch = 0; // pitch in global frame [rad]
  float yaw = 0;   // yaw in global frame [rad] CCW from magnetic east
} state_t;

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

  // State Access
  state_t prevState;
  state_t curState;

  void updateState(void);
  String printState(void);

  // from DataSource
  size_t writeDataBytes(unsigned char *buffer, size_t idx);

  int lastExecutionTime = -1;

private:
  float dt = LOOP_PERIOD / 1000.0; // [s]
};

#endif
