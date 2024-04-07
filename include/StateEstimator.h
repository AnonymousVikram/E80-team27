#ifndef STATE_ESTIMATOR_H_INCLUDED
#define STATE_ESTIMATOR_H_INCLUDED

#include "TimingOffsets.h"
#include <Arduino.h>
#include <SensorIMU.h>
#include <string>

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
class StateEstimator {
public:
  StateEstimator(void);

  // init
  void init(void);

  // State Access
  state_t prevState;
  state_t curState;

  void updateState(void);
  String printState(void);

  std::string logData(void);

  int lastExecutionTime = -1;

  std::string headers =
      "State Estimator X [m],State Estimator Y [m],State Estimator Z [m],State "
      "Estimator Roll [rad],State Estimator Pitch [rad],State Estimator Yaw "
      "[rad]";

private:
  float dt = LOOP_PERIOD / 1000.0; // [s]
};

#endif
