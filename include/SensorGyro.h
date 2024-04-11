#ifndef SENSOR_GYRO_H_INCLUDED
#define SENSOR_GYRO_H_INCLUDED

#include <Adafruit_ISM330DHCX.h>
#include <Arduino.h>
#include <Wire.h>
#include <string>

#define GYRO_THRESHOLD 0.01F

typedef struct {
  double accelX;   // [mg] (g=acceleration due to gravity)
  double accelY;   // [mg]
  double accelZ;   // [mg]
  double droll;    // [degrees/s]
  double dpitch;   // [degrees/s]
  double dheading; // [degrees/s]
} gyro_state_t;

typedef struct {
  double roll;  // [degrees]
  double pitch; // [degrees]
  double yaw;   // [degrees]
} orientation_t;

class SensorGyro {
public:
  SensorGyro(void);

  // Starts the connection to the sensor
  void init(void);

  // Reads data from the sensor
  void read(void);

  // Latest reported orientation data is stored here
  gyro_state_t state;
  orientation_t orientation;

  // prints state to serial
  String printRollPitchYaw(void);
  String printAccels(void);
  String printOrientation(void);

  std::string logData(void);

  int lastExecutionTime = -1;

  std::string headers =
      "Accel X [mg],Accel Y [mg],Accel Z [mg],dRoll [deg/s],dPitch "
      "[deg/s],dHeading [deg/s],Roll [deg],Pitch [deg],Yaw [deg]";

private:
  // Create sensor instance
  Adafruit_ISM330DHCX myGyro;

  // Offsets applied to raw x/y/z accel values
  float accel_offsets[3] = {0.0F, -0.25F, 0.09F};

  // Offsets applied to raw x/y/z gyro values
  float gyro_offsets[3] = {0.01F, 0.0F, 0.0F};
};

#endif
