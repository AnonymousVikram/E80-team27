#ifndef SENSOR_GYRO_H
#define SENSOR_GYRO_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ISM330DHCX.h>
#include "DataSource.h"

typedef struct {
  float accelX;   // [mg] (g=acceleration due to gravity)
  float accelY;   // [mg]
  float accelZ;   // [mg]
  float droll;     // [degrees/s]
  float dpitch;    // [degrees/s]
  float dheading;  // [degrees/s]
} gyro_state_t;

typedef struct {
  float roll;     // [degrees]
  float pitch;    // [degrees]
  float yaw;      // [degrees]
} orientation_t;

class SensorGyro : public DataSource {
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

  // from DataSource
  size_t writeDataBytes(unsigned char * buffer, size_t idx);

  int lastExecutionTime = -1;

private:

  // Create sensor instance
  Adafruit_ISM330DHCX myGyro;

  // Offsets applied to raw x/y/z accel values
  float accel_offsets[3]      = { 0.0F, -0.25F, 0.09F };
  
  // Offsets applied to raw x/y/z gyro values
  float gyro_offsets[3]      = { 0.01F, 0.0F, 0.0F };

};

#endif
