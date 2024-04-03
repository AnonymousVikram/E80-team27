#include "SensorGyro.h"
#include "Printer.h"
#include "TimingOffsets.h"
extern Printer printer;

SensorGyro::SensorGyro(void)
    : DataSource("rollGyro,pitchGyro,headingGyro,gyroX,gyroY,gyroZ",
                 "float,float,float,float,float,float") {}

void SensorGyro::init(void) {
  Serial.print("Initializing Gyro... ");

  // Creating i2c interface
  myGyro.begin_I2C();

  Serial.println("done");
}

void SensorGyro::read(void) {
  // Get new data samples
  sensors_event_t accel_event;
  sensors_event_t gyro_event;
  sensors_event_t temp_event;
  myGyro.getEvent(&accel_event, &gyro_event, &temp_event);

  // Remove offsets from acceleration measurements
  state.accelX = accel_event.acceleration.x - accel_offsets[0];
  state.accelY = accel_event.acceleration.y - accel_offsets[1];
  state.accelZ = accel_event.acceleration.z - accel_offsets[2];

  // Remove offsets from gyro measurements
  state.droll = gyro_event.gyro.x - gyro_offsets[0];
  state.dpitch = gyro_event.gyro.y - gyro_offsets[1];
  state.dheading = gyro_event.gyro.z - gyro_offsets[2];

  // Update orientation if the gyro crosses a threshold
  if (abs(state.droll) > GYRO_THRESHOLD) {
    orientation.roll += state.droll * LOOP_PERIOD / 1000.0;
  }
  if (abs(state.dpitch) > GYRO_THRESHOLD) {
    orientation.pitch += state.dpitch * LOOP_PERIOD / 1000.0;
  }
  if (abs(state.dheading) > GYRO_THRESHOLD) {
    orientation.yaw += state.dheading * LOOP_PERIOD / 1000.0;
  }
}

String SensorGyro::printRollPitchYaw(void) {
  String GyroStr = "[Gyro Angles] ";
  GyroStr += "dRoll: ";
  GyroStr += state.droll * 180 / PI;
  GyroStr += " dPitch: ";
  GyroStr += state.dpitch * 180 / PI;
  GyroStr += " dYaw: ";
  GyroStr += state.dheading * 180 / PI;
  return GyroStr;
}

String SensorGyro::printAccels(void) {
  String GyroStr = "[Gyro Accels] ";
  GyroStr += "Accel X: ";
  GyroStr += state.accelX;
  GyroStr += " Accel Y: ";
  GyroStr += state.accelY;
  GyroStr += " Accel Z: ";
  GyroStr += state.accelZ;
  return GyroStr;
}

String SensorGyro::printOrientation(void) {
  String GyroStr = "[Gyro Orientation] ";
  GyroStr += "Roll: ";
  GyroStr += orientation.roll * 180 / PI;
  GyroStr += " Pitch: ";
  GyroStr += orientation.pitch * 180 / PI;
  GyroStr += " Yaw: ";
  GyroStr += orientation.yaw * 180 / PI;
  return GyroStr;
}

size_t SensorGyro::writeDataBytes(unsigned char *buffer, size_t idx) {
  float *data_slot = (float *)&buffer[idx];
  data_slot[0] = state.droll;
  data_slot[1] = state.dpitch;
  data_slot[2] = state.dheading;
  data_slot[3] = state.accelX;
  data_slot[4] = state.accelY;
  data_slot[5] = state.accelZ;
  return idx + 6 * sizeof(float);
}