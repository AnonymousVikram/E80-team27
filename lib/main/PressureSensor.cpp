#include "PressureSensor.h"
#include "Pinouts.h"
#include "Printer.h"
extern Printer printer;

PressureSensor::PressureSensor(void) : DataSource("z", "float") {
  state.z = 0.0F;
}

void PressureSensor::init(void) {
  Serial.print("Initializing Pressure Sensor... ");
  float curPres = analogRead(PRESSURE_SENSOR_PIN);
  state.z = (curPres + pressure_cal_offset) * pressure_cal_slope;
  Serial.println("done, cur pressure: " + String(state.z));
}

void PressureSensor::read(void) {
  float curPres = analogRead(PRESSURE_SENSOR_PIN);
  state.z = (curPres + pressure_cal_offset) * pressure_cal_slope;
}

String PressureSensor::printPressure(void) {
  String PressureStr = "[Pressure Sensor] ";
  PressureStr += "Depth: ";
  PressureStr += state.z;
  return PressureStr;
}

size_t PressureSensor::writeDataBytes(unsigned char *buffer, size_t idx) {
  float *data_slot = (float *)&buffer[idx];
  data_slot[0] = state.z;
  return idx + sizeof(float);
}