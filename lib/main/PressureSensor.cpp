#include "PressureSensor.h"
#include "Pinouts.h"
#include "Printer.h"
extern Printer printer;

PressureSensor::PressureSensor(void) : DataSource("z", "float") {
  depth = 0.0F;
}

void PressureSensor::init(void) {
  pinMode(PRESSURE_SENSOR_PIN, INPUT);
  Serial.print("Initializing Pressure Sensor... ");
  float curPres = analogRead(PRESSURE_SENSOR_PIN);
  depth = (curPres + pressure_cal_offset) * pressure_cal_slope;
  Serial.println("done, cur pressure: " + String(depth));
}

void PressureSensor::read(void) {
  float curPres = analogRead(PRESSURE_SENSOR_PIN);
  curPres = curPres / 1023.0 * 3.3;
  depth = (curPres + pressure_cal_offset) * pressure_cal_slope;
}

String PressureSensor::printPressure(void) {
  String PressureStr = "[Pressure Sensor] ";
  PressureStr += "Depth: ";
  PressureStr += depth;
  return PressureStr;
}

size_t PressureSensor::writeDataBytes(unsigned char *buffer, size_t idx) {
  float *data_slot = (float *)&buffer[idx];
  data_slot[0] = depth;
  return idx + sizeof(float);
}