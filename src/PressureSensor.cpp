#include "PressureSensor.h"
#include "FloatFormatter.h"
#include "Pinouts.h"

extern FloatFormatter formatter;

PressureSensor::PressureSensor(void) {}

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

std::string PressureSensor::logData(void) { return formatter.format(depth); }