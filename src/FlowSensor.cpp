#include "FlowSensor.h"
#include "Pinouts.h"

FlowSensor::FlowSensor(void) {}

void FlowSensor::init(void) {
  Serial.print("Initializing Flow Sensor... ");
  pinMode(FLOW_SENSOR_PIN, INPUT);
  Serial.println("done");
}

void FlowSensor::read(void) {
  // Read from sensor
  float curFlow = analogRead(FLOW_SENSOR_PIN);
  velocity = Lmin_to_mps(curFlow);
}

String FlowSensor::printFlow(void) {
  String FlowStr = "[Flow Sensor] ";
  FlowStr += "Velocity: ";
  FlowStr += velocity;
  return FlowStr;
}

std::string FlowSensor::logData(void) {
  std::string data = std::to_string(velocity) + ",";
  return data;
}

float FlowSensor::Lmin_to_mps(float rate) {
  // return (rate * flow_cal_slope + flow_cal_offset) / pipe_cs_area / 1000.0F
  // *
  //  60.0F;
  return rate;
}