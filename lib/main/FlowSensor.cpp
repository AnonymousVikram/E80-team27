#include "FlowSensor.h"
#include "Pinouts.h"
#include "Printer.h"
extern Printer printer;

FlowSensor::FlowSensor(void) : DataSource("velocity", "float") {
  velocity = 0.0F;
}

void FlowSensor::init(void) {
  pinMode(FLOW_SENSOR_PIN, INPUT);
  Serial.print("Initializing Flow Sensor... ");
  velocity = 0.0F;
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

size_t FlowSensor::writeDataBytes(unsigned char *buffer, size_t idx) {
  float *data_slot = (float *)&buffer[idx];
  data_slot[0] = velocity;
  return idx + sizeof(float);
}

float FlowSensor::Lmin_to_mps(float rate) {
  return rate * flow_cal_slope +
         flow_cal_offset / pipe_cs_area / 1000.0F * 60.0F;
}