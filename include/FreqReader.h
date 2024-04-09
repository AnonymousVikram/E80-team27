#ifndef FREQ_READER_H_INCLUDED
#define FREQ_READER_H_INCLUDED

#include <Arduino.h>
#include <Wire.h>
#include <string>

class FreqReader {
public:
  FreqReader(void);

  // Starts the connection to the sensor
  void init(void);

  // Reads data from the sensor
  void read(void);

  int lastExecutionTime = -1;

  std::string headers =
      "Frequency Reader Frequency [Hz], Calibrated Velocity [m/s]";

  std::string logData(void);
  float velocity = 0;

private:
  void freqToVel(void);
  bool isHigh = false;
  unsigned long lastTime = 0;

  // Latest reported flow data is stored here
  float frequency = 0;
  float velCalSlope = 1.0F;
  float velCalOffset = 0.0F;
};
#endif