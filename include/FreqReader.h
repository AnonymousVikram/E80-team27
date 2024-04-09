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

  std::string headers = "Frequency Reader Frequency [Hz]";

  std::string logData(void);

private:
  bool isHigh = false;
  unsigned long lastTime = 0;

  // Latest reported flow data is stored here
  float frequency = 0;
};
#endif