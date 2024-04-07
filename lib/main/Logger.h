#ifndef LOGGER_H_INCLUDED
#define LOGGER_H_INCLUDED

#include <Arduino.h>
#include <SD.h>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string>


class Logger {
public:
  Logger(void);

  ~Logger(void);

  void include(std::string headers);

  // Defines the filestream and writes headers
  void init(void);

  // Writes data to the filestream
  void writeData(std::string data);

  String printState(void);

  int lastExecutionTime = -1;

private:
  // stringstream to store headers, initalized with "Time, "
  std::stringstream dataHeaders;

  // fstream to write data to
  std::ofstream fileStream;

  // counter for number of data points logged
  int dataCount = 0;
};
#endif