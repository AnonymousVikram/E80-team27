#ifndef LOGGER_H_INCLUDED
#define LOGGER_H_INCLUDED

#include <Arduino.h>
#include <SD.h>
#include <sstream>
#include <string>

class Logger {
public:
  Logger(void);

  ~Logger(void);

  void include(std::string header, bool endLine = false);

  void init(void);

  void beginData(void);

  void writeData(std::string data, bool endLine = false);

  void endData(void);

  String printState(void);

  int lastExecutionTime = -1;

  int flushCount = 0;

  void close(void);

  int writeTime = 0;

private:
  File file;

  std::stringstream headers;
  std::stringstream data;

  int dataCount = 0;
};

#endif