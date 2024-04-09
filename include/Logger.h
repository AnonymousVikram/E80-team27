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

  void include(std::string header);

  void init(void);

  String printState(void);

  int lastExecutionTime = -1;

  int flushCount = 0;

  void close(void);

  int writeTime = 0;
  int avgWriteTime = 0;
  int flushTime = 0;

  void write(std::string data);

private:
  bool writing = false;
  File file;

  std::stringstream headers;

  int dataCount = 0;
};

#endif