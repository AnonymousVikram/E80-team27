#include "Logger.h"

Logger::Logger(void) {
  writeTime = millis();
  pinMode(10, OUTPUT);
  headers << "Time, ";
}

Logger::~Logger(void) {
  file.flush();
  file.close();
}

void Logger::include(std::string header) { headers << header << ","; }

void Logger::init(void) {
  // initialize the SD card
  Serial.print("Initializing SD card...");
  if (!SD.begin()) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  // iterate through increasing file numbers until we find one that doesn't
  // exist
  // file.open("data");

  for (int i = 0; i < 1000; i++) {
    String fileName = "data" + String(i) + ".csv";
    if (!SD.exists(fileName)) {
      file = SD.open(fileName, O_WRITE | O_CREAT);
      Serial.println("File opened");
      break;
    }
  }

  // if the file isn't open, exit
  if (!file) {
    Serial.println("error opening file");
    return;
  }

  // write the header to the file
  std::string header = headers.str();
  header.pop_back();
  file.write((header + "\n").c_str());
  file.flush();
}

void Logger::write(std::string data) {
  // write the data to the file
  // file.write(data.c_str());
  std::string time = std::to_string(millis());
  time += ",";
  writeTime = writeTime - millis();
  file.write(time.c_str());
  data.pop_back();
  file.write((data + "\n").c_str());

  dataCount++;

  writeTime = writeTime + millis();

  if (dataCount >= 100) {
    flushTime = millis();
    file.flush();
    flushTime = millis() - flushTime;
    flushCount++;
    avgWriteTime = (float)writeTime / dataCount;
    writeTime = 0;
    dataCount = 0;
    Serial.println(printState());
  }

  if (flushCount >= 10) {
    file.close();
    Serial.println("File closed");
  }
}

String Logger::printState(void) {
  // print the state of the logger
  String state = "Logger State: ";
  state += file.name();
  state += ", ";
  state += "Flush Count: " + String(flushCount);
  state += ", ";
  state += "Average Write Time: " + String(avgWriteTime);
  state += ", ";
  state += "Flush Time: " + String(flushTime);
  return state;
}