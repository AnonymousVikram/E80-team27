#include "Logger.h"
#include "Printer.h"

extern Printer printer;

Logger::Logger(void) {
  writeTime = millis();
  pinMode(10, OUTPUT);
  headers << "Time, ";
}

Logger::~Logger(void) { close(); }

void Logger::close(void) {
  file.flush();
  file.close();
}

void Logger::include(std::string header, bool endline) {
  if (endline)
    headers << header << "\n";
  else
    headers << header << ",";
}

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
  file.write(header.c_str());
  file.write("\n");
}

void Logger::beginData(void) {
  writeTime = writeTime - millis();
  // begin the data string
  // file.write(millis() << ", ");
  file.write(String(millis() + ",").c_str());
}

void Logger::writeData(std::string inData, bool endLine) {
  // write the data to the buffer
  // file.write(inData + ", ");
  if (endLine) {
    inData.pop_back();
    file.write((inData + "\n").c_str());
  } else
    file.write((inData + ",").c_str());
}

void Logger::endData(void) {
  writeTime = writeTime + millis();
  // get rid of the last comma
  // data >> std::ws;
  // end the data string

  // increment the data count
  dataCount++;

  // if we have written 1000 data points, flush the buffer
  // if (dataCount >= 100) {
  //   int time = millis();
  //   // file.print(data.str().c_str());
  //   file.flush();
  //   // printer.printValue(12, "Time taken to flush: " + String(millis() -
  //   // time));
  //   file.write(("Time taken to flush: " + String(millis() - time)).c_str());
  //   file.write(("Average time per write: " + String(writeTime / 1000) + "
  //   ms")
  //                  .c_str());
  //   dataCount = 0;
  //   data.str("");
  //   writeTime = 0;

  //   flushCount++;
  // }

  // if (dataCount >= 1000) {
  //   int time = millis();
  //   file.print(data.str().c_str());
  //   file.flush();
  //   time = millis() - time;
  //   printer.printValue(12, "Time taken to flush: " + String(time));
  //   file.println("Time taken to flush: " + String(time));
  //   file.flush();
  //   dataCount = 0;
  // }
}

String Logger::printState(void) {
  // print the state of the logger
  String state = "Logger State: ";
  state += file.name();
  state += ", ";
  state += "Flush Count: " + String(flushCount);
  return state;
}