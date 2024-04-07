#include "Logger.h"
#include "Printer.h"

extern Printer printer;

Logger::Logger(void) {}

void Logger::include(std::string headers) {
  dataHeaders << headers;
  dataHeaders << ", ";
}

Logger::~Logger(void) { fileStream.close(); }

void Logger::init(void) {
  // initialize the SD card
  Serial.print("Initializing SD card...");
  if (!SD.begin(53)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  // iterate through increasing file numbers until we find one that doesn't
  // exist
  for (int i = 0; i < 1000; i++) {
    String filename = "dataRun" + String(i) + ".csv";
    if (!SD.exists(filename)) {
      fileStream.open(std::string(filename.c_str()), std::ofstream::app);
    }
  }

  // if the file isn't open, exit
  if (!fileStream.is_open()) {
    Serial.println("error opening file");
    return;
  }

  // write the header to the file
  fileStream << "Time, ";
  fileStream << dataHeaders.str() << std::endl;
}

void Logger::writeData(std::string data) {
  fileStream << data;
  fileStream << "\n";
  dataCount++;

  // if we've written 1000 data points, flush the buffer
  if (dataCount % 1000 == 0) {

    // check the time taken to write the data
    int curTime = millis();
    fileStream.flush();
    lastExecutionTime = millis() - curTime;

    // print the time taken to write the data
    printer.printValue(
        11, ("Time to write 1000 data points: " + lastExecutionTime));
  }
}

String Logger::printState(void) {
  String state = "Logger State: ";
  state += fileStream.rdstate();
  return state;
}