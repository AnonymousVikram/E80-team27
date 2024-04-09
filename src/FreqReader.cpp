#include "FreqReader.h"
#include "FloatFormatter.h"
#include "Pinouts.h"
#include "TimingOffsets.h"

extern FloatFormatter formatter;

FreqReader::FreqReader(void) {
  // empty constructor
}

void FreqReader::init(void) { pinMode(FREQ_READER_PIN, INPUT); }

void FreqReader::read(void) {
  // read the frequency from the sensor
  unsigned long currentTime = micros();
  bool curVal = digitalRead(FREQ_READER_PIN);
  if (curVal != isHigh) {
    if (curVal) {
      frequency = 1000000.0 / (currentTime - lastTime) / 2.0;
      freqToVel();
      Serial.println(velocity);
    }
    isHigh = curVal;
    lastTime = currentTime;
  }
}

void FreqReader::freqToVel(void) {
  // convert frequency to velocity
  velocity = frequency * velCalSlope + velCalOffset;
}

std::string FreqReader::logData(void) {
  return (formatter.format(frequency) + "," + formatter.format(velocity) + ",");
}