#include "FreqReader.h"
#include "FloatFormatter.h"
#include "Pinouts.h"
#include "SensorGyro.h"
#include "TimingOffsets.h"

extern FloatFormatter formatter;
extern SensorGyro gyro;

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
      // Serial.println(velocity);
    }
    isHigh = curVal;
    lastTime = currentTime;
  }
}

void FreqReader::freqToVel(void) {
  // convert frequency to velocity
  velocity = frequency * velCalSlope + velCalOffset;
  if (abs(velocity) < 0.1) {
    if (gyro.state.accelX > 1) {
      fwd = 1;
    } else if (gyro.state.accelX < -1) {
      fwd = -1;
    }
  }

  velocity = velocity * fwd;
}

std::string FreqReader::logData(void) {
  return (formatter.format(frequency) + "," + formatter.format(velocity) + ",");
}