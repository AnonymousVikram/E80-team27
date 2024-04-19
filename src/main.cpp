/********
Default E80 Code
Authors:
    Vikram Krishna (vkrishna@hmc.edu) '26 (contributed in 2024)
    Wilson Ives (wives@g.hmc.edu) '20 (contributed in 2018)
    Christopher McElroy (cmcelroy@g.hmc.edu) '19 (contributed in 2017)
    Josephine Wong (jowong@hmc.edu) '18 (contributed in 2016)
    Apoorva Sharma (asharma@hmc.edu) '17 (contributed in 2016)
*/

// #include <ADCSampler.h>
#include <Arduino.h>
// #include <ErrorFlagSampler.h>
#include "FloatFormatter.h"
#include "FreqReader.h"
#include "Logger.h"
#include "MotorDriver.h"
#include "Pinouts.h"
#include "PressureSensor.h"
#include "Printer.h"
#include "RobotControl.h"
#include "SensorGyro.h"
#include "ServoDriver.h"
#include "StateEstimator.h"
#include "TimingOffsets.h"
#include <Wire.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#define UartSerial Serial1
#define DELAY 2 * 60 * 1000
// #include <GPSLockLED.h>

/////////////////////////* Global Variables *////////////////////////

MotorDriver motor_driver;
// SensorIMU imu;
Logger logger;
// Printer printer;
SensorGyro gyro;
// PressureSensor pSensor;
// ServoDriver rudder;
// StateEstimator stateEstimator;
// RobotControl robotControl;
FreqReader freqReader;
FloatFormatter formatter;

// loop start recorder
int loopStartTime;
int currentTime;

// waypoints
float waypoints[NUMWAYPOINTS][3] = {{0, 0, 1}, {1, 0, 1}, {1, 0, 0}};

////////////////////////* Setup *////////////////////////////////

void setup() {
  delay(DELAY);

  UartSerial.begin(9600);
  logger.include(gyro.headers);
  logger.include(freqReader.headers);
  // logger.include(pSensor.headers);
  // logger.include(stateEstimator.headers);
  // logger.include(motor_driver.headers);
  // logger.include(rudder.headers);
  // logger.include(robotControl.headers);
  logger.init();

  // printer.init();
  // motor_driver.init();
  gyro.init();
  // pSensor.init();
  // rudder.init();
  // stateEstimator.init();
  // robotControl.init(3, waypoints);

  // printer.printMessage("Starting main loop", 1);
  loopStartTime = millis();
  // printer.lastExecutionTime = loopStartTime - LOOP_PERIOD + PRINTER_LOOP_OFFSET;
  gyro.lastExecutionTime = loopStartTime - LOOP_PERIOD + GYRO_LOOP_OFFSET;
  // pSensor.lastExecutionTime =
      // loopStartTime - LOOP_PERIOD + PRESSURE_SENSOR_LOOP_OFFSET;
  // logger.lastExecutionTime = loopStartTime - LOOP_PERIOD + LOGGER_LOOP_OFFSET;
  freqReader.lastExecutionTime =
      loopStartTime - LOOP_PERIOD + FREQ_READER_LOOP_OFFSET;
}

//////////////////////////////* Loop */////////////////////////
// int counter = 0;
void loop() {

  currentTime = millis();

  if (currentTime - gyro.lastExecutionTime > GYRO_PERIOD) {
    gyro.lastExecutionTime = currentTime;
    gyro.read();
  }

  // if (currentTime - pSensor.lastExecutionTime > LOOP_PERIOD) {
  //   pSensor.lastExecutionTime = currentTime;
  //   pSensor.read();
  // }

  if (currentTime - freqReader.lastExecutionTime > LOOP_PERIOD) {
    freqReader.lastExecutionTime = currentTime;
    freqReader.read();
  }

  if (currentTime - logger.lastExecutionTime > LOGGER_PERIOD) {
    logger.lastExecutionTime = currentTime;
    std::string data = "";
    data += gyro.logData();
    data += freqReader.logData();
    // data += pSensor.logData();
    // data += stateEstimator.logData();
    // data += motor_driver.logData();
    // data += rudder.logData();
    // data += robotControl.logData();
    logger.write(data);
  }

  // no matter what the state, update the robot control
  // robotControl.update();
}
