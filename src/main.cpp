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
#define DELAY 2.5 * 60 * 1000
// #include <GPSLockLED.h>

/////////////////////////* Global Variables *////////////////////////

MotorDriver motor_driver;
// SensorIMU imu;
Logger logger;
Printer printer;
SensorGyro gyro;
PressureSensor pSensor;
ServoDriver rudder;
StateEstimator stateEstimator;
RobotControl robotControl;
FreqReader freqReader;
FloatFormatter formatter;

// loop start recorder
int loopStartTime;
int currentTime;

// waypoints
float waypoints[NUMWAYPOINTS][3] = {{0, 0, 1}, {1, 0, 1}, {1, 0, 0}};

////////////////////////* Setup *////////////////////////////////
long startTime = 0;
void setup() {
  delay(DELAY);

  logger.include(gyro.headers);
  logger.include(freqReader.headers);
  logger.include(pSensor.headers);
  logger.include(stateEstimator.headers);
  logger.include(motor_driver.headers);
  logger.include(rudder.headers);
  logger.include(robotControl.headers);
  logger.init();

  printer.init();
  UartSerial.begin(9600);
  motor_driver.init();
  gyro.init();
  pSensor.init();
  rudder.init();
  stateEstimator.init();
  robotControl.init(3, waypoints);

  printer.printMessage("Starting main loop", 1);
  loopStartTime = millis();
  printer.lastExecutionTime = loopStartTime - LOOP_PERIOD + PRINTER_LOOP_OFFSET;
  gyro.lastExecutionTime = loopStartTime - LOOP_PERIOD + GYRO_LOOP_OFFSET;
  pSensor.lastExecutionTime =
      loopStartTime - LOOP_PERIOD + PRESSURE_SENSOR_LOOP_OFFSET;
  logger.lastExecutionTime = loopStartTime - LOOP_PERIOD + LOGGER_LOOP_OFFSET;
  freqReader.lastExecutionTime =
      loopStartTime - LOOP_PERIOD + FREQ_READER_LOOP_OFFSET;
  
  startTime = millis();
}

//////////////////////////////* Loop */////////////////////////
// int counter = 0;
void loop() {

  currentTime = millis();

  if (currentTime > startTime + 3*60*1000) {
    robotControl.giveUp = true;
  }

  // if (currentTime - printer.lastExecutionTime > LOOP_PERIOD) {
  //   printer.lastExecutionTime = currentTime;
  //   // printer.printValue(0, adc.printSample());
  //   // printer.printValue(1, ef.printStates());
  //   printer.printValue(0, logger.printState());
  //   // printer.printValue(1, gps.printState());
  //   printer.printValue(1, motor_driver.printState());
  //   // printer.printValue(2, imu.printRollPitchHeading());
  //   // printer.printValue(3, imu.printAccels());
  //   printer.printValue(2, gyro.printRollPitchYaw());
  //   printer.printValue(3, gyro.printAccels());
  //   printer.printValue(4, gyro.printOrientation());
  //   printer.printValue(5, pSensor.printPressure());
  //   // printer.printValue(6, flow.printFlow());
  //   printer.printValue(6, freqReader.velocity);
  //   printer.printValue(7, rudder.printState());
  //   printer.printValue(8, stateEstimator.printState());
  //   printer.printValue(9, robotControl.printString());
  //   printer.printValue(10, robotControl.printWaypoint());
  //   printer.printToSerial(); // To stop printing, just comment this line
  // }

  if (currentTime - gyro.lastExecutionTime > GYRO_PERIOD) {
    gyro.lastExecutionTime = currentTime;
    gyro.read();
  }

  if (currentTime - pSensor.lastExecutionTime > LOOP_PERIOD) {
    pSensor.lastExecutionTime = currentTime;
    pSensor.read();
  }

  if (currentTime - freqReader.lastExecutionTime > LOOP_PERIOD) {
    freqReader.lastExecutionTime = currentTime;
    freqReader.read();
  }

  if (currentTime - logger.lastExecutionTime > LOGGER_PERIOD) {
    logger.lastExecutionTime = currentTime;
    std::string data = "";
    data += gyro.logData();
    data += freqReader.logData();
    data += pSensor.logData();
    data += stateEstimator.logData();
    data += motor_driver.logData();
    data += rudder.logData();
    data += robotControl.logData();
    logger.write(data);
  }

  // no matter what the state, update the robot control
  robotControl.update();
}
