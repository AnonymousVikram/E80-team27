/********
Default E80 Code
Authors:
    Vikram Krishna (vkrishna@hmc.edu) '26 (contributed in 2024)
    Wilson Ives (wives@g.hmc.edu) '20 (contributed in 2018)
    Christopher McElroy (cmcelroy@g.hmc.edu) '19 (contributed in 2017)
    Josephine Wong (jowong@hmc.edu) '18 (contributed in 2016)
    Apoorva Sharma (asharma@hmc.edu) '17 (contributed in 2016)
*/

#include <ADCSampler.h>
#include <Arduino.h>
#include <ErrorFlagSampler.h>
#include <Logger.h>
#include <MotorDriver.h>
#include <Pinouts.h>
#include <PressureSensor.h>
#include <Printer.h>
#include <SensorGPS.h>
#include <SensorIMU.h>
#include <ServoDriver.h>
#include <TimingOffsets.h>
#include <Wire.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#define UartSerial Serial1
#define DELAY 0
#include <GPSLockLED.h>
#include <SensorGyro.h>

/////////////////////////* Global Variables *////////////////////////

MotorDriver motor_driver;
SensorGPS gps;
Adafruit_GPS GPS(&UartSerial);
ADCSampler adc;
SensorIMU imu;
Logger logger;
Printer printer;
GPSLockLED led;
SensorGyro gyro;
PressureSensor pSensor;
ServoDriver rudder;

// loop start recorder
int loopStartTime;
int currentTime;
int current_way_point = 0;

////////////////////////* Setup *////////////////////////////////

void setup() {

  logger.include(&imu);
  logger.include(&gps);
  logger.include(&motor_driver);
  // logger.include(&adc);
  // logger.include(&ef);
  logger.include(&gyro);
  logger.include(&pSensor);
  logger.init();

  printer.init();
  // ef.init();
  imu.init();
  UartSerial.begin(9600);
  gps.init(&GPS);
  motor_driver.init();
  led.init();
  gyro.init();
  pSensor.init();
  rudder.init();

  printer.printMessage("Starting main loop", 1);
  loopStartTime = millis();
  printer.lastExecutionTime = loopStartTime - LOOP_PERIOD + PRINTER_LOOP_OFFSET;
  imu.lastExecutionTime = loopStartTime - LOOP_PERIOD + IMU_LOOP_OFFSET;
  gps.lastExecutionTime = loopStartTime - LOOP_PERIOD + GPS_LOOP_OFFSET;
  // adc.lastExecutionTime = loopStartTime - LOOP_PERIOD + ADC_LOOP_OFFSET;
  // ef.lastExecutionTime = loopStartTime - LOOP_PERIOD +
  // ERROR_FLAG_LOOP_OFFSET;
  gyro.lastExecutionTime = loopStartTime - LOOP_PERIOD + GYRO_LOOP_OFFSET;
  pSensor.lastExecutionTime =
      loopStartTime - LOOP_PERIOD + PRESSURE_SENSOR_LOOP_OFFSET;
  logger.lastExecutionTime = loopStartTime - LOOP_PERIOD + LOGGER_LOOP_OFFSET;
}

//////////////////////////////* Loop */////////////////////////

void loop() {
  currentTime = millis();

  if (currentTime - printer.lastExecutionTime > LOOP_PERIOD) {
    printer.lastExecutionTime = currentTime;
    // printer.printValue(0,adc.printSample());
    // printer.printValue(1,ef.printStates());
    printer.printValue(0, logger.printState());
    printer.printValue(1, gps.printState());
    printer.printValue(2, motor_driver.printState());
    printer.printValue(3, imu.printRollPitchHeading());
    printer.printValue(4, imu.printAccels());
    printer.printValue(5, gyro.printRollPitchYaw());
    printer.printValue(6, gyro.printAccels());
    printer.printValue(7, gyro.printOrientation());
    printer.printValue(8, pSensor.printPressure());
    printer.printValue(9, rudder.printState());
    printer.printToSerial(); // To stop printing, just comment this line out
  }

  if (currentTime - imu.lastExecutionTime > LOOP_PERIOD) {
    imu.lastExecutionTime = currentTime;
    imu.read(); // blocking I2C calls
  }

  if (currentTime - gyro.lastExecutionTime > LOOP_PERIOD) {
    gyro.lastExecutionTime = currentTime;
    gyro.read(); // blocking I2C calls
  }

  if (currentTime - pSensor.lastExecutionTime > LOOP_PERIOD) {
    pSensor.lastExecutionTime = currentTime;
    pSensor.read(); // blocking I2C calls
  }

  gps.read(&GPS); // blocking UART calls, need to check for UART every cycle

  if (currentTime - led.lastExecutionTime > LOOP_PERIOD) {
    led.lastExecutionTime = currentTime;
    led.flashLED(&gps.state);
  }

  if (currentTime - logger.lastExecutionTime > LOOP_PERIOD &&
      logger.keepLogging) {
    logger.lastExecutionTime = currentTime;
    logger.log();
  }
}
