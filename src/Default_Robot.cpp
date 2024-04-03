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
ErrorFlagSampler ef;
SensorIMU imu;
Logger logger;
Printer printer;
GPSLockLED led;
SensorGyro gyro;
PressureSensor pSensor;

// loop start recorder
int loopStartTime;
int currentTime;
int current_way_point = 0;
volatile bool EF_States[NUM_FLAGS] = {1, 1, 1};

// GPS Waypoints
const int number_of_waypoints = 6;
const int waypoint_dimensions =
    3; // waypoints are set to have two pieces of information, x then y.
double waypoints[] = {
    0,  0, 5, 10, 0, 5, 10, 0, 0,
    10, 0, 5, 0,  0, 5, 0,  0, 0}; // listed as x0,y0,x1,y1, ... etc.

////////////////////////* Setup *////////////////////////////////

void setup() {

  // logger.include(&imu);
  // logger.include(&gps);
  // logger.include(&motor_driver);
  // logger.include(&adc);
  // logger.include(&ef);
  logger.include(&gyro);
  logger.include(&pSensor);
  logger.init();

  printer.init();
  // ef.init();
  // imu.init();
  UartSerial.begin(9600);
  // gps.init(&GPS);
  // motor_driver.init();
  // led.init();
  gyro.init();

  printer.printMessage("Starting main loop", 1);
  loopStartTime = millis();
  printer.lastExecutionTime = loopStartTime - LOOP_PERIOD + PRINTER_LOOP_OFFSET;
  // imu.lastExecutionTime             = loopStartTime - LOOP_PERIOD +
  // IMU_LOOP_OFFSET; gps.lastExecutionTime             = loopStartTime -
  // LOOP_PERIOD + GPS_LOOP_OFFSET; adc.lastExecutionTime             =
  // loopStartTime - LOOP_PERIOD + ADC_LOOP_OFFSET; ef.lastExecutionTime =
  // loopStartTime - LOOP_PERIOD + ERROR_FLAG_LOOP_OFFSET;
  gyro.lastExecutionTime = loopStartTime - LOOP_PERIOD + GYRO_LOOP_OFFSET;
  pSensor.lastExecutionTime =
      loopStartTime - LOOP_PERIOD + PRESSURE_SENSOR_LOOP_OFFSET;
  logger.lastExecutionTime = loopStartTime - LOOP_PERIOD + LOGGER_LOOP_OFFSET;
}

void EFA_Detected(void);
void EFB_Detected(void);
void EFC_Detected(void);

//////////////////////////////* Loop */////////////////////////

void loop() {
  currentTime = millis();

  if (currentTime - printer.lastExecutionTime > LOOP_PERIOD) {
    printer.lastExecutionTime = currentTime;
    // printer.printValue(0,adc.printSample());
    // printer.printValue(1,ef.printStates());
    printer.printValue(0, logger.printState());
    // printer.printValue(3,gps.printState());
    // printer.printValue(7,motor_driver.printState());
    // printer.printValue(8,imu.printRollPitchHeading());
    // printer.printValue(9,imu.printAccels());
    printer.printValue(1, gyro.printRollPitchYaw());
    printer.printValue(2, gyro.printAccels());
    printer.printValue(3, gyro.printOrientation());
    printer.printValue(4, pSensor.printPressure());
    printer.printToSerial(); // To stop printing, just comment this line out
  }

  // if ( currentTime-surface_control.lastExecutionTime > LOOP_PERIOD ) {
  //   motor_driver.drive(surface_control.uL,surface_control.uR,0);
  // }

  // if ( currentTime-ef.lastExecutionTime > LOOP_PERIOD ) {
  //   ef.lastExecutionTime = currentTime;
  //   attachInterrupt(digitalPinToInterrupt(ERROR_FLAG_A), EFA_Detected, LOW);
  //   attachInterrupt(digitalPinToInterrupt(ERROR_FLAG_B), EFB_Detected, LOW);
  //   attachInterrupt(digitalPinToInterrupt(ERROR_FLAG_C), EFC_Detected, LOW);
  //   delay(5);
  //   detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_A));
  //   detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_B));
  //   detachInterrupt(digitalPinToInterrupt(ERROR_FLAG_C));
  //   ef.updateStates(EF_States[0],EF_States[1],EF_States[2]);
  //   EF_States[0] = 1;
  //   EF_States[1] = 1;
  //   EF_States[2] = 1;
  // }

  // uses the ButtonSampler library to read a button -- use this as a template
  // for new libraries! if ( currentTime-button_sampler.lastExecutionTime >
  // LOOP_PERIOD ) {
  //   button_sampler.lastExecutionTime = currentTime;
  //   button_sampler.updateState();
  // }

  // if ( currentTime-imu.lastExecutionTime > LOOP_PERIOD ) {
  //   imu.lastExecutionTime = currentTime;
  //   imu.read();     // blocking I2C calls
  // }

  if (currentTime - gyro.lastExecutionTime > LOOP_PERIOD) {
    gyro.lastExecutionTime = currentTime;
    gyro.read(); // blocking I2C calls
  }

  if (currentTime - pSensor.lastExecutionTime > LOOP_PERIOD) {
    pSensor.lastExecutionTime = currentTime;
    pSensor.read(); // blocking I2C calls
  }

  // gps.read(&GPS); // blocking UART calls, need to check for UART every cycle

  // if ( currentTime-led.lastExecutionTime > LOOP_PERIOD ) {
  //   led.lastExecutionTime = currentTime;
  //   led.flashLED(&gps.state);
  // }

  if (currentTime - logger.lastExecutionTime > LOOP_PERIOD &&
      logger.keepLogging) {
    logger.lastExecutionTime = currentTime;
    logger.log();
  }
}

void EFA_Detected(void) { EF_States[0] = 0; }

void EFB_Detected(void) { EF_States[1] = 0; }

void EFC_Detected(void) { EF_States[2] = 0; }
