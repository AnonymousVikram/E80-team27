/********
Default E80 Code
Current Author:
    Vikram Krishna (vkrishna@g.hmc.edu) '26 (contributed in 2024)
Previous Contributors:
    Wilson Ives (wives@g.hmc.edu) '20 (contributed in 2018)
    Christopher McElroy (cmcelroy@g.hmc.edu) '19 (contributed in 2017)
    Josephine Wong (jowong@hmc.edu) '18 (contributed in 2016)
    Apoorva Sharma (asharma@hmc.edu) '17 (contributed in 2016)
*/

/* Libraries */

// general
#include <Arduino.h>
#include <Pinouts.h>
#include <Wire.h>

// E80-specific
#include <Logger.h>
#include <MotorDriver.h>
#include <Printer.h>
#include <SensorIMU.h>

/* Global Variables */

// period in ms of logger and printer
#define LOOP_PERIOD 100

// Motors
MotorDriver motorDriver;

// IMU
SensorIMU imu;

// Logger
Logger logger;
bool keepLogging = true;

// Printer
Printer printer;

// loop start recorder
int loopStartTime;

void setup() {
  printer.init();

  /* Initialize the Logger */
  logger.include(&imu);
  logger.include(&motorDriver);
  logger.init();

  /* Initialise the sensors */
  imu.init();

  /* Initialize motor pins */
  motorDriver.init();

  /* Keep track of time */
  printer.printMessage("Starting main loop", 10);
  loopStartTime = millis();
}

void loop() {

  int currentTime = millis() - loopStartTime;

  ///////////  Don't change code above here! ////////////////////
  // write code here to make the robot fire its motors in the sequence specified
  // in the lab manual the currentTime variable contains the number of ms since
  // the robot was turned on The motorDriver.drive function takes in 3 inputs
  // arguments motorA_power, motorB_power, motorC_power:
  //       void motorDriver.drive(int motorA_power,int motorB_power,int
  //       motorC_power);
  // the value of motorX_power can range from -100 to 100, and sets the PWM
  // applied to the motor The following example will turn on motor B for four
  // seconds between seconds 4 and 8

  if (currentTime > 60000 && currentTime < 62500) {
    // Test Motor A & C
    motorDriver.drive(100, 0, 100);
  } else if (currentTime > 62500 && currentTime < 65000) {
    motorDriver.drive(-100, 0, -100);
  } else if (currentTime > 65000 && currentTime < 67500) {
    // Test Motor B
    motorDriver.drive(0, 100, 0);
  } else if (currentTime > 67500 && currentTime < 70000) {
    motorDriver.drive(0, -100, 0);
  } else if (currentTime > 70000 && currentTime < 72500) {
    // Bring the robot back up
    motorDriver.drive(0, 100, 0);
  } else {
    motorDriver.drive(0, 0, 0);
  }

  // DONT CHANGE CODE BELOW THIS LINE
  // --------------------------------------------------------------------------

  if (currentTime - printer.lastExecutionTime > LOOP_PERIOD) {
    printer.lastExecutionTime = currentTime;
    printer.printValue(0, imu.printAccels());
    printer.printValue(1, imu.printRollPitchHeading());
    printer.printValue(2, motorDriver.printState());
    printer.printToSerial(); // To stop printing, just comment this line out
  }

  if (currentTime - imu.lastExecutionTime > LOOP_PERIOD) {
    imu.lastExecutionTime = currentTime;
    imu.read(); // this is a sequence of blocking I2C read calls
  }

  if (currentTime - logger.lastExecutionTime > LOOP_PERIOD &&
      logger.keepLogging) {
    logger.lastExecutionTime = currentTime;
    logger.log();
  }
}
