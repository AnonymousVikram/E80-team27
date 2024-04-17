#ifndef __MOTOR_DRIVER_H__
#define __MOTOR_DRIVER_H__

#define NUM_MOTORS 5

#define DIRECTION_PIN 0
#define SPEED_PIN 1

// the minimum PWM amount that causes the motors to actually spin
#define MOTOR_DEADZONE 34
#include "Pinouts.h"
#include <Arduino.h>
#include <string>

/*
 * MotorDriver handles the raw signals that are needed to drive the robot's
 * motors.
 */
class MotorDriver {
public:
  MotorDriver();

  // initializes motor command state to zero
  void init(void);

  // applies the stored motor command state values to the pins
  void apply(void);

  // prints current motor command state as string
  String printState(void);

  // helper function that sets motor command state and then applies it
  void drive(int motorA_power, int motorB_power, int motorC_power,
             int motorD_power, int motorE_power);

  // Range from -255 to +255 for full reverse or full forward
  int motorValues[NUM_MOTORS];

  // from DataSource
  std::string logData(void);

  std::string headers = "Motor A,Motor B,Motor C,Motor D,Motor E";

private:
  // pins for the motors
  int motorPins[NUM_MOTORS][2] = {{MOTOR_A_DIRECTION, MOTOR_A_SPEED},
                                  {MOTOR_B_DIRECTION, MOTOR_B_SPEED},
                                  {MOTOR_C_DIRECTION, MOTOR_C_SPEED},
                                  {MOTOR_D_DIRECTION, MOTOR_D_SPEED},
                                  {MOTOR_E_DIRECTION, MOTOR_E_SPEED}};

  // pwm data
  unsigned int pwmValues[NUM_MOTORS]; // absolute value
  bool pwmDir[NUM_MOTORS];            // direction
};

#endif
