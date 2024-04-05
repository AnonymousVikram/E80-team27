#ifndef __TIMINGOFFSETS_H__
#define __TIMINGOFFSETS_H__

// all times are recorded in units of ms
// the offsets define when tasks happen within a loop period
#define LOOP_PERIOD 1
#define PRINTER_LOOP_OFFSET 0
#define IMU_LOOP_OFFSET 0.1
#define GPS_LOOP_OFFSET 0.2
#define ADC_LOOP_OFFSET 0.4
#define ERROR_FLAG_LOOP_OFFSET 0.45
#define GYRO_LOOP_OFFSET 0.5
#define PRESSURE_SENSOR_LOOP_OFFSET 0.6
#define LED_LOOP_OFFSET 0.7
#define LOGGER_LOOP_OFFSET 0.8

#endif
