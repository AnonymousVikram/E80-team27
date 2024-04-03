#ifndef __TIMINGOFFSETS_H__
#define __TIMINGOFFSETS_H__

// all times are recorded in units of ms
// the offsets define when tasks happen within a loop period
#define LOOP_PERIOD 10
#define PRINTER_LOOP_OFFSET 0
#define IMU_LOOP_OFFSET 1
#define GPS_LOOP_OFFSET 2
#define ADC_LOOP_OFFSET 3
#define ERROR_FLAG_LOOP_OFFSET 4
#define GYRO_LOOP_OFFSET 5
#define PRESSURE_SENSOR_LOOP_OFFSET 6
#define LED_LOOP_OFFSET 7
#define LOGGER_LOOP_OFFSET 8

#endif
