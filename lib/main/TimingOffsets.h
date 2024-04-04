#ifndef __TIMINGOFFSETS_H__
#define __TIMINGOFFSETS_H__

// all times are recorded in units of ms
// the offsets define when tasks happen within a loop period
#define LOOP_PERIOD 2
#define PRINTER_LOOP_OFFSET 0
#define IMU_LOOP_OFFSET 0.2
#define GPS_LOOP_OFFSET 0.4
#define ADC_LOOP_OFFSET 0.8
#define ERROR_FLAG_LOOP_OFFSET 0.9
#define GYRO_LOOP_OFFSET 1.0
#define PRESSURE_SENSOR_LOOP_OFFSET 1.2
#define LED_LOOP_OFFSET 1.4
#define LOGGER_LOOP_OFFSET 1.6

#endif
