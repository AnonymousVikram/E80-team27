#ifndef __TIMINGOFFSETS_H__
#define __TIMINGOFFSETS_H__

// all times are recorded in units of ms
// the offsets define when tasks happen within a loop period
#define LOOP_PERIOD 0.25
#define PRINTER_LOOP_OFFSET 0
// #define IMU_LOOP_OFFSET 0.05
// #define GPS_LOOP_OFFSET 0.2
// #define ADC_LOOP_OFFSET 0.1
// #define ERROR_FLAG_LOOP_OFFSET 0.45
#define GYRO_LOOP_OFFSET 0.01
#define PRESSURE_SENSOR_LOOP_OFFSET 0.02
#define FLOW_SENSOR_LOOP_OFFSET 0.03
// #define LED_LOOP_OFFSET 0.25
#define FREQ_READER_LOOP_OFFSET 0.04
#define LOGGER_LOOP_OFFSET 0.05

#endif
