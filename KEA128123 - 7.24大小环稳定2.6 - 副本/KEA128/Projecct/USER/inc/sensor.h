#ifndef _SENSOR_H
#define _SENSOR_H

#include "headfile.h"

#define ADC_CLS ADC_8bit

extern int sensor_filter_average(int *buf);
extern void sensor_value_get(void);
extern void sensor_init(void);
extern int sensor_filter(int *buf);
extern void sensor_value_get(void);
extern float offset_get(void);
extern int Direction_output(uint8 period);
extern float cal_deviation(uint16 * sensor_value);
extern float Direction_get(float turn_offset_value);
extern uint8 roundabout_deal(int16 buf,int16 buf_left,int16 buf_right,int16 left,int16 right);
extern int turn_core_series(float offset_value);

#define LOSETHRESHOLD 500




#endif