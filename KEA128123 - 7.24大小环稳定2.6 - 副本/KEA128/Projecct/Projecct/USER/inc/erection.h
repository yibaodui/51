#ifndef _ERECTION_H
#define _ERECTION_H

#include "headfile.h"

extern void erec_speed(int speed_set);
extern int zhili_speed(float angle,float gyro);
extern void real_speed(void);
extern void speed_init(void);
extern float Smooth_speed(uint8 period);
extern void speedoutput(int pwm_count,int direction_output);
extern int angle_core_series(float gyro);
extern void angle_shell_series(float angle);
extern void stop_car(void);
extern int16 tyre_raised_deal(int16 *real_speed,int16 *real_speed_old);
extern void car_speed_limit(void);


#endif