#ifndef _ANGLE_H
#define _ANGLE_H

#include "headfile.h"

extern float gy_angle(void);
extern float accel_angle(void);
extern float Complement_Filter(void);
extern float angleFilter(void);
extern void original_angle(void);
extern float Kalman_Filter(float Accel,float Gyro);

#endif