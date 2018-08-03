#ifndef _SENSOR_H
#define _SENSOR_H

#include "headfile.h"
#define KP 0
#define KI 1
#define KD 2
#define KT 3
#define KB 4
#define KF 5
#define ADC_CLS ADC_8bit
typedef struct PID
{
	float SumError;	//误差累计	
	int32 LastError;	//Error[-1]
	int32 PrevError;	//Error[-2]	
	int32 LastData;	//Speed[-1]
} PID;
extern int32 PID_Realize(PID *sptr, float *PID, int32 NowData, int32 Point);
extern PID MOTOR_PID, Angle_PID, Ang_Vel_PID, Direct_PID, Turn_PID, Distance_PID;	//定义舵机和电机的PID参数结构体
extern int sensor_filter_average(int *buf);
extern void sensor_value_get(void);
extern void sensor_init(void);
extern int sensor_filter(int *buf);
extern void sensor_value_get(void);
extern float offset_get(void);
extern int Direction_output(uint8 period);
extern float cal_deviation(uint16 * sensor_value);
extern float Direction_get(int32 turn_offset_value);
extern int32 PID_Realize(PID *sptr, float *PID, int32 NowData, int32 Point);
extern int turn_core_series(float offset_value);
extern uint8 roundabout_deal(int16 buf,int16 buf_left,int16 buf_right,int16 left,int16 right,uint8 howflag);
extern float regression(int startline,int endline,float X[10]); //最小二乘法计算函数 注意数组范围;
extern uint8 roundabout_deal_two(int16 buf,int16 buf_left,int16 buf_right,int16 left,int16 right);


#define LOSETHRESHOLD 500




#endif