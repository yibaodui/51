#include "angle.h"
#include "math.h"
#include "SEEKFREE_MPU6050.h"
#include "arm_math.h"
#include "erection.h"

#define gyro_zero 1
//#define dt 0.005  //0.012968
#define K1 0.0075
#define K 0.93 

float acc_angle;
float gyro_angle,gy0_angle;
float Complement_angle;
float dt=0.005;
int16 mpu_gyro_y_old;
//float angle, angle_dot; 	
//float Q_angle=0.001;// 过程噪声的协方差
//float Q_gyro=0.003;//0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
//float R_angle=0.5;// 测量噪声的协方差 既测量偏差
////                 
//char  C_0 = 1;
//float Q_bias, Angle_err;
//float PCt_0, PCt_1, E;
//float K_0, K_1, t_0, t_1;
//float Pdot[4] ={0,0,0,0};
//float PP[2][2] = { { 1, 0 },{ 0, 1 } };


/**********************************
 **函数名:accel_angle
 **作用 :计算加速度计角度
 **返回值类型:float
 **作者:王鹏辉
 **时间:2018/1/22
************************************/ 

float accel_angle(void)//加速度计量范围 2g 16384LSB/s  z0/16464
{
    Get_AccData();
    acc_angle = atanf(1.0 * mpu_acc_z / mpu_acc_x) * 180 / PI;
    //printf("%d\n",(int)mpu_acc_x);
    return acc_angle;
//      get_mma8451();    //读取加速度计值
//      acc_angle = (float)atan2f(1.0 * acc_z,1.0 * acc_y) * 180 / PI;    //反三角函数计算当前加速度计角度
//      return acc_angle;  
}


/**********************************
 **函数名:gy_angle
 **作用 :读取陀螺仪原始数据
 **返回值类型:float
 **作者:王鹏辉
 **时间:2018/1/22
************************************/ 
float gy_angle(void)//陀螺仪计量范围 2000 16.4LSB/s
{ 
    Get_Gyro();   
    mpu_gyro_y = 0.8 * mpu_gyro_y_old + 0.2 * mpu_gyro_y;
    mpu_gyro_y_old = mpu_gyro_y;
    gyro_angle = 1.0 * (mpu_gyro_y - gyro_zero) / 16.4;
    gy0_angle = gy0_angle + gyro_angle * dt;
    gyro_angle = K*gyro_angle;  //陀螺仪值修正
    return gyro_angle; 
//      get_l3g4200d();
//      gyro_angle = 1.0*(gyro_y - gyro_zero) / 15.4;
//      gy0_angle = gy0_angle + gyro_angle * dt;
//      return gyro_angle;
}


/**********************************
 **函数名:Complement_Filter
 **作用 :互补融合滤波
 **返回值类型:float
 **作者:王鹏辉
 **时间:2018/1/22
************************************/ 
float Complement_Filter(void)
{	
     //float gyro_y,acc_z;
      //gyro_y = gy_angle();
      //acc_z = accel_angle();
      //gyro_angle = K*gyro_angle;
      Complement_angle = K1 * acc_angle+ (1-K1) * (Complement_angle + gyro_angle * dt);
      if(Complement_angle > 90) Complement_angle = 90;
      if(Complement_angle < -90) Complement_angle = -90;
      return Complement_angle;

}
//          float mid_temp;//互补融合滤波中间量																
//    float back_value;//融合后的角度与加速度静态角度差值，作为反馈量加入积分回路，让静差为0
//																											 
//    mid_temp=Complement_angle;  																	 
//    gyro_angle=K*gyro_angle;//角速度修正 																			   
//    back_value=(acc_angle-mid_temp)*3.95;//融合后的角度与加速度静态角度差值	
//    Complement_angle=Complement_angle+(gyro_angle+back_value )*dt;
//    //zhili_speed(Complement_angle,gyro_angle);
//    return  Complement_angle;




//float Kalman_Filter(float Accel,float Gyro)		
//{
//	Complement_angle+=(Gyro - Q_bias) * dt; //先验估计
//	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分
//
//	Pdot[1]=-PP[1][1];
//	Pdot[2]=-PP[1][1];
//	Pdot[3]=Q_gyro;
//	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
//	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
//	PP[1][0] += Pdot[2] * dt;
//	PP[1][1] += Pdot[3] * dt;
//		
//	Angle_err = Accel - Complement_angle;	//zk-先验估计
//	
//	PCt_0 = C_0 * PP[0][0];
//	PCt_1 = C_0 * PP[1][0];
//	
//	E = R_angle + C_0 * PCt_0;
//	
//	K_0 = PCt_0 / E;
//	K_1 = PCt_1 / E;
//	
//	t_0 = PCt_0;
//	t_1 = C_0 * PP[0][1];
//
//	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
//	PP[0][1] -= K_0 * t_1;
//	PP[1][0] -= K_1 * t_0;
//	PP[1][1] -= K_1 * t_1;
//		
//	Complement_angle	+= K_0 * Angle_err;	 //后验估计
//	Q_bias	+= K_1 * Angle_err;	 //后验估计
//	angle_dot   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
//        return Complement_angle;
//}
//
//
