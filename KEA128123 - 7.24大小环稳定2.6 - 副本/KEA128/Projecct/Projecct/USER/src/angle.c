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
//float Q_angle=0.001;// ����������Э����
//float Q_gyro=0.003;//0.003 ����������Э���� ����������Э����Ϊһ��һ�����о���
//float R_angle=0.5;// ����������Э���� �Ȳ���ƫ��
////                 
//char  C_0 = 1;
//float Q_bias, Angle_err;
//float PCt_0, PCt_1, E;
//float K_0, K_1, t_0, t_1;
//float Pdot[4] ={0,0,0,0};
//float PP[2][2] = { { 1, 0 },{ 0, 1 } };


/**********************************
 **������:accel_angle
 **���� :������ٶȼƽǶ�
 **����ֵ����:float
 **����:������
 **ʱ��:2018/1/22
************************************/ 

float accel_angle(void)//���ٶȼ�����Χ 2g 16384LSB/s  z0/16464
{
    Get_AccData();
    acc_angle = atanf(1.0 * mpu_acc_z / mpu_acc_x) * 180 / PI;
    //printf("%d\n",(int)mpu_acc_x);
    return acc_angle;
//      get_mma8451();    //��ȡ���ٶȼ�ֵ
//      acc_angle = (float)atan2f(1.0 * acc_z,1.0 * acc_y) * 180 / PI;    //�����Ǻ������㵱ǰ���ٶȼƽǶ�
//      return acc_angle;  
}


/**********************************
 **������:gy_angle
 **���� :��ȡ������ԭʼ����
 **����ֵ����:float
 **����:������
 **ʱ��:2018/1/22
************************************/ 
float gy_angle(void)//�����Ǽ�����Χ 2000 16.4LSB/s
{ 
    Get_Gyro();   
    mpu_gyro_y = 0.8 * mpu_gyro_y_old + 0.2 * mpu_gyro_y;
    mpu_gyro_y_old = mpu_gyro_y;
    gyro_angle = 1.0 * (mpu_gyro_y - gyro_zero) / 16.4;
    gy0_angle = gy0_angle + gyro_angle * dt;
    gyro_angle = K*gyro_angle;  //������ֵ����
    return gyro_angle; 
//      get_l3g4200d();
//      gyro_angle = 1.0*(gyro_y - gyro_zero) / 15.4;
//      gy0_angle = gy0_angle + gyro_angle * dt;
//      return gyro_angle;
}


/**********************************
 **������:Complement_Filter
 **���� :�����ں��˲�
 **����ֵ����:float
 **����:������
 **ʱ��:2018/1/22
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
//          float mid_temp;//�����ں��˲��м���																
//    float back_value;//�ںϺ�ĽǶ�����ٶȾ�̬�ǶȲ�ֵ����Ϊ������������ֻ�·���þ���Ϊ0
//																											 
//    mid_temp=Complement_angle;  																	 
//    gyro_angle=K*gyro_angle;//���ٶ����� 																			   
//    back_value=(acc_angle-mid_temp)*3.95;//�ںϺ�ĽǶ�����ٶȾ�̬�ǶȲ�ֵ	
//    Complement_angle=Complement_angle+(gyro_angle+back_value )*dt;
//    //zhili_speed(Complement_angle,gyro_angle);
//    return  Complement_angle;




//float Kalman_Filter(float Accel,float Gyro)		
//{
//	Complement_angle+=(Gyro - Q_bias) * dt; //�������
//	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��
//
//	Pdot[1]=-PP[1][1];
//	Pdot[2]=-PP[1][1];
//	Pdot[3]=Q_gyro;
//	PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
//	PP[0][1] += Pdot[1] * dt;   // =����������Э����
//	PP[1][0] += Pdot[2] * dt;
//	PP[1][1] += Pdot[3] * dt;
//		
//	Angle_err = Accel - Complement_angle;	//zk-�������
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
//	PP[0][0] -= K_0 * t_0;		 //����������Э����
//	PP[0][1] -= K_0 * t_1;
//	PP[1][0] -= K_1 * t_0;
//	PP[1][1] -= K_1 * t_1;
//		
//	Complement_angle	+= K_0 * Angle_err;	 //�������
//	Q_bias	+= K_1 * Angle_err;	 //�������
//	angle_dot   = Gyro - Q_bias;	 //���ֵ(�������)��΢��=���ٶ�
//        return Complement_angle;
//}
//
//
