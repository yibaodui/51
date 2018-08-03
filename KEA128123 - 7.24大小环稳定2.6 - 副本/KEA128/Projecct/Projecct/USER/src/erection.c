#include "erection.h"
#include "sensor.h"
//
#define orignal_Angle 18.85//25.8
#define dead_voltage   0
#define ANGLE_MAX 25
#define ANGLE_MIN -10
#define ANGLE_OUTPUT_MAX 3000;
#define CAR_SPEED_MAX 140
#define DT 0.005
extern float turn_offset;
extern uint8 flag_start;
extern int set_speed;
extern float angle_max; 
//int16 car_speed_err;
//int16 car_real_speed = 0,car_real_speed_old;
//int16 speed_left_err,speed_right_err;
float sss;
struct pid{
  
    float P;
    float D;
    float I;
    float err;
    float last_err;
    float pre_err;
    float   output;
    float   outputlast;
    float shell_output;
    float integral_value;
    float   actual_output;
};
struct pid Angle;
struct pid speed;
int16 car_real_speed = 0,car_real_speed_old;
uint16 count_car = 0;
uint8 flag_stop = 0;
//float Angle_P = 7.05,Angle_D = 0.025,Angle_I = 0.00;//27.4  2.15  28 2.5 250 17,Angle_D;
//float integral_value = 0.0,last_err = 0.0;
//int16 speed_left=0, speed_right=0,speed_left_old,speed_right_old;
//
///**********************************
// **函数名:speed_init
// **作用 :FTM及编码器测速端口初始化
// **返回值类型:void
// **作者:王鹏辉
// **时间:2018/1/27
//************************************/ 
//void speed_init(void)
//{
//    ftm_pwm_init(ftm2,ftm_ch0,15000,0);
//    ftm_pwm_init(ftm2,ftm_ch1,15000,0);
//    ftm_pwm_init(ftm2,ftm_ch2,15000,0);
//    ftm_pwm_init(ftm2,ftm_ch3,15000,0);
//    ftm_count_init(ftm0);   //对E0引脚输入的脉冲进行计数    E0接编码器LSB    
//    gpio_init(C5,GPI,0);    //用于判断方向                  C5接编码器DIR
//    port_pull(C5);          //IO上拉
//    port_pull(E0);          //IO上拉
//    
//    ftm_count_init(ftm1);   //对E7引脚输入的脉冲进行计数    E7接编码器LSB
//    gpio_init(H5,GPI,0);    //用于判断方向                  H5接编码器DIR
//    port_pull(H5);          //IO上拉
//    port_pull(E7);          //IO上拉
//
//}
//
///**********************************
// **函数名:zhili_speed
// **作用 :直立角度控制
// **返回值类型:void
// **作者:王鹏辉
// **时间:2018/1/27
//************************************/ 
//int zhili_speed(float angle,float gyro)
//{
//    Angle.err = angle - orignal_Angle;
//    Angle.output = Angle_P * Angle.err + Angle_D * gyro;
//    Angle.output = Angle.output;
//    return (int)Angle.output;
//    //printf("%d\n",(int)Angle.output);
//}
//int angle_core_series(float gyro)       //角速度内环控制
//{
//    float angle_rate_err;
//    Angle.P = 65.5;
//    Angle.D = 0.025;
//    Angle.I = 0.0;
//    
//    Angle.err = Angle.shell_output + gyro;
//    integral_value += Angle.err;
//    if(integral_value > 3000) integral_value = 3000;
//    if(integral_value < -3000) integral_value = -3000;
//    Angle.output = Angle.P * Angle.err + Angle.D * (Angle.err - last_err) + Angle.I * integral_value;
//    last_err = Angle.err;
//    angle_rate_err = Angle.output - Angle.outputlast;
// 
//    Angle.outputlast = Angle.output;
////    if(myabs((int)angle_rate_err) > 200)
////    {
////      if(Angle.output > 0)       Angle.output = ANGLE_OUTPUT_MAX;
////      if(Angle.output < 0)       Angle.output = -ANGLE_OUTPUT_MAX;
////    }
//    if(Angle.output > 8999) Angle.output = 8999;
//    if(Angle.output < -8999) Angle.output = -8999;
//    
//  //  printf("%d",(int)Angle.output);   printf(",");   printf("%d\n",(int)angle_rate_err);   
//    return (int)Angle.output;
//    
//}
//void angle_shell_series(float angle)    //角度外环控制
//{
//    if(angle > ANGLE_MAX)
//    {
//        angle = ANGLE_MAX;
//    }
//    if(angle < ANGLE_MIN)
//    {
//        angle = ANGLE_MIN;
//    }
//    Angle.last_err = angle - orignal_Angle + speed.output;
//    Angle.integral_value += Angle.last_err;
//    if(Angle.integral_value > 1000) Angle.integral_value = 1000;
//    if(Angle.integral_value < -1000) Angle.integral_value = -1000;
//    Angle.shell_output = Angle_P * Angle.last_err + Angle_D * (Angle.last_err - Angle.pre_err) + Angle_I * Angle.integral_value;
//    Angle.pre_err = Angle.last_err;
//    if(Angle.shell_output > 100) Angle.shell_output = 100;
//    if(Angle.shell_output < -100) Angle.shell_output = -100;
//   // if(angle > 31.2)    Angle.outputlast = 0;
//    //Angle.integral_value=0;
//}
///**********************************
// **函数名:real_speed
// **作用 :编码器速度读取
// **返回值类型:void
// **作者:王鹏辉
// **时间:2018/1/27
//************************************/ 
//void real_speed(void)
//{  
//    uint16 temp1,temp2;
//    //获取速度信息
//    speed_left_old = speed_left;
//    speed_right_old = speed_right;
//    temp1 = ftm_count_get(ftm0);
//    temp2 = ftm_count_get(ftm1);
//    //计数器清零
//    ftm_count_clean(ftm0);
//    ftm_count_clean(ftm1);
//    //根据方向信号判断正负，假设方向信号是高电平时为反转
//    if(gpio_get(C5))    speed_left = -((int16)(temp1));//速度取负
//    else                speed_left = temp1;             
//    if(gpio_get(H5)==0) speed_right = -((int16)(temp2));//速度取负
//    else                speed_right = temp2;
//
//   //printf("%d",speed_left);printf(",");printf("%d\n",speed_right);
//}
//
//
///**********************************
// **函数名:erec_speed
// **作用 :直立速度控制
// **返回值类型:void
// **作者:王鹏辉
// **时间:2018/1/27
//************************************/ 
//void erec_speed(int speed_set)
//{
//    //float SPEED_P,SPEED_I;
//    //speed.integral_value = 0;
//
//    uint8 count=0;
//    float temp;
//    speed.P = 0.228;//43.5  25 300 83
//    speed.I = 0.0075;// 1.0 * speed.P / 270
////     if(myabs(speed_left) - myabs(speed_left_old) > 40)
////    {
////        car_real_speed = car_real_speed_old;
////    }
//    
//    speed.last_err = speed_set - car_real_speed;
//    speed.err = 0.8 * speed.err + 0.2 * speed.last_err; //低通滤波
////    SPEED_P = speed.err*speed.P;
////    SPEED_I = speed.err*speed.I;
//    if(myabs((int16)speed.err) > 30)    //积分分离  编码器测速大于20不再积分
//    {
//        temp = 0.0;
//    }
//    else if(myabs((int16)speed.err) < 10)        //编码器测速小于9开启积分
//    {
//        temp = 1.0;
//        speed.integral_value += speed.err;
//        count ++;
//    }
//    else
//    {
//        temp = 1.0 * (30 - myabs((int16)speed.err)) / 20.0;       //变积分计算
//    }
//    if(count == 20)     //积分每隔20次清零
//    {
//         speed.integral_value = 0;
//         count=0;
//    }
//    
//    /*积分限幅*/
//    if(speed.integral_value >= 6999)       speed.integral_value = 6999;
//    if(speed.integral_value <= -6999)      speed.integral_value = -6999;
//
//    speed.outputlast = speed.output;    
//    speed.output = speed.err * speed.P + temp * speed.integral_value * speed.I;
//    //speed.output = 0.7 * speed.outputlast + 0.3 * speed.output;
//    if(speed.output >= 2000)       speed.output = 2000;
//    if(speed.output <= -2000)      speed.output = -2000;
//  //  printf("%d\n",(int)speed.actual_output);
//
//
//}
///**********************************
// **函数名:Smooth_speed
// **作用 :速度平滑输出
// **返回值类型:float
// **作者:王鹏辉
// **时间:2018/1/27
//************************************/ 
//float Smooth_speed(uint8 period)
//{
//    float mid_value;
//    
//    mid_value = speed.output - speed.outputlast;
//    speed.actual_output = (float)mid_value * (float)(period + 1 ) / 50.0 + speed.outputlast;
//    if(speed.output >= 2000)       speed.output = 2000;
//    if(speed.output <= -2000)      speed.output = -2000;
//    return speed.actual_output;
//    
//}
//
///**********************************
// **函数名:tyre_raised_deal
// **作用 :抬轮处理
// **返回值类型:无
// **作者:王鹏辉
// **时间:2018/4/29
//************************************/ 
//int16 tyre_raised_deal(int16 *real_speed,int16 *real_speed_old)
//{
//    int16 speed_err;
//    speed_err = *real_speed - *real_speed_old;
//    if(*real_speed_old > 50 && *real_speed_old < 500)
//    {
//        if(myabs(speed_err) > 40)
//        {
//            *real_speed = *real_speed_old;
//        }
//    }
//        return *real_speed;
//}
//
//void car_speed_limit(void)
//{
//    if(speed_left > CAR_SPEED_MAX)    speed_left = CAR_SPEED_MAX;
//    if(speed_left < -CAR_SPEED_MAX)    speed_left = -CAR_SPEED_MAX;
//    
//    if(speed_right > CAR_SPEED_MAX)    speed_right = CAR_SPEED_MAX;
//    if(speed_right < -CAR_SPEED_MAX)    speed_right = -CAR_SPEED_MAX;
//    car_real_speed_old = car_real_speed;
//    
//    car_real_speed = (speed_left + speed_right) / 2;      //计算车实际速度并减半以平衡左右轮转速
//    car_real_speed = tyre_raised_deal(&car_real_speed,&car_real_speed_old);
//}
///**********************************
// **函数名:stop_car
// **作用 :干簧管检测
// **返回值类型:无
// **作者:王鹏辉
// **时间:2018/5/13
//************************************/ 
//void stop_car(void)
//{
//   uint8 temp = 1;
//   temp = gpio_get(C1);
//    if(temp == 0)
//    {
//          DisableInterrupts;
//          ftm_pwm_duty(ftm2,ftm_ch1,0);
//          ftm_pwm_duty(ftm2,ftm_ch0,0);
//          ftm_pwm_duty(ftm2,ftm_ch3,0);
//          ftm_pwm_duty(ftm2,ftm_ch2,0);
//          while(1);
//    }
//}
//
///**********************************
// **函数名:speedoutput
// **作用 :电机pwm赋值
// **返回值类型:void
// **作者:王鹏辉
// **时间:2018/1/27
//************************************/ 
//float pwm_output_left = 0,pwm_output_right = 0;
//void speedoutput(int pwm_count,int direction_output)
//{  
////    speed_left_err = speed_left - speed_left_old;
////    speed_right_err = speed_right - speed_right_old;
//    pwm_output_left = pwm_count + direction_output;
//    pwm_output_right = pwm_count - direction_output;
//   // car_speed_err = speed_left - speed_right;
////    if(car_speed_err > 30)
////    {       
////        if(speed_left > 90 && speed_right < 65)
////        {
////            pwm_output_right = 0;
////            pwm_output_left += 2000;
////        }    
////    }
////    
////    if(car_speed_err < -30)
////    {
////       if(speed_right > 90 && speed_left < 65)
////        {
////            pwm_output_left = 0;
////            pwm_output_right += 2000;
////        }     
////    }
//    
//    //printf("%d",(int)pwm_output_left); printf(",");printf("%d\n",(int)pwm_output_right);
////    if(myabs(speed_left) > 330 || myabs(speed_right) > 330)
////    {   
////        ftm_pwm_duty(ftm2,ftm_ch1,0);
////         ftm_pwm_duty(ftm2,ftm_ch0,0);
////         ftm_pwm_duty(ftm2,ftm_ch3,0);
////         ftm_pwm_duty(ftm2,ftm_ch2,0);
////         DisableInterrupts;
////         while(1);
////         
////    }
////    if((myabs(speed_left) + myabs(speed_right) < 10) && (pwm_output_left > 9999 || pwm_output_left < -9999))
////    {
////          ftm_pwm_duty(ftm2,ftm_ch1,0);
////         ftm_pwm_duty(ftm2,ftm_ch0,0);
////         ftm_pwm_duty(ftm2,ftm_ch3,0);
////         ftm_pwm_duty(ftm2,ftm_ch2,0);
////         DisableInterrupts;
////         while(1);
////    }
////    if (myabs(speed_left) < 5 && myabs(speed_right) >150)
////    {
////        ftm_pwm_duty(ftm2,ftm_ch1,0);
////         ftm_pwm_duty(ftm2,ftm_ch0,0);
////         ftm_pwm_duty(ftm2,ftm_ch3,0);
////         ftm_pwm_duty(ftm2,ftm_ch2,0);
////         DisableInterrupts;
////         while(1);
////    }
////    if (myabs(speed_right) < 5 && myabs(speed_left) >150)
////    {
////        ftm_pwm_duty(ftm2,ftm_ch1,0);
////         ftm_pwm_duty(ftm2,ftm_ch0,0);
////         ftm_pwm_duty(ftm2,ftm_ch3,0);
////         ftm_pwm_duty(ftm2,ftm_ch2,0);
////         DisableInterrupts;
////         while(1);
////    }
//    if(pwm_output_left >= 7200)
//    {
//        pwm_output_left = 7200;       
//    }
//    if(pwm_output_right >= 7200)
//    {
//        pwm_output_right = 7200;
//    }
//    if(pwm_output_left <= -7200)
//    {
//        pwm_output_left = -7200;
//       
//    }
//    if(pwm_output_right <= -7200)
//    {
//        pwm_output_right = -7200;
//    }
//    if(pwm_output_left > 0 && pwm_output_right > 0)
//    {
//         ftm_pwm_duty(ftm2,ftm_ch0,(uint32_t)(pwm_output_left) + dead_voltage);
//         ftm_pwm_duty(ftm2,ftm_ch1,0);
//         ftm_pwm_duty(ftm2,ftm_ch2,(uint32_t)(pwm_output_right) + dead_voltage);
//         ftm_pwm_duty(ftm2,ftm_ch3,0);
//         
//      
//    }
//    if(pwm_output_left < 0 && pwm_output_right < 0)
//    {
//         ftm_pwm_duty(ftm2,ftm_ch1,(uint32_t)(-pwm_output_left) + dead_voltage);
//         ftm_pwm_duty(ftm2,ftm_ch0,0);
//         ftm_pwm_duty(ftm2,ftm_ch3,(uint32_t)(-pwm_output_right) + dead_voltage );
//         ftm_pwm_duty(ftm2,ftm_ch2,0);
//    }
//    if(pwm_output_left > 0 && pwm_output_right < 0)
//    {
//         ftm_pwm_duty(ftm2,ftm_ch0,(uint32_t)(pwm_output_left) + dead_voltage);
//         ftm_pwm_duty(ftm2,ftm_ch1,0);
//         ftm_pwm_duty(ftm2,ftm_ch3,(uint32_t)(pwm_output_right) + dead_voltage);
//         ftm_pwm_duty(ftm2,ftm_ch2,0);
//         
//      
//    }
//    if(pwm_output_left < 0 && pwm_output_right > 0)
//    {
//         ftm_pwm_duty(ftm2,ftm_ch1,(uint32_t)(-pwm_output_left) + dead_voltage);
//         ftm_pwm_duty(ftm2,ftm_ch0,0);
//         ftm_pwm_duty(ftm2,ftm_ch2,(uint32_t)(-pwm_output_right) + dead_voltage );
//         ftm_pwm_duty(ftm2,ftm_ch3,0);
//    }
//    
//
//    //printf("%d",(int)pwm_output_left); printf(",");printf("%d\n",(int)pwm_output_right);
//}
//
////uint8 unusual_motor()
//#include "erection.h"
//#include "sensor.h"
//
//#define orignal_Angle 17.43//25.8
//#define dead_voltage   0


//struct pid{
//  
//    float P;
//    float D;
//    float I;
//    float err;
//    float last_err;
//    float pre_err;
//    float   output;
//    float   outputlast;
//    float integral_value;
//    float   actual_output;
//};
//struct pid Angle;
//struct pid speed;

//float Angle_P = 6.65,Angle_D = 0.95,Angle_I = 0.00;//27.4  2.15  28 2.5 250 17,Angle_D;
float Angle_P = 7.55,Angle_D = 0.15,Angle_I = 0.00;//27.4  2.15  28 2.5 250 17,Angle_D;
float integral_value = 0.0,last_err = 0.0,speed_err;
int16 speed_left=0, speed_right=0,speed_left_old,speed_right_old;
uint16 Timer_count = 0; 
/**********************************
 **函数名:speed_init
 **作用 :FTM及编码器测速端口初始化
 **返回值类型:void
 **作者:王鹏辉
 **时间:2018/1/27
************************************/ 
void speed_init(void)
{
    ftm_pwm_init(ftm2,ftm_ch0,15000,0);
    ftm_pwm_init(ftm2,ftm_ch1,15000,0);
    ftm_pwm_init(ftm2,ftm_ch2,15000,0);
    ftm_pwm_init(ftm2,ftm_ch3,15000,0);
    ftm_count_init(ftm0);   //对E0引脚输入的脉冲进行计数    E0接编码器LSB    
    gpio_init(C5,GPI,0);    //用于判断方向                  C5接编码器DIR
    port_pull(C5);          //IO上拉
    port_pull(E0);          //IO上拉
    
    ftm_count_init(ftm1);   //对E7引脚输入的脉冲进行计数    E7接编码器LSB
    gpio_init(H5,GPI,0);    //用于判断方向                  H5接编码器DIR
    port_pull(H5);          //IO上拉
    port_pull(E7);          //IO上拉

}

/**********************************
 **函数名:zhili_speed
 **作用 :直立角度控制
 **返回值类型:void
 **作者:王鹏辉
 **时间:2018/1/27
************************************/ 
int zhili_speed(float angle,float gyro)
{
    Angle.err = angle - orignal_Angle;
    Angle.output = Angle_P * Angle.err + Angle_D * gyro;
    Angle.output = Angle.output;
    return Angle.output;
    //printf("%d\n",(int)Angle.output);
}
float angle_delta_old;;
int angle_core_series(float gyro)       //角速度内环控制
{
    float angle_rate_err;
    Angle.P = 65.5;
    Angle.D = 0.005;
    Angle.I = 0.0;
    angle_delta_old = Angle.err;
    Angle.err = 0.4 * angle_delta_old + 0.6 * (Angle.shell_output + gyro);
    integral_value += Angle.err;
    if(integral_value > 3000) integral_value = 3000;
    if(integral_value < -3000) integral_value = -3000;
    Angle.output = Angle.P * Angle.err + Angle.D * (Angle.err - last_err) + Angle.I * integral_value;
    last_err = Angle.err;
    angle_rate_err = Angle.output - Angle.outputlast;
 
    Angle.outputlast = Angle.output;
    if(myabs((int)angle_rate_err) > 200)
    {
      if(Angle.output > 0)       Angle.output = ANGLE_OUTPUT_MAX;
      if(Angle.output < 0)       Angle.output = -ANGLE_OUTPUT_MAX;
    }
    if(Angle.output > 8999) Angle.output = 8999;
    if(Angle.output < -8999) Angle.output = -8999;
    
  //  printf("%d",(int)Angle.output);   printf(",");   printf("%d\n",(int)angle_rate_err);   
    return (int)Angle.output;
    
}
//float angle_last,angle_next;
//int angle_core_series(float gyro)       //角速度内环控制
//{
//    float angle_rate_err;
//    float angle_increment;
//    Angle.P = 65.52;
//    Angle.D = 0.5;
//    Angle.I = 0.0;
//    
//    Angle.err = Angle.shell_output + gyro;
//    angle_increment = Angle.P * (Angle.err - angle_next) + Angle.D * (Angle.err - 2 * angle_next + angle_last) + Angle.I * Angle.err;
//    
// //   angle_rate_err = Angle.output - Angle.outputlast;
//    Angle.output += angle_increment;
//    angle_last = angle_next;
//    angle_next = Angle.err;
//    Angle.outputlast = Angle.output;
////    if(myabs((int)angle_rate_err) > 200)
////    {
////      if(Angle.output > 0)       Angle.output = ANGLE_OUTPUT_MAX;
////      if(Angle.output < 0)       Angle.output = -ANGLE_OUTPUT_MAX;
////    }
//    if(Angle.output > 8999) Angle.output = 8999;
//    if(Angle.output < -8999) Angle.output = -8999;
//    
//  //  printf("%d",(int)Angle.output);   printf(",");   printf("%d\n",(int)angle_rate_err);   
//    return (int)Angle.output;
//    
//}
void angle_shell_series(float angle)    //角度外环控制
{
    
    Angle.last_err = angle - orignal_Angle - speed.actual_output;
//    if(Angle.last_err > ANGLE_MAX)
//    {
//        Angle.last_err = ANGLE_MAX;
//    }
//    if(Angle.last_err < ANGLE_MIN)
//    {
//        Angle.last_err = ANGLE_MIN;
//    }
    if(angle > angle_max)
    {
        angle = angle_max;
    }
    if(angle < ANGLE_MIN)
    {
        angle = ANGLE_MIN;
    }
    Angle.integral_value += Angle.last_err;
    if(Angle.integral_value > 1000) Angle.integral_value = 1000;
    if(Angle.integral_value < -1000) Angle.integral_value = -1000;
    Angle.shell_output = Angle_P * Angle.last_err + Angle_D * (Angle.last_err - Angle.pre_err) + Angle_I * Angle.integral_value;
    Angle.pre_err = Angle.last_err;
    if(Angle.shell_output > 150) Angle.shell_output = 150;
    if(Angle.shell_output < -150) Angle.shell_output = -150;
    //printf("%d\n",(int)(10*angle));
   // if(angle > 31.2)    Angle.outputlast = 0;
    //Angle.integral_value=0;
}
/**********************************
 **函数名:real_speed
 **作用 :编码器速度读取
 **返回值类型:void
 **作者:王鹏辉
 **时间:2018/1/27
************************************/ 
float speed_car;
void real_speed(void)
{  
    uint16 temp1,temp2;
    //获取速度信息
    speed_left_old = speed_left;
    speed_right_old = speed_right;
    temp1 = ftm_count_get(ftm0);
    temp2 = ftm_count_get(ftm1);
    //计数器清零
    ftm_count_clean(ftm0);
    ftm_count_clean(ftm1);
    //根据方向信号判断正负，假设方向信号是高电平时为反转
    if(gpio_get(C5))    speed_left = -((int16)(temp1));//速度取负
    else                speed_left = temp1;             
    if(gpio_get(H5)==0) speed_right = -((int16)(temp2));//速度取负
    else                speed_right = temp2;
    speed_err = (speed_left + speed_right) - (speed_left_old + speed_right_old);
    speed_car = speed_left - speed_right;
    //printf("%d",(int)(speed_left));printf(",");printf("%d\n",(int)(speed_right));
}


/**********************************
 **函数名:erec_speed
 **作用 :直立速度控制
 **返回值类型:void
 **作者:王鹏辉
 **时间:2018/1/27
************************************/ 
float speed_set_last;
float speed_err_count[20];    

void erec_speed(int speed_set)
{
    float speed_filter_ratio = 0.85;   //低通滤波系数
    uint8 count=0;
    uint8 i;
    float temp;
//    speed.P = 0.355;//43.5  25 300 83
//    speed.I = 0.0075;// 1.0 * speed.P / 270 0.0075
    speed.P = 0.2;//43.5  25 300 83
    speed.I = 0.0;// 1.0 * speed.P / 270 0.0075
    if(myabs(turn_offset) < 3) speed_set *=  1;
//    if(flag_start == 1)
//    {
//        speed.I = 0.25;
//    }
//    else
//    {
//        speed.I = 0.005;
//    }
    Timer_count ++;
    speed_set = speed_set_last * (1 - speed_filter_ratio) + speed_set * speed_filter_ratio;
    speed_set_last = speed_set;
    speed.last_err = speed_set - car_real_speed;
    speed.err = 0.4 * speed.err + 0.6 * speed.last_err; //低通滤波
    if(Timer_count > 100 && myabs(turn_offset) < 5)
    {
        if(speed.err > 82)   speed.err = 82;
        if(speed.err < -82)  speed.err = -82;
    }
    if(Timer_count > 100)
    {
        if(myabs((int16)speed.err) > 20)    //积分分离  测速误差大于40不再积分
        {
            temp = 0.0;
        }
        else if(myabs((int16)speed.err) < 4)        //测速误差小于10开启积分
        {
            temp = 1.0;
            speed.integral_value += speed.err;
            count ++;
        }
        else
        {
            temp = 1.0 * (16 - myabs((int16)speed.err)) / 20.0;       //变积分计算
        }
    }
    for(i = 0; i < 19; i ++)
    {
        speed_err_count[i] = speed_err_count[i + 1];
        speed.integral_value += speed_err_count[i];
    }
    speed_err_count[19] = speed.err;
    speed.integral_value += speed_err_count[19];
    
    
    /*积分限幅*/
    if(speed.integral_value >= 10)       speed.integral_value = 10;
    if(speed.integral_value <= -10)      speed.integral_value = -10;

    speed.outputlast = speed.output;
    speed.output = speed.err * speed.P + temp * speed.integral_value * speed.I;
    //speed.output = 0.6 * speed.output + 0.4 * speed.outputlast;
    if(speed.output >= 20.5)       speed.output = 20.5;
    if(speed.output <= -20.5)      speed.output = -20.5;
  //  printf("%d\n",(int)speed.actual_output);
    //car_real_speed = car_real_speed_old;
}
/**********************************
 **函数名:Smooth_speed
 **作用 :速度平滑输出
 **返回值类型:float
 **作者:王鹏辉
 **时间:2018/1/27
************************************/ 
float Smooth_speed(uint8 period)
{
    float mid_value;
    
    mid_value = speed.output - speed.outputlast;
    speed.actual_output = (float)mid_value * (period + 1 ) / 100 + speed.outputlast;
    return speed.actual_output;
    
}
int16 Right_speed,Left_speed;
int16 Right_speed_old,Left_speed_old;
int16 Right_speed_accel,Left_speed_accel;
uint8 Left_crazy;
uint8 Right_crazy;
/**********************************
 **函数名:tyre_raised_deal
 **作用 :抬轮处理
 **返回值类型:无
 **作者:王鹏辉
 **时间:2018/4/29
************************************/ 
int16 tyre_raised_deal(int16 *real_speed,int16 *real_speed_old)
{
    int16 speed_err;
    speed_err = *real_speed - *real_speed_old;
    if(*real_speed_old > 50 && *real_speed_old < 500)
    {
        if(myabs(speed_err) > 60)
        {
            *real_speed = *real_speed_old;
        }
    }
        return *real_speed;
}

/**********************************
 **函数名:car_speed_limit
 **作用 :抬轮处理
 **返回值类型:无
 **作者:王鹏辉
 **时间:2018/7/20
************************************/ 
void car_speed_limit(void)
{
    int16 SPEED_SET = set_speed;
//    if(speed_left > CAR_SPEED_MAX)    speed_left = CAR_SPEED_MAX;
//    if(speed_left < -CAR_SPEED_MAX)    speed_left = -CAR_SPEED_MAX;
//    
//    if(speed_right > CAR_SPEED_MAX)    speed_right = CAR_SPEED_MAX;
//    if(speed_right < -CAR_SPEED_MAX)    speed_right = -CAR_SPEED_MAX;
    Left_speed_accel = speed_left - Left_speed_old;
    Right_speed_accel = speed_right - Right_speed_old;
    
  /*左轮疯转*/  
/************************************************/
    if(myabs(Left_speed_accel) > 100)
    {
        Left_crazy = 1;
    }
    if(myabs(speed_left) > (myabs(SPEED_SET - 200)))
    {
        Left_crazy = 2;
    }
    if(Left_crazy)
    {
        if((myabs(Left_speed_accel) < 100) && (myabs(speed_left) < (myabs(SPEED_SET - 200))))
        {
            Left_crazy = 0;
        }
    }
    if(!Left_crazy)
    {
        Left_speed = 0.9 * speed_left + 0.1 * Left_speed_old;
        Left_speed_old = Left_speed;
    }
    else
    {
        Left_speed = 0.4 * speed_left + 0.6 * Left_speed_old;
        Left_speed_old = Left_speed;
    }
    
 /*右轮疯转*/   
/****************************************/    
    if(myabs(Right_speed_accel) > 100)
    {
        Right_crazy = 1;
    }
    if(myabs(speed_right) > (myabs(SPEED_SET - 200)))
    {
        Right_crazy = 2;
    }
    if(Right_crazy)
    {
        if((myabs(Right_speed_accel) < 100) && (myabs(speed_right) < myabs(SPEED_SET - 200)))
        {
            Right_crazy = 0;
        }
    }
    if(!Right_crazy)
    {
        Right_speed = 0.9 * speed_right + 0.1 * Right_speed_old;
        Right_speed_old = Right_speed;
    }
    else
    {
        Right_speed = 0.4 * speed_right + 0.6 * Right_speed_old;
        Right_speed_old = Right_speed;
    }
    
/*疯转处理*/
/*****************************************************/  
    
    if((Left_crazy > 0) && (Right_crazy > 0))
    {
        car_real_speed = SPEED_SET;       
    }
    else if(Left_crazy)
    {
        if(myabs(Right_speed) > myabs(SPEED_SET))
        {
            car_real_speed = SPEED_SET;
        }
        else
        {
            car_real_speed = Right_speed;
        }
    }
    else if(Right_crazy)
    {
        if(myabs(Left_speed) > myabs(SPEED_SET))
        {
            car_real_speed = SPEED_SET;
        }
        else
        {
            car_real_speed = Left_speed;
        }
    }
    else
    {
        car_real_speed = (Left_speed + Right_speed) / 2;      //计算车实际速度并减半以平衡左右轮转速
        sss += (float)(car_real_speed) * DT;
    }
//    if(sss < -7750)
//      set_speed = -60;
//    if(sss < -8250)
//      set_speed = -115;
   // printf("%d\n",(int)(10*sss));
   // car_real_speed = car_real_speed * 0.1 + car_real_speed_old * 0.9; 
    car_real_speed_old = car_real_speed * 0.9 + car_real_speed_old * 0.1;
/*********************************************************************************/
    //car_real_speed = tyre_raised_deal(&car_real_speed,&car_real_speed_old);
}

/**********************************
 **函数名:stop_car
 **作用 :干簧管检测
 **返回值类型:无
 **作者:王鹏辉
 **时间:2018/5/13
************************************/ 
void stop_car(void)
{
   uint8 temp = 1;
   temp = gpio_get(C1);
    if(temp == 0)
    {
        flag_stop = 1;  
    }
    if(flag_stop)   
    {
        count_car++;
      if(count_car == 700)
      {
          DisableInterrupts;
          ftm_pwm_duty(ftm2,ftm_ch1,0);
          ftm_pwm_duty(ftm2,ftm_ch0,0);
          ftm_pwm_duty(ftm2,ftm_ch3,0);
          ftm_pwm_duty(ftm2,ftm_ch2,0);
          while(1);
      }
      else
      {
          speedoutput(0,0);
          ftm_pwm_duty(ftm2,ftm_ch1,0);
          ftm_pwm_duty(ftm2,ftm_ch0,3000);
          ftm_pwm_duty(ftm2,ftm_ch3,3000);
          ftm_pwm_duty(ftm2,ftm_ch2,0);
      }
    }
    
}
/**********************************
 **函数名:speedoutput
 **作用 :电机pwm赋值
 **返回值类型:void
 **作者:王鹏辉
 **时间:2018/1/27
************************************/ 
float pwm_output_left = 0,pwm_output_right = 0;
void speedoutput(int pwm_count,int direction_output)
{  
    pwm_output_left = pwm_count + direction_output;
    pwm_output_right = pwm_count - direction_output;
    //printf("%d",(int)pwm_output_left); printf(",");printf("%d\n",(int)pwm_output_right);
//    if(myabs(speed_left) > 290 || myabs(speed_right) > 290)
//    {   
//        ftm_pwm_duty(ftm2,ftm_ch1,0);
//         ftm_pwm_duty(ftm2,ftm_ch0,0);
//         ftm_pwm_duty(ftm2,ftm_ch3,0);
//         ftm_pwm_duty(ftm2,ftm_ch2,0);
//         DisableInterrupts;
//         while(1);
//         
//    }
//    if((myabs(speed_left) + myabs(speed_right) < 10) && (pwm_output_left > 9999 || pwm_output_left < -9999))
//    {
//          ftm_pwm_duty(ftm2,ftm_ch1,0);
//         ftm_pwm_duty(ftm2,ftm_ch0,0);
//         ftm_pwm_duty(ftm2,ftm_ch3,0);
//         ftm_pwm_duty(ftm2,ftm_ch2,0);
//         DisableInterrupts;
//         while(1);
//    }
//    if (myabs(speed_left) < 5 && myabs(speed_right) >150)
//    {
//        ftm_pwm_duty(ftm2,ftm_ch1,0);
//         ftm_pwm_duty(ftm2,ftm_ch0,0);
//         ftm_pwm_duty(ftm2,ftm_ch3,0);
//         ftm_pwm_duty(ftm2,ftm_ch2,0);
//         DisableInterrupts;
//         while(1);
//    }
//    if (myabs(speed_right) < 5 && myabs(speed_left) >150)
//    {
//        ftm_pwm_duty(ftm2,ftm_ch1,0);
//         ftm_pwm_duty(ftm2,ftm_ch0,0);
//         ftm_pwm_duty(ftm2,ftm_ch3,0);
//         ftm_pwm_duty(ftm2,ftm_ch2,0);
//         DisableInterrupts;
//         while(1);
//    }
    if(pwm_output_left >= 9999)
    {
        pwm_output_left = 9999;       
    }
    if(pwm_output_right >= 9999)
    {
        pwm_output_right = 9999;
    }
    if(pwm_output_left <= -9999)
    {
        pwm_output_left = -9999;
       
    }
    if(pwm_output_right <= -9999)
    {
        pwm_output_right = -9999;
    }
    if(pwm_output_left > 0 && pwm_output_right > 0)
    {
         ftm_pwm_duty(ftm2,ftm_ch0,(uint32_t)(pwm_output_left) + dead_voltage);
         ftm_pwm_duty(ftm2,ftm_ch1,0);
         ftm_pwm_duty(ftm2,ftm_ch3,(uint32_t)(pwm_output_right) + dead_voltage);
         ftm_pwm_duty(ftm2,ftm_ch2,0);
      
    }
    if(pwm_output_left < 0 && pwm_output_right > 0)
    {
         ftm_pwm_duty(ftm2,ftm_ch0,0);
         ftm_pwm_duty(ftm2,ftm_ch1,(uint32_t)(-pwm_output_left) + dead_voltage);
         ftm_pwm_duty(ftm2,ftm_ch3,(uint32_t)(pwm_output_right) + dead_voltage);
         ftm_pwm_duty(ftm2,ftm_ch2,0);
      
    }
    if(pwm_output_left > 0 && pwm_output_right < 0)
    {
         ftm_pwm_duty(ftm2,ftm_ch0,(uint32_t)(pwm_output_left) + dead_voltage);
         ftm_pwm_duty(ftm2,ftm_ch1,0);
         ftm_pwm_duty(ftm2,ftm_ch3,0);
         ftm_pwm_duty(ftm2,ftm_ch2,(uint32_t)(-pwm_output_right) + dead_voltage);
      
    }
    if(pwm_output_left < 0 && pwm_output_right < 0)
    {
         ftm_pwm_duty(ftm2,ftm_ch1,(uint32_t)(-pwm_output_left) + dead_voltage);
         ftm_pwm_duty(ftm2,ftm_ch0,0);
         ftm_pwm_duty(ftm2,ftm_ch2,(uint32_t)(-pwm_output_right) + dead_voltage);
         ftm_pwm_duty(ftm2,ftm_ch3,0);
    }

    //printf("%d",(int)pwm_output_left); printf(",");printf("%d\n",(int)pwm_output_right);
}

//uint8 unusual_motor()