#include "isr.h"
#include "headfile.h"
#include "angle.h"
#include "sensor.h"

#define speed_control_period 100
#define direction_control_period 1
float gyro,angle_set;
extern int speed_left;
extern uint8 flag_set_power;
extern uint8 speed_rank;
float accel;
float angle_max;
float offset_value,offset_count;
int speed_out,Direction_out,pwm_out,angle_out;
uint8 speed_count = 0,direction_count = 0,angle_count = 0,angle_dot_count = 0,dire_count = 0;
uint8 flag_start = 0;
uint16 start_count = 0,stop_count = 0; 
int set_speed;
extern int16 sen_sor[2];
uint8 flag_sensor_start = 0;
void PIT_CH0_IRQHandler(void)
{       
    PIT_FLAG_CLR(pit0);
    speed_count ++;
    direction_count ++;
    angle_count ++;
    stop_count ++;
    angle_dot_count ++;
    dire_count ++;
    //temp = gpio_get(C1);
     port_pull(C1);
    
    switch (speed_rank)
    {
        case 0 :        set_speed = -115;   angle_max = 6.5;    break;
        case 1 :        set_speed = -108;   angle_max = 8;      break;
        case 2 :        set_speed = -95;    angle_max = 9.5;    break;
        case 3 :        set_speed = -90;    angle_max = 11.5;   break;
        case 4 :        set_speed = -85;    angle_max = 13.5;   break;
        case 5 :        set_speed = -60;    angle_max = 15.5;   break;
    }
    if(sen_sor[2] > 2)
    {
        flag_sensor_start = 1;
    }
    if(start_count == 5000)
    {     
      
         accel = accel_angle();
         gyro = gy_angle();
          
         
        angle_out = angle_core_series(gyro); 
           
        pwm_out = angle_out;
        angle_dot_count = 0;
         
         
        if(angle_count == 5)
        {
            
            real_speed(); 
            car_speed_limit();
            angle_set = Complement_Filter();
            angle_shell_series(angle_set);

            angle_count = 0;
        }
        if(dire_count == 5)
        {
            offset_value = offset_get();
             offset_count = Direction_get(offset_value);
            dire_count = 0;
        }
        
        if(direction_count == direction_control_period)
        {     
           
            turn_core_series(offset_count);
            direction_count = 0;
        }
        if(speed_count == speed_control_period)
        { 
           // temp = offset_value * offset_value * m + n;
            flag_start = 0;
            erec_speed(set_speed);
            speed_count = 0;
          
        }
        speed_out = Smooth_speed(speed_count);
        Direction_out = Direction_output(direction_count);
        if(flag_set_power && flag_sensor_start)  speedoutput(pwm_out,Direction_out);
        if((sen_sor[2] == 2) && flag_sensor_start)//冲出赛道保护
        {
           DisableInterrupts;
           ftm_pwm_duty(ftm2,ftm_ch1,0);
           ftm_pwm_duty(ftm2,ftm_ch0,0);
           ftm_pwm_duty(ftm2,ftm_ch3,0);
           ftm_pwm_duty(ftm2,ftm_ch2,0);
           while(1);
           
        }
        if(stop_count > 15000)
        {
              stop_car();
        }
        start_count = 5000;
    }
    else
    {
        start_count ++;
        speed_out = Smooth_speed(speed_count);
       // Direction_out = Direction_output(direction_count); 
         
         
         accel = accel_angle();
         gyro = gy_angle(); 
         if(angle_dot_count == 1)
         {
              angle_out = angle_core_series(gyro); 
              pwm_out = angle_out;
              angle_dot_count = 0;
         }
         
        if(angle_count == 5)
        {
            real_speed();
            angle_set = Complement_Filter();
            angle_shell_series(angle_set);
            angle_count = 0;
        }
        if(speed_count == speed_control_period)
        { 
            flag_start = 1;
            erec_speed(0);
            speed_count = 0;
            
        }
        //if(flag_set_power) speedoutput(pwm_out,Direction_out);    
    }
       
}

void PIT_CH1_IRQHandler(void)
{
    
}
void IRQ_IRQHandler(void)
{
    CLEAR_IRQ_FLAG;
    
}


void KBI0_IRQHandler(void)
{
    CLEAN_KBI0_FLAG;
    
}





/*
中断函数名称，用于设置对应功能的中断函数
Sample usage:当前启用了周期定时器 通道0得中断
void PIT_CH0_IRQHandler(void)
{
    ;
}
记得进入中断后清除标志位

FTMRE_IRQHandler      
PMC_IRQHandler        
IRQ_IRQHandler        
I2C0_IRQHandler       
I2C1_IRQHandler       
SPI0_IRQHandler       
SPI1_IRQHandler       
UART0_IRQHandler 
UART1_IRQHandler 
UART2_IRQHandler 
ADC0_IRQHandler       
ACMP0_IRQHandler      
FTM0_IRQHandler       
FTM1_IRQHandler       
FTM2_IRQHandler       
RTC_IRQHandler        
ACMP1_IRQHandler      
PIT_CH0_IRQHandler    
PIT_CH1_IRQHandler    
KBI0_IRQHandler       
KBI1_IRQHandler       
Reserved26_IRQHandler 
ICS_IRQHandler        
WDG_IRQHandler        
PWT_IRQHandler        
MSCAN_Rx_IRQHandler   
MSCAN_Tx_IRQHandler   
*/



