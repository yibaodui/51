#include "isr.h"
#include "headfile.h"
#include "angle.h"
#include "sensor.h"


#define speed_control_period 100
#define direction_control_period 1
float gyro,angle_set;
extern int speed_left;
extern uint8 flag_set_power;
float accel;
float offset_value,offset_count;
int speed_out,Direction_out,pwm_out,angle_out;
uint8 speed_count = 0,direction_count = 0,angle_count = 0,angle_dot_count = 0,dire_count = 0;
uint16 start_count = 0,stop_count = 0; 
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
    if(stop_count > 10000)
    {
          stop_car();
    }
    if(start_count == 5000)
    {     
      
         accel = accel_angle();
         gyro = gy_angle();

         if(angle_dot_count = 1)
         {
              angle_out = angle_core_series(gyro); 
                 
              pwm_out = angle_out;
              angle_dot_count = 0;
         }
         
        if(angle_count == 5)
        {
            car_speed_limit(); 
            real_speed(); 
            angle_set = Complement_Filter();
            angle_shell_series(angle_set);
            
            angle_count = 0;
        }
        if(dire_count == 10)
        {
            offset_value = offset_get();
            
            dire_count = 0;
        }
        if(direction_count == direction_control_period)
        {     
           offset_count = Direction_get(offset_value); 
            //turn_core_series(offset_count);
            direction_count = 0;
        }
        if(speed_count == speed_control_period)
        { 
           // temp = offset_value * offset_value * m + n;
          
            erec_speed(-40);
            speed_count = 0;
            
        }
        speed_out = Smooth_speed(speed_count);
        Direction_out = Direction_output(direction_count);
        if(flag_set_power) speedoutput(pwm_out,Direction_out);
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
            erec_speed(0);
            speed_count = 0;
            
        }
        if(flag_set_power) speedoutput(pwm_out,Direction_out);    
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
�жϺ������ƣ��������ö�Ӧ���ܵ��жϺ���
Sample usage:��ǰ���������ڶ�ʱ�� ͨ��0���ж�
void PIT_CH0_IRQHandler(void)
{
    ;
}
�ǵý����жϺ������־λ

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



