#include "headfile.h"
#include "sensor.h"
#include "angle.h"
#include "erection.h"
extern int16 x,y,z;

extern int16 offset_value_new;
extern int16 pwm_out;
extern uint8 flag_count;
uint8 ch[100];
int16 a,a1,a2;
extern int16 sen_sor[5];
extern float Complement_angle;
//extern int pwm_output_right;
extern float Angle_P;
extern float Angle_D;
extern float D;
extern float Direction_P;
extern float turn_P;//30.65
extern float turn_D;//15.5
extern float turn_I;
extern int16 next_value;
extern struct pid{
  
    float P;
    float D;
    float I;
    float err;
    float last_err;
    float   output;
    float   outputlast;
    float integral_value;
    float   actual_output;
};
extern struct pid Angle;
extern int turn_offset;
uint8 dat,flag_set_power = 0;
uint8 flag_sensor = 0;
uint8 flag_set_speed = 0;
uint8 flag_start_ok = 0;
extern int set_speed;
uint8 speed_rank = 0;
uint8 flag_cycle = 0;
extern int16 mid_pre;
uint8 choose_cycle_flag = 0;
uint8 flag_cho = 0;
uint8 flag_adjust = 0;
uint8 flag_adjust_dir;
uint8 flag_set_dir = 0;
extern int16 ofsetss;
extern int16 center_offset;
uint8 adjust_count = 0;
extern int sss;
void vcan_sendware(uint8 *wareaddr, uint8 waresize)
{
#define CMD_WARE     3
    uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};    //串口调试 使用的前命令
    uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};    //串口调试 使用的后命令

    uart_putbuff(DEBUG_PORT, cmdf, sizeof(cmdf));    //先发送前命令
    uart_putbuff(DEBUG_PORT, wareaddr, waresize);    //发送数据
    uart_putbuff(DEBUG_PORT, cmdr, sizeof(cmdr));    //发送后命令    

}
/*
      p1 =       186.1  (176, 196.2)
       p2 =      -90.58  (-104.4, -76.82)
       q1 =     -0.7717  (-0.9377, -0.6058)
       q2 =       0.858  (0.7332, 0.9829)
*/
/*
    f(x) = (p1*x + p2) / (x^2 + q1*x + q2)
*/
int main(void)
{
    int16 var[6];
    get_clk();              //获取时钟频率 必须执行
    
    IIC_init();
    InitMPU6050();
    //MMA845x_init();
    speed_init();
    sensor_init();
    OLED_Init();
    FLASH_Init();
    gpio_init(G0,GPI,1);
    gpio_init(G1,GPI,1);
    gpio_init(G2,GPI,1);
    gpio_init(G3,GPI,1);
    gpio_init(C1,GPI,1);
    gpio_init(C6,GPO,1);
    port_pull(C1);
    port_pull(C6);  
    pit_init_ms(pit0,1);
    set_irq_priority(PIT_CH0_IRQn,1);	//设置pit0优先级
    enable_irq(PIT_CH0_IRQn);
    EnableInterrupts; 
   while(1)
   {        
 
      if(gpio_get(G1) == 0)
      {
          OLED_Fill(0x00);
          systick_delay_ms(10);
          flag_sensor = 1;
          while(!gpio_get(G1));
      }
      
      
      if(flag_sensor)
      {           
          OLED_P6x8Str(1,1,"left1");
          OLED_Print_Num1(70,10,sen_sor[0]);
          OLED_P6x8Str(1,2,"right2");
          OLED_Print_Num1(70,10,sen_sor[1]);
          OLED_P6x8Str(1,3,"mid");
          OLED_Print_Num1(70,10,sen_sor[2]);
          OLED_P6x8Str(1,4,"offset");
          OLED_Print_Num1(70,10,ofsetss);
          OLED_P6x8Str(1,5,"cenoffset");
          OLED_Print_Num1(70,10,center_offset);
          OLED_P6x8Str(1,6,"predic");
          OLED_Print_Num1(70,10,mid_pre);
          OLED_P6x8Str(1,7,"angle");
          OLED_Print_Num1(70,10,Complement_angle);
      }
      
      
      if(gpio_get(G2) == 0)
      {     
          systick_delay_ms(10);
          if(gpio_get(G2) == 0)
          {
              OLED_Fill(0x00);
              flag_cycle = 1;
              choose_cycle_flag ++;
          }
          while(!gpio_get(G2));
      }
      if(flag_cycle)
      {
          
          OLED_P6x8Str(1,1,"cycleModel");
          if((choose_cycle_flag % 3) == 1)
          {
              flag_cho = 0;
              flag_adjust = 0;
              OLED_P6x8Str(1,6,"danleft");
              
          }
          else if((choose_cycle_flag % 3) == 2)
          {
              flag_cho = 0;
              flag_adjust = 1;
              OLED_P6x8Str(1,6,"danright");
          }
          else
          {
              flag_cho = 1;
              OLED_P6x8Str(1,6,"shuang");
          }
      }
      if(gpio_get(G3) == 0)
      {
          systick_delay_ms(10);
          if(gpio_get(G3) == 0)
          {
            OLED_Fill(0x00);
            speed_rank  = speed_rank + 1;
            flag_set_speed = 1;
          }
          while(!gpio_get(G3));
      }
      
      if(flag_set_speed)
      {
          OLED_P6x8Str(1,1,"rank");
          OLED_Print_Num1(58,10,speed_rank);
          OLED_P6x8Str(1,2,"Set");
          OLED_Print_Num1(58,10,set_speed);
          OLED_P6x8Str(1,3,"Set");
          OLED_Print_Num1(58,10,sss);
      }
      
      
      
      if(gpio_get(G0) == 0)
      {
          OLED_Fill(0x00);
          systick_delay_ms(10);
          flag_set_speed = 0;
          flag_sensor = 0;
          flag_start_ok = 1;
          flag_cycle = 0;
          flag_set_dir = 0;
          while(!gpio_get(G0));
      }
      if(flag_start_ok)
      {
          if(gpio_get(G1) == 0)
          {
              systick_delay_ms(10);
              if(gpio_get(G1) == 0)
              {
                  OLED_Fill(0x00);
                  OLED_P8x16Str(2,3,"GOGOGO!!!");
                  flag_set_power = 1;
                
              }
              while(!gpio_get(G1));
          }
      }
   }
}







//        ftm_pwm_duty(ftm2,ftm_ch0,0);
//         ftm_pwm_duty(ftm2,ftm_ch1,4000);
//         ftm_pwm_duty(ftm2,ftm_ch3,0);
//         ftm_pwm_duty(ftm2,ftm_ch2,4000);
//                 ftm_pwm_duty(ftm2,ftm_ch0,0);
//         ftm_pwm_duty(ftm2,ftm_ch1,0);
//         ftm_pwm_duty(ftm2,ftm_ch2,0);
//         ftm_pwm_duty(ftm2,ftm_ch3,3000);
//          var[0] = sen_sor[0];
//          var[1] = sen_sor[3];
//          var[2] = 0;
//          var[3] = (sen_sor[0] + sen_sor[3]);
//          var[4] = (sen_sor[2] + sen_sor[4]);
//          var[5] = sen_sor[1];
////          var[6] =  next_value;
        // vcan_sendware((uint8 *)var, sizeof(var));
////          var[0] = x;
//          var[1] = y;
//          var[2] = z;
//          var[3] = 0;
//          var[4] = 0;
 //         var[5] = 0;

////        //uart_getchar(uart1,&dat); 
//
          //accel = accel_angle();
          //offset_value = offset_get();
//          a = Complement_angle*100;
//          a1 = Direction_P*10;
//          a2 =D*10;
//          OLED_P6x8Str(1,1,"angle");
//          OLED_Print_Num1(38,10,a);
//          OLED_P6x8Str(1,3,"TP");
//          OLED_Print_Num1(38,10,a1);
//          OLED_P6x8Str(1,5,"DP");
//          OLED_Print_Num1(38,10,a2);
//          if(gpio_get(G0) == 0)
//          {
//              systick_delay_ms(10);
//              if(gpio_get(G0) == 0)
//              flag_set_power = 1;
//              while(!gpio_get(G0));
//              
//          }
//          if(gpio_get(G2) == 0)
//          { 
//              systick_delay_ms(10);
//              if(gpio_get(G2) == 0)
//              Direction_P = Direction_P + 0.5;
//              while(!gpio_get(G2));
//          }
////        if(gpio_get(G1) == 0)
////          {
////              systick_delay_ms(10);
////              if(gpio_get(G1) == 0)
////              Direction_D = Direction_D + 0.25;
////              while(!gpio_get(G1));
////              
////          }
//          if(gpio_get(G3) == 0)
//          {
//              systick_delay_ms(10);
//              if(gpio_get(G3) == 0)
//              D = D + 0.5;
//              while(!gpio_get(G3));
//              
//          }
//void OLED_modify(void)
//{
//    OLED_P6x8Str(2,1,"Msensor");
//    OLED_P6x8Str(2,3,"Mcycle");
//    OLED_P6x8Str(2,5,"Mspeed");
//    OLED_P6x8Str(2,7,"Start");
    //OLED_Fill(0x00);