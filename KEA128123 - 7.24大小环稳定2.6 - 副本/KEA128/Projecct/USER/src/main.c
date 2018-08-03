#include "headfile.h"
#include "sensor.h"
#include "angle.h"
#include "erection.h"
extern int16 x,y,z;
/*2*/
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
extern float Direction_P;
extern float Direction_D;
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
extern float turn_offset;
uint8 dat,flag_set_power = 0;
void vcan_sendware(uint8 *wareaddr, uint8 waresize)
{
#define CMD_WARE     3
    uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};    //串口调试 使用的前命令
    uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};    //串口调试 使用的后命令

    uart_putbuff(DEBUG_PORT, cmdf, sizeof(cmdf));    //先发送前命令
    uart_putbuff(DEBUG_PORT, wareaddr, waresize);    //发送数据
    uart_putbuff(DEBUG_PORT, cmdr, sizeof(cmdr));    //发送后命令    

}

int main(void)
{
    int16 var[7];
    get_clk();              //获取时钟频率 必须执行
    
    IIC_init();
    InitMPU6050();
    //MMA845x_init();
    speed_init();
    sensor_init();
    OLED_Init();
    gpio_init(G0,GPI,1);
    gpio_init(G1,GPI,1);
    gpio_init(G2,GPI,1);
    gpio_init(I2,GPO,1);
    gpio_init(G3,GPI,1);
    gpio_init(C1,GPI,1);
    port_pull(C1);
    port_pull(I2);
    pit_init_ms(pit0,1);
    set_irq_priority(PIT_CH0_IRQn,1);	//设置pit0优先级
    enable_irq(PIT_CH0_IRQn);
    EnableInterrupts;
//    
    //InitL3G4200D();
    //uart_init(uart1,115200);
   while(1)
   {   
          gpio_set(I2,0);
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
//          var[2] = sen_sor[2];
//          var[3] = sen_sor[1];
//          var[4] = sen_sor[4];
//          var[5] = (int16)turn_offset;
//          var[6] =  next_value;
//          vcan_sendware((uint8 *)var, sizeof(var));
//          var[0] = x;
//          var[1] = y;
//          var[2] = z;
//          var[3] = 0;
//          var[4] = 0;
 //         var[5] = 0;

////        //uart_getchar(uart1,&dat); 
//
          //accel = accel_angle();
          //offset_value = offset_get();
          a = Complement_angle*100;
          a1 = sen_sor[0];
          a2 = sen_sor[1];
          OLED_P6x8Str(1,1,"angle");
          OLED_Print_Num1(38,10,a);
          OLED_P6x8Str(1,3,"P");
          OLED_Print_Num1(38,10,a1);
          OLED_P6x8Str(1,5,"D");
          OLED_Print_Num1(38,10,a2);
          if(gpio_get(G0) == 0)
          {
              systick_delay_ms(10);
              if(gpio_get(G0) == 0)
              flag_set_power = 1;
              while(!gpio_get(G0));
              
          }
//          if(gpio_get(G2) == 0)
//          { 
//              systick_delay_ms(10);
//              if(gpio_get(G2) == 0)
//              Direction_P = Direction_P - 2.5;
//              while(!gpio_get(G2));
//          }
//        if(gpio_get(G1) == 0)
//          {
//              systick_delay_ms(10);
//              if(gpio_get(G1) == 0)
//              Direction_D = Direction_D + 0.25;
//              while(!gpio_get(G1));
//              
//          }
//          if(gpio_get(G3) == 0)
//          {
//              systick_delay_ms(10);
//              if(gpio_get(G3) == 0)
//              Direction_D = Direction_D - 0.25;
//              while(!gpio_get(G3));
//              
//          }

//      if(sen_sor[2] == 0)
//      {
//         DisableInterrupts;
//         ftm_pwm_duty(ftm2,ftm_ch1,0);
//         ftm_pwm_duty(ftm2,ftm_ch0,0);
//         ftm_pwm_duty(ftm2,ftm_ch3,0);
//         ftm_pwm_duty(ftm2,ftm_ch2,0);
//         
//      }

   }

}

