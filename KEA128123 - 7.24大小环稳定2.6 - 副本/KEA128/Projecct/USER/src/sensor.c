#include "sensor.h"
#include "fuzzy.h"
#include "fuzzy_wa.h"
#define gyro_x_zero -13
#define K 50000
extern int16 speed_left;
extern int16 speed_right;
extern int16 car_real_speed;
extern float speed_err;
float turn_output,turn_output_last;
struct pid_out{
  
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

struct pid_out Direction;

extern float fuzzy_p;
extern float fuzzy_d;
int left_1[7],right_1[7],mid[7],right_2[7],left_2[7];
uint16 sen_sor[5];;
float offset_value_new,offset_value_old;
uint8 temp = 0,temp_1=0;
int16 count_set = 0;
void sensor_init(void)
{
    adc_init(ADC0_SE13);
    adc_init(ADC0_SE12);
    adc_init(ADC0_SE14);
    adc_init(ADC0_SE15);
    adc_init(ADC0_SE11);
    adc_init(ADC0_SE10);
    
}

void sensor_value_get(void)
{
    uint8 i;
    for(i = 0; i < 7; i++)
    {
        left_1[i] = adc_once(ADC0_SE12,ADC_CLS);
        right_1[i] = adc_once(ADC0_SE15,ADC_CLS);
        mid[i] = adc_once(ADC0_SE11,ADC_CLS);
        right_2[i] = adc_once(ADC0_SE10,ADC_CLS);
        left_2[i] = adc_once(ADC0_SE14,ADC_CLS);
    }
}
/*冒泡法中值均值滤波*/
int sensor_filter_average(int buf[7])
{
    int buf_value[7],val,mid_value;
    uint8 i,j;
    for(i = 0; i < 7; i++)
    {
        buf_value[i] = buf[i];
    }
    i = 0;
    for(j = 0; i < 7; j++)
    {
        for(i = 0; i < 7 - j; i++)
        {
            if(buf_value[i] > buf_value[i + 1])
            {
                mid_value = buf_value[i];  buf_value[i] = buf_value[i + 1]; buf_value[i + 1] = mid_value;
            }
        }
    }
    val = (buf_value[3]+buf_value[4]+buf_value[5]) / 3;
    return val;
}

//int recursion_median_filtering(int *buf)
//{
//   int buff[7],i;
//   buf[i++] = adc_once(ADC0_SE12,ADC_CLS);
//   
//}

float turn_offset_err = 0.0,turn_offset_last = 0.0,err,GYRO_X_T;
double turn_offset;
float next_angle_dot,next_dot,pre_R_T;
//float p = 0.000,b = 69.5;
float D = 0;
float Direction_P = 80;              // 108
//float Direction_D = 17.7;//25
float GYRO_X,GYRO_X_last;
uint8 flag = 0,flag1 = 0;
double max_value;
float car_real_old,pre_R;
float max_value_last;
int16 mid_value;
float offset_center[10];
float offset_get(void)
{
     
    uint8 i = 0;
     double  p1 = 4.239;
     double  p2 = 4.94;
     double  p3 = -7.92;
     double  q1 = -0.8563;
     double  q2 = -0.04062;
    sensor_value_get(); //读取AD通道采集值
    
    sen_sor[0] = sensor_filter_average(left_1) + 2 ; //读取滤波后的值 left
    sen_sor[1] = sensor_filter_average(right_1)+ 2 ;//right
    sen_sor[2] = sensor_filter_average(mid)+ 2 ;//mid
    sen_sor[3] = sensor_filter_average(left_2)+ 2 ; //读取滤波后的值 left
    sen_sor[4] = sensor_filter_average(right_2) + 2;//right
    car_real_speed = (speed_left + speed_right) / 2;
    mid_value = sen_sor[2];
    if(mid_value > 250)     mid_value = 250;   
    turn_offset = 3000.0 * (float)(sen_sor[0] - sen_sor[1]) / (float)((sen_sor[0] + sen_sor[1] + mid_value) * mid_value);
// 
//       if(sen_sor[2] == 2)//冲出赛道保护
//      {
//         DisableInterrupts;
//         ftm_pwm_duty(ftm2,ftm_ch1,0);
//         ftm_pwm_duty(ftm2,ftm_ch0,0);
//         ftm_pwm_duty(ftm2,ftm_ch3,0);
//         ftm_pwm_duty(ftm2,ftm_ch2,0);
//         while(1);
//         
//      }
      
//    if(!(roundabout_deal(sen_sor[2],sen_sor[3],sen_sor[4],sen_sor[0],sen_sor[1])))
//    {
//          Direction_P = 28.2;              // 108
//          turn_offset = -(1.0 * K / 50.0 *(1.0 / sen_sor[0] - 1.0 / sen_sor[1]));
//         // turn_offset = 3000.0 * (float)(sen_sor[0] - sen_sor[1]) / (float)((sen_sor[0] + sen_sor[1] + mid_value) * mid_value);
//          flag = 0;
//    }
    car_real_old = speed_left - speed_right;    //计算左右轮差速
    Get_Gyro();
    GYRO_X_T = 1.0 * (mpu_gyro_x - gyro_x_zero) / 70.4;//更新x轴角速度 
    pre_R_T = (p1 * turn_offset * turn_offset + p2 * turn_offset + p3) / (turn_offset * turn_offset + q1 * turn_offset + q2);
    next_angle_dot = 4.0*(float)car_real_speed / (float)GYRO_X_T;
    if(turn_offset > 50)        turn_offset =  50;//偏差限幅
    if(turn_offset < -50)       turn_offset = -50;
    pre_R = (float)150.0 / pre_R_T;
    next_dot = (float)150.0 / next_angle_dot;
    if(myabs(turn_offset) < 1)
      next_angle_dot = 0;
    max_value = pre_R - next_dot;
    if(next_angle_dot > 100)
    {
        next_angle_dot=100;
    }
    if(pre_R_T < -100)
    {
        pre_R_T = -100;
    }
    if(pre_R_T > 100)
    {
        pre_R_T=100;
    }
    if(pre_R_T < -100)
    {
        pre_R_T = -100;
    }
//    printf("%d",sen_sor[0]);    printf(",");    printf("%d",sen_sor[1]);      printf(",");
//    printf("%d",sen_sor[2]); printf(",");   printf("%d",sen_sor[3]); printf(","); printf("%d\n",sen_sor[4]);
    printf("%d",(int)turn_offset);      printf(","); printf("%d\n",(int)(100*next_angle_dot));
//    printf("%d",(int)next_dot);    printf(",");  printf("%d",(int)pre_R_T);   
//    printf(","); printf("%d",(int)car_real_old);printf(",");
//    printf("%d",(int)max_value);printf(","); printf("%d\n",(int)turn_offset);
       for(i=0;i<9;i++)
       {
            offset_center[i]=offset_center[i+1];
   
        }
    
        offset_center[9]=turn_offset;
    
    return turn_offset;
    

        
}
float Direction_get(float turn_offset_value)
{
//    float D_slope,y_p;
//    D_slope = regression(0,9,offset_center);
    //y_p = Fuzzy(turn_offset_value,2.5*D_slope); 
   Get_Gyro();
   GYRO_X = 1.0 * (mpu_gyro_x - gyro_x_zero) / 70.6;

   if(GYRO_X > 110) GYRO_X =  110;
   if(GYRO_X < - 110) GYRO_X = -110;
    turn_offset_last = turn_offset_value;
     offset_value_old = offset_value_new;
    offset_value_new = Direction_P * max_value + GYRO_X*D;;//  + max_value * P; 
    
//    if(offset_value_new > 225) 
//       offset_value_new = 225;
//    if(offset_value_new < -225)
//       offset_value_new = -225;
//    if(myabs(speed_err) > 14)
//    {
//        if(offset_value_new > 190) 
//          offset_value_new = 190;
//        if(offset_value_new < -190)
//          offset_value_new = -190;
//    }
//    if(myabs(speed_err) > 18)
//    {
//        if(offset_value_new > 180) 
//          offset_value_new = 180;
//        if(offset_value_new < -180)
//          offset_value_new = -180;
//    }
////
//
   return offset_value_new;
}

//int turn_core_series(float offset)
//{
////      ADRC_Control(offset);
////      turn_output=(int16_t)OutPut;
////     printf("%d",(int)offset); printf(",");  printf("%d\n",(int)turn_output);
////      return turn_output;
//   float turn_P = 30.65;
//   float turn_D = 0.5;
//   float turn_I = 0.0;
//   float turn_rate_err;
//   float turn_rate_last_err;
//    
//
//   turn_output_last = turn_output;
//   turn_rate_err = offset_value_new + GYRO_X*2.3;
//   //fuzzy_deal(max_value,err);    //模糊计算对应的P值
//   turn_output = turn_P * turn_rate_err + turn_D * (turn_rate_err - turn_rate_last_err);
//    //turn_output =  max_value * fuzzy_p* P+ GYRO_X*D + turn_D * (max_value - max_value_last);
//    
//   turn_rate_last_err = turn_rate_err;
//   GYRO_X_last = GYRO_X;
//   //printf("%d\n",(int)GYRO_speed);
//   return turn_output;
//}


/**************************************/
/**************转向平滑输出*************/
/**************************************/

int Direction_output(uint8 period)
{
    float err,Dire_out;
    static float turn_out_filter[4];
    
     
    turn_out_filter[3]=turn_out_filter[2];
    turn_out_filter[2]=turn_out_filter[1];
    turn_out_filter[1]=turn_out_filter[0];
    turn_out_filter[0]=offset_value_new;
    offset_value_new=turn_out_filter[0]*0.4+turn_out_filter[1]*0.3+turn_out_filter[2]*0.2+turn_out_filter[3]*0.1;
    err = offset_value_new - offset_value_old;
    Dire_out = err * (period + 1) / 1 + offset_value_old;

    if(Dire_out > 2600) Dire_out = 2600;
    if(Dire_out < -2600) Dire_out = -2600;
    //Dire_out = 0;

    return (int)Dire_out;
}


/**************************************/
/**************环岛处理****************/
/**************************************/


//uint8 flag_count = 0;
//uint8 i=0;
//uint8 roundabout_deal(int16 buf,int16 buf_left,int16 buf_right,int16 left,int16 right)
//{
//
//    static  int16 last_value[5];     
//    for(i=0;i<5;i++)
//     {
//          last_value[i]=last_value[i+1];
// 
//      }
//    last_value[4]=buf;
//    if((left > 90) && (right > 90))
//    {
//        if(buf > 175 && buf < 210 )
//        {       
//          if(buf_left>70 || buf_right > 60)
//          {
//            if(last_value[4] < last_value[2] && last_value[2] < last_value[0])  //满足入环条件
//            {           
//                //turn_offset = (float)(sen_sor[0] + sen_sor[1] -  sen_sor[2]-(car_real_speed*0.87)) / (float)(sen_sor[0] + 15 +sen_sor[1] + sen_sor[2]) * 100;
//                turn_offset = (float)(buf_left -  buf_right - car_real_speed*1.75) / (float)(buf_left +  buf_right) * 100;
//                Direction_P = 60.0;              
//                flag = 1;                               //返回标志位flag
//                flag1 = 1;
//            }
//          }
//        }
//     }
//    if(flag1 == 1 && (!flag_count))
//    {
//        //turn_offset = (float)(sen_sor[0] + sen_sor[1] -  sen_sor[2] - (car_real_speed*0.87)) / (float)(sen_sor[0]  + 15+ sen_sor[1] + sen_sor[2]) * 100;
//        turn_offset = (float)(buf_left -  buf_right - car_real_speed*1.75) / (float)(buf_left + buf_right) * 100;
//        //turn_offset = (float)(sen_sor[2] -  sen_sor[4] );
//        Direction_P = 60.0;              // 108
//        
//    }
//    if(((sen_sor[0] + 25) > sen_sor[2]) && flag)                //车已入环
//    {
//        turn_offset = 3000.0 * (float)(sen_sor[0] - sen_sor[1]) / (float)((sen_sor[0] + sen_sor[1] + mid_value) * mid_value);
//        Direction_P = 27.4;              // 108
//        flag = 0; // 入环完成
//        flag1 = 0;
//        temp = 1;
//        temp_1 = 0;        
//    }
//
//        if(temp == 1)
//        {
//            count_set ++;
//            flag_count = 1;
//        }
//        if(count_set == 500)
//        {
//          
//           temp = 0;          
//           count_set = 0;
//           temp_1 = 1;
//           
//        }  
//        if((left > 95) && (right > 95)&&temp_1)           flag_count = 0;
//    return flag;
///*******************************************************************************************/    
//    
//}


/*丢线处理*/

//uint8 lose_line_deal(int16 offset)
//{
//    if(sen_sor[0] + sen_sor[1] +  sen_sor[2] < LOSETHRESHOLD )
//}







//    last_value[i++] = sen_sor[2];
//    if (sen_sor[2] > 180 && sen_sor[2] < 236 && (sen_sor[3] + sen_sor[4]) > 200)   max_value = sen_sor[2];
//    if(last_value[i - 1] + 30 < max_value)
//    {
//        turn_offset = (float)(sen_sor[0] + sen_sor[2] -  sen_sor[1] - 20) / (float)(sen_sor[0] + 20 + sen_sor[1] + sen_sor[2]) * 100;
//        Direction_P = 100.5;              // 108
//        Direction_D = 25.7;//25
//    }
//    if(sen_sor[0] > sen_sor[2] + 25)
//    {
//        turn_offset = (float)(sen_sor[0] - sen_sor[1]) / (float)(sen_sor[0] + sen_sor[1] + sen_sor[2]) * 100;
//        Direction_P = 108.5;              // 108
//        Direction_D = 25.7;//25
//        if(turn_offset > 21)    turn_offset = 21;
//        if(turn_offset < -21)    turn_offset = -21;
//        flag = 1; // 入环完成
//    }
//    if(flag)
//    {
//          Direction_P = 108.5;              // 108
//          Direction_D = 25.7;//25
//          turn_offset = (float)(sen_sor[0] - sen_sor[1]) / (float)(sen_sor[0] + sen_sor[1] + sen_sor[2]) * 100;
//    }

//    if(i == 5)   i = 0; 


//float cal_deviation(uint16 * sensor_value)
//{
//	float amp = 10000.0f;		//放大倍数
//	float pwr_total = 0.0f;
//	float position = 0.0f;
//	float ad_sum = 0.0f;
//	float sensor_value_copy[6] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};  
//	float ad_cal[4] = {0.0f,0.0f,0.0f,0.0f};
//	float minus[3] = {0.0f,0.0f,0.0f};
//	float multiply[3] = {0.0f,0.0f,0.0f};
//	float line[3] = {0.0f,0.0f,0.0f};
//	float pwr[3] = {0.0f,0.0f,0.0f};
//	float kp[3] = {0.0f,0.0f,0.0f};
//	
//	for(uint8 i = 0 ; i<4 ; i++)
//	{
//		sensor_value_copy[i] = (float)(sensor_value[i]);		//将原数组复制一份，以便处理 
//	}
//	 
//	sensor_value_copy[0] += 2.5f;		//输入有值为0时会发生计算错误，所以每个值都加上1 
//	sensor_value_copy[1] += 2.5f;
//	sensor_value_copy[2] += 2.5f;
//	sensor_value_copy[3] += 2.5f;
////	sensor_value_copy[4] += 2.5f;
////	sensor_value_copy[5] += 2.5f;
//	
//	ad_cal[0] = sensor_value_copy[0];
//        ad_cal[1] = sensor_value_copy[1];
//	ad_cal[2] = sensor_value_copy[2];
//        ad_cal[3] = sensor_value_copy[3];		
//
////        ad_sum = ad_cal[0] + ad_cal[1] + ad_cal[2] + ad_cal[3];
//        
////	ad_cal[0] = (ad_cal[0] / ad_sum) * 295.0f ;		//这个不懂
////	ad_cal[1] = (ad_cal[1] / ad_sum) * 295.0f ;
////	ad_cal[2] = (ad_cal[2] / ad_sum) * 295.0f ;
////	ad_cal[3] = (ad_cal[3] / ad_sum) * 295.0f ;
//	
//	minus[0] = (int)((ad_cal[1] - ad_cal[0]) * amp);		//相邻两个电感做差
//	minus[1] = (int)((ad_cal[2] - ad_cal[1]) * amp);
//	minus[2] = (int)((ad_cal[3] - ad_cal[2]) * amp);
//
//	multiply[0] =((int)(ad_cal[0] + 10.0f)) * ((int)(ad_cal[1] + 10.0f));			//相邻两个电感相乘
//	multiply[1] =((int)(ad_cal[1] + 10.0f)) * ((int)(ad_cal[2] + 10.0f));
//	multiply[2] =((int)(ad_cal[2] + 10.0f)) * ((int)(ad_cal[3] + 10.0f));
//
//	line[0] = minus[0] / multiply[0];			//差比积算出三条中线
//	line[1] = minus[1] / multiply[1];
//	line[2] = minus[2] / multiply[2];
//        
////        printf("%5d %5d %5d\r\n",(int)(line[0]*100),(int)(line[1]*100),(int)(line[2]*100));
//        
//	line[0] = line[0] - 155.77;				//以第二条中线为基准做中线漂移，三线合一
//	line[1] = line[1];
//	line[2] = line[2] - 156.60f;
//
//	pwr[0] = (int)(ad_cal[0] + ad_cal[1]);			//相邻两个电感做和
//	pwr[1] = (int)(ad_cal[1] + ad_cal[2]);
//	pwr[2] = (int)(ad_cal[2] + ad_cal[3]);
//
//	pwr[0] = pwr[0] * pwr[0];				//上一步做出的和值再平方
//	pwr[1] = pwr[1] * pwr[1];
//	pwr[2] = pwr[2] * pwr[2];
//
//	pwr_total = pwr[0] +pwr[1] + pwr[2];		//算出上一步三个平方的和
//
//	kp[0] =((float)(pwr[0])) / ((float)(pwr_total));			//三个差分别比上总和得出三个比例
//	kp[1] =((float)(pwr[1])) / ((float)(pwr_total));			//这个比例用于动态拟合最终的中线
//	kp[2] =((float)(pwr[2])) / ((float)(pwr_total));			//为啥这么算我也不知道
//
//	position = ((kp[0] * line[0]) + (kp[1] * line[1]) + (kp[2] * line[2])) * 1;
//        //printf("%d",(int)line[0]); printf(",");printf("%d",(int)line[1]); printf(",");printf("%d\n",(int)line[2]);
//        printf("%d\n",(int)(position + 70));
//	return position;
//}
//





//    if(last_value[i - 1] < next_value)
//    {  
//        if(next_value < next_two_value)
//        { 
//            if(next_two_value < next_third_value)
//            {
//                if(next_third_value > 500)
//                {
//                      turn_offset = (float)(sen_sor[0] + sen_sor[1] -  sen_sor[2] + 50) / (float)(sen_sor[0] + 5 + sen_sor[1] + sen_sor[2]) * 100;
//                      Direction_P = 140.5;              // 108
//                      Direction_D = 25.7;//25
//                      if(turn_offset > 35)    turn_offset = 35;
//                      if(turn_offset < -35)    turn_offset = -35;     
//                      flag = 1;                               //返回标志位flag
//                }
//            }
//        }
//      }
//      next_third_value = next_two_value;
//      next_two_value = next_value;
//      next_value = last_value[i - 1];
//      
//    if(last_value[i - 1] > next_value_1)
//    {  
//        if(next_value_1 > next_two_value_1)
//        { 
//            if(next_two_value_1 > next_third_value_1)
//            {
//                if(last_value[i - 1] > max_value)
//                {
//                      turn_offset = (float)(sen_sor[0] -  sen_sor[1] - 10) / (float)(sen_sor[0] + 5 + sen_sor[1] + sen_sor[2]) * 100;
//                      Direction_P = 145.5;              // 108
//                      Direction_D = 25.7;//25
//                      if(turn_offset > 35)    turn_offset = 35;
//                      if(turn_offset < -35)    turn_offset = -35;     
//                      flag = 1;                               //返回标志位flag
//                }
//            }
//        }
//      }
//      next_third_value_1 = next_two_value_1;
//      next_two_value_1 = next_value_1;
//      next_value_1 = last_value[i - 1];

