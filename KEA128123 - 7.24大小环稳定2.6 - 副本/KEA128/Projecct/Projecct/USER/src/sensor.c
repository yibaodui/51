#include "sensor.h"
#include "fuzzy.h"
#include "fuzzy_wa.h"
#define gyro_x_zero -13
#define K 50000
#define dt_cir 0.005
extern float speed_err;
extern uint8 sector;
float turn_P = 40.5;//133.65 50
float turn_D = 8.5;//-58.5  10
float turn_I = 0.0;//1.0
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
extern int16 car_real_speed_old;
extern float Complement_angle;
int left_1[7],right_1[7],mid[7],right_2[7],left_2[7];
uint16 sen_sor[5];
int32 offset_value_new,offset_value_old;
uint8 temp = 0;
uint8 temp_1=1;
uint16 count_set = 0;
float turn_offset_err = 0.0;
float turn_offset_last = 0.0,err,GYRO_X_L;
double turn_offset;
float offset_center[10];
float D = 0.0;//7.5
float Direction_P = 17.5;              // 108
float Direction_I = 0.0;
float GYRO_X,GYRO_X_last;
float mid_value;
float Distance[30];
float TURN_Inter;
int32 distance[30];
float turn_slope;
float turn_rate_err;
uint8 Fres = 0;
float err,Dire_out;
float turn_output,turn_output_last;
float turn_rate_last_err;
float turn_integral_value = 0;
uint8 flag = 0;
uint8 flag1 = 0;
uint8 flag_count = 0;
double x;
uint16 mid_pre;
int16 center_offset;
int16 the_num;
double sensor_predict;  //电流预测值
float count_num = 0;
uint16 record_count = 0;
float slopee,ofsetss;
float pre_R,pre_R_T;
uint8 flag_left = 0;
uint8 flag_right = 0;
uint8 roundabout_flag = 0;
extern uint8 flag_cho;
extern uint8 flag_adjust;
extern uint8 adjust_count;
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



float offset_get(void)
{
    uint8 i = 0;
    double p1 = -0.001438;
    double p2 = 0.3146;
    double p3 = -9.188;
    double p4 = 153.9;
    
    double p11 = -0.004031;
    double p22 = 0.2523;
    double p33 = -5.645;
    double p44 = 87.41;
    sensor_value_get(); //读取AD通道采集值
    
    sen_sor[0] = sensor_filter_average(left_1) + 2 ; //读取滤波后的值 left
    sen_sor[1] = sensor_filter_average(right_1) + 2 ;//right
    sen_sor[2] = sensor_filter_average(mid) + 2 ;//mid
    sen_sor[3] = sensor_filter_average(left_2) + 2 ; //读取滤波后的值 left
    sen_sor[4] = sensor_filter_average(right_2) + 2;//right
    mid_value = sen_sor[2];
    if(mid_value > 200)     mid_value = 200;   
    turn_offset = 3000.0 * (float)(sen_sor[0] - sen_sor[1]) / (float)((sen_sor[0] + sen_sor[1] + mid_value) * mid_value);
    x = Complement_angle;
    mid_pre = p1 * x * x * x + p2 * x * x + p3 * x + p4;
    if(mid_pre > 240)   mid_pre = 240;
    if(mid_pre < -240)  mid_pre = -240;
    sensor_predict = p11 * x * x * x  + p22 * x * x+ p33 * x + p44;
    if(sensor_predict > 240)   sensor_predict = 240;
    if(sensor_predict < -240)  sensor_predict = -240;
    //sensor_radus = ;
    //err_record[record_count ++] = (sensor_predict/ (float)(sen_sor[2]));
//    if(record_count == 200)
//    {
//        sensor_radus = 1.0 * (err_record[198] + err_record[197] + err_record[196] + err_record[195] +err_record[194]) / 5;
//        record_count = 199;
//    }
 
    if(flag_cho)  roundabout_flag = roundabout_deal_two(sen_sor[2],sen_sor[3],sen_sor[4],sen_sor[0],sen_sor[1]);
    else          roundabout_flag = roundabout_deal(sen_sor[2],sen_sor[3],sen_sor[4],sen_sor[0],sen_sor[1],flag_adjust);
    if(!(roundabout_flag))
    {
      if(!temp)         
      {
          turn_offset = 3000.0 * (float)(sen_sor[0] - sen_sor[1]) / (float)((sen_sor[0] + sen_sor[1] + mid_value) * mid_value);
      }
          flag = 0;
          flag1 = 0;
//          if(turn_offset > 50)        turn_offset =  50;//偏差限幅
//          if(turn_offset < -50)       turn_offset = -50;
    }
//    Get_Gyro();
//    GYRO_X_L = 1.0 * (mpu_gyro_x - gyro_x_zero);
//    pre_R = (float)speed_left * GYRO_X_L * 0.00025;
//    pre_R_T = (float)speed_right * GYRO_X_L * 0.00025;
//    ofsetss = pre_R - pre_R_T;
    for(i=0;i<29;i++)
   {
   
      Distance[i]=Distance[i+1];
   
   }
   Distance[29]=turn_offset;   
   turn_offset=0.8*turn_offset + 0.12*Distance[28] + 0.08*Distance[27];//方向低通滤波
//    printf("%d",sen_sor[0]);    printf(",");    printf("%d",sen_sor[1]);      printf(",");
//    printf("%d",sen_sor[2]);    printf(",");    printf("%d",sen_sor[3]);      printf(",");
//    printf("%d\n",sen_sor[4]);    //  printf(",");
  //  printf("%d",(int)pre_R); printf(",");printf("%d",(int)ofsetss); printf(",");printf("%d\n",(int)pre_R_T);
    return turn_offset;
    

        
}

float Direction_get(int32 turn_offset_value)
{
    uint8 i=0;
    turn_offset_err = turn_offset_value - turn_offset_last;
//    if (car_real_speed_old < 40)
//    {
//            Fres = 0;   
//            Direction_P = 12.5;
//            D = 38.5;//7.5
//    }
//    else if (car_real_speed_old < 50)
//    {
//            Fres = 1;
//            Direction_P = 13.5; 
//            D = 39.5;//7.5
//    }
//    else if (car_real_speed_old < 60)
//    {
//            Fres = 2;
//            D = 40.5;//7.5
//    }
//    else if (car_real_speed_old < 70)
//    {
//            Fres = 3;
//            Direction_P = 14.5;
//            D = 41.5;//7.5
//    }
//    else if (car_real_speed_old >= 80)
//    {
//            Fres = 4;
//            Direction_P = 15.5;
//            D = 42.5;//7.5
//    }
    offset_value_new = Direction_P * turn_offset_value + D  * turn_offset_err + Direction_I * TURN_Inter;///  + max_value * P; 
    turn_offset_last = turn_offset_value;
    for(i=0;i<29;i++)
   {
   
      distance[i]=distance[i + 1];
   
   }
   distance[29]=offset_value_new;   
   offset_value_new=0.8*offset_value_new+0.12*distance[28]+0.08*distance[27];//方向低通滤波
//  if(myabs(ofsetss) > 63)     
//  { 
//    if(offset_value_new > 350) 
//      offset_value_new = 350;
//      if(offset_value_new < -350) 
//      offset_value_new = -350;
//  }
//  /* printf("%d\n",(int)(turn_slope));   /* printf(",");printf("%d",(int)(turn_rate_err));  // printf(",");*/  
//   /*printf("%d",(int)(count_num));    printf(","); */  printf("%d",(int)(count_num));   
//   printf(",");   printf("%d",(int)(sen_sor[2])); printf(",");   printf("%d",(int)(10*slopee));printf(",");printf("%d\n",(int)(100*sensor_radus));
//   

    return offset_value_new;
}


int turn_core_series(float offset)
{      
   GYRO_X = 1.0 * (mpu_gyro_x - gyro_x_zero)/ 16.4;
   GYRO_X = 0.4 * GYRO_X_last + 0.6 * GYRO_X;
   GYRO_X_last = GYRO_X;
   turn_slope = offset_value_new * car_real_speed_old * 0.0175;
   if(turn_slope > 470)     turn_slope = 470;
   if(turn_slope < -470)     turn_slope = -470;
   turn_output_last = turn_output;
   turn_rate_err = ((-turn_slope) + GYRO_X);//
   turn_integral_value +=  turn_rate_err;
   if(turn_integral_value > 400)       turn_integral_value = 400; 
   if(turn_integral_value < -400)       turn_integral_value = -400; 
   turn_output = turn_P * turn_rate_err + turn_D * (turn_rate_err - turn_rate_last_err) + turn_I * turn_integral_value;    
   turn_rate_last_err = turn_rate_err;
   return (int)turn_output;
}


/**************************************/
/**************转向平滑输出*************/
/**************************************/
int Direction_output(uint8 period)
{
    
//    static float turn_out_filter[4]; 
//    turn_out_filter[3]=turn_out_filter[2];
//    turn_out_filter[2]=turn_out_filter[1];
//    turn_out_filter[1]=turn_out_filter[0];
//    turn_out_filter[0]=turn_output;
//    turn_output=turn_out_filter[0]*0.4+turn_out_filter[1]*0.3+turn_out_filter[2]*0.2+turn_out_filter[3]*0.1;
    err = turn_output - turn_output_last;
    Dire_out = err * (period + 1) / 1 + turn_output_last;
    turn_output_last = turn_output;
    if(Dire_out > 8000) Dire_out = 8000;
    if(Dire_out < -8000) Dire_out = -8000;
    //Dire_out = 0;

    return (int)Dire_out;
}

int32 PID_Realize(PID *sptr, float *PID, int32 NowData, int32 Point)
{
	//当前误差，定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
	int32 iError,	// 当前误差
		 Realize;	// 最后得出的实际增量

	iError = Point - NowData;	// 计算当前误差
	sptr->SumError += PID[KI] * iError;	// 误差积分
	if (sptr->SumError >= PID[KT])
	{
		sptr->SumError = PID[KT];
	}
	else if (sptr->SumError <= -PID[KT])
	{
		sptr->SumError = -PID[KT];
	}

	Realize = PID[KP] * iError
			+ sptr->SumError
			+ PID[KD] * (iError - sptr->LastError);
	sptr->PrevError = sptr->LastError;	// 更新前次误差
	sptr->LastError = iError;		  	// 更新上次误差
	sptr->LastData  = NowData;			// 更新上次数据

	return Realize;	// 返回实际值
}
int32 PlacePID_Control(PID *sprt, float *PID, int32 NowPiont, int32 SetPoint)
{
	//定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
	int32 iError,	//当前误差
		  Actual;	//最后得出的实际输出值
	float Kp;		//动态P
	
	iError = SetPoint - NowPiont;	//计算当前误差
	sprt->SumError += iError*0.01;
	if (sprt->SumError >= PID[KT])
	{
		sprt->SumError = PID[KT];
	}
	else if (sprt->SumError <= PID[KT])
	{
		sprt->SumError = -PID[KT];
	}
	Kp = 1.0 * (iError*iError) / PID[KP] + PID[KI];	//P值与差值成二次函数关系，此处P和I不是PID参数，而是动态PID参数，要注意！！！
	
	Actual = Kp * iError
		   + PID[KD] * ((0.8*iError + 0.2*sprt->LastError) - sprt->LastError);//只用PD
	sprt->LastError = iError;		//更新上次误差

//	Actual += sprt->SumError*0.1;
	Actual = range_protect(Actual, -260, 260);

	return Actual;
}


int sumX=0,sumY=0,avrX=0,avrY=0;
int num=0,B_up1=0,B_up2=0,B_up=0,B_down=0;
float regression(int startline,int endline,float X[10]) //最小二乘法计算函数 注意数组范围
{
    float slope=0;//斜率
    int num,i;
    sumX=0;sumY=0;avrX=0;avrY=0 ;
    num=0;B_up1=0;B_up2=0;
    for(i=startline;i<=endline;i++)
    {
         if(X[i]) 
         {
            num++;
            sumX+=i;
            sumY+=X[i];
         }
    }
    avrX=sumX/num;
    avrY=sumY/num;
    B_up=0;
    B_down=0;
    for(i=startline;i<=endline;i++)
    {
	         if(X[i]) 
	         { 
	            B_up1=(int)X[i]-(int)avrY;
	            B_up2=i-avrX;
	            B_up+=(int)B_up1*(int)B_up2;
	            //B_up=B_up/100*100;
	            B_down+=(int)(i-avrX)*(int)(i-avrX);
	         }
   }
   if(B_down==0) slope=0;
   else slope=B_up*10/B_down;
   
   return slope/10.0;
}
/**************************************/
/**************环岛处理****************/
/**************************************/



uint8 roundabout_deal_two(int16 buf,int16 buf_left,int16 buf_right,int16 left,int16 right)
{
    uint8 i; 
    static  float last_value[10]; 
    
    for(i=0;i<9;i++)
     {
          last_value[i]=last_value[i+1];
 
      }
    last_value[9] = buf;  //将中间电感的值存入数组
    slopee = regression(0,9,last_value);
    center_offset = buf - (mid_pre + adjust_count);
    ofsetss = buf_left - buf_right;
    if((left > 0) && (right > 0))//如果左右电感都大于80为条件1防止误判  需要根据实际去调整
    {
          if((myabs(center_offset) < 25))//如果中间电感在此范围内mid_pre为预测中间电感值
          {
             //入环则蜂鸣器响

                if((buf < (left + right)))   //满足入环条件
               { 
                 if(temp_1)       
                  { 
                    count_num  += (float)(car_real_speed_old) * dt_cir;
                     
                    //gpio_set(C6,0);
                  }   
                          
                  if(count_num < -17)
                  {    
                    if(ofsetss < -80)
                    {
                      turn_offset = 13; //      flag_left = 1;}//竖直电感循迹，主要目的实现偏差递增 左环岛 
                      flag_left = 1;
                    }
                    gpio_set(C6,0); //入环则蜂鸣器响 
                    if(ofsetss > 80)
                    {
                       turn_offset = -13;
                       flag_right = 1;
                    }
                    //if(left > right)    {turn_offset = -13;      flag_right = 1;}//竖直电感循迹，主要目的实现偏差递增 左环岛
                    if(turn_offset > 15)        turn_offset = 15;           //防止电感衔接时突变
                    if(turn_offset < -15)       turn_offset = -15;                  
                    flag = 1; //返回标志位flag 若为0则正常循迹
                    flag1 = 1;//
                    count_num = 0;
                  }  
                
                

              }
              
            }
    }

    if(flag1 == 1)             //确为环岛信息
    {
      if(flag_left)        turn_offset = 13; //      flag_left = 1;}//竖直电感循迹，主要目的实现偏差递增 左环岛 
      if(flag_right)        turn_offset = -13; //      flag_left = 1;}//竖直电感循迹，主要目的实现偏差递增 左环岛
     
       //turn_offset = -13;//竖直电感循迹，主要目的实现偏差递增 左环岛
        count_num = 0;
        
    }
    if((left > (buf-3))&& flag1 && flag_left)//如果左边电感与右边电感的差值大于20，表示车已入环完成
    {
        /*恢复正常循迹*/
        turn_offset = 3000.0 * (float)(sen_sor[0] - sen_sor[1]) / (float)((sen_sor[0] + sen_sor[1] + mid_value) * mid_value);
        flag = 0;// 入环完成
        flag1 = 0;// 入环完成
        temp = 1;//完成入环后开启计时
        temp_1 = 0;
        flag_left = 0;
        gpio_set(C6,1); //入环完成后蜂鸣器灭
    }
    if((right > (buf-3)) && flag1 && flag_right)//如果左边电感与右边电感的差值大于20，表示车已入环完成
    {
        /*恢复正常循迹*/
        turn_offset = 3000.0 * (float)(sen_sor[0] - sen_sor[1]) / (float)((sen_sor[0] + sen_sor[1] + mid_value) * mid_value);
        flag = 0; // 入环完成
        flag1 = 0;// 入环完成
        temp = 1;//完成入环后开启计时
        temp_1 = 0;
        flag_right = 0;
        gpio_set(C6,1); //入环完成后蜂鸣器灭
    }
    if(temp == 1)//完成入环后开启计时
    {
        count_set ++;
        //temp_1 = 0;
    }
    if(count_set == 600) // 2.5s后再启动圆环识别
    {
       if(turn_offset > 30)        turn_offset = 30;
       if(turn_offset < -30)       turn_offset = -30;
       temp = 0;          
       temp_1 = 1;
       count_set = 0;
       count_num = 0;
       gpio_set(C6,1); //蜂鸣器灭
       
    } 
   // printf("%d\n",(int)(10*count_num));
    return flag;
/*******************************************************************************************/    
    
}


uint8 roundabout_deal(int16 buf,int16 buf_left,int16 buf_right,int16 left,int16 right,uint8 howflag)
{
    uint8 i; 
    static  int16 last_value[5]; 
    
    for(i=0;i<4;i++)
     {
          last_value[i]=last_value[i+1];
 
      }
    last_value[4] = buf;  //将中间电感的值存入数组
    center_offset = buf - mid_pre;
    if((left > 0) && (right > 0))//如果左右电感都大于80为条件1防止误判  需要根据实际去调整
    {
          if((myabs(center_offset) < 25))//如果中间电感在此范围内mid_pre为预测中间电感值
          {
            if(temp_1)       
            { 
              count_num  += (float)(car_real_speed_old) * dt_cir;
              //gpio_set(C6,0);
            } //入环则蜂鸣器响
            if(howflag == 0)
            {  
                if(count_num < -17)
                {
                      turn_offset = 13; //      flag_left = 1;}//竖直电感循迹，主要目的实现偏差递增 左环岛                            
                      if(turn_offset > 15)        turn_offset = 15;           //防止电感衔接时突变
                      if(turn_offset < -15)       turn_offset = -15;                  
                      flag = 1; //返回标志位flag 若为0则正常循迹
                      flag1 = 1;//
                      flag_left = 1;
                      gpio_set(C6,0); //入环则蜂鸣器响
                      count_num = 0;
                }    

                
             }
            else
            {
                if(count_num < -17)
                {
                      turn_offset = -13; //      flag_left = 1;}//竖直电感循迹，主要目的实现偏差递增 左环岛                            
                      if(turn_offset > 15)        turn_offset = 15;           //防止电感衔接时突变
                      if(turn_offset < -15)       turn_offset = -15;                  
                      flag = 1; //返回标志位flag 若为0则正常循迹
                      flag1 = 1;//
                      flag_right = 1;
                      count_num = 0;
                      gpio_set(C6,0); //入环则蜂鸣器响
                }    
                
                
                
            }
              
            }
    }

    if(flag1 == 1)             //确为环岛信息
    {
       if(flag_left)    turn_offset = 13;//竖直电感循迹，主要目的实现偏差递增 左环岛
       if(flag_right)   turn_offset = -13;//竖直电感循迹，主要目的实现偏差递增 左环岛
       count_num = 0;       
    }

    if((left > (buf-3)) && flag1 && flag_left)//如果左边电感与右边电感的差值大于20，表示车已入环完成
    {
        /*恢复正常循迹*/
        turn_offset = 3000.0 * (float)(sen_sor[0] - sen_sor[1]) / (float)((sen_sor[0] + sen_sor[1] + mid_value) * mid_value);
        flag = 0; // 入环完成
        flag1 = 0;// 入环完成
        temp = 1;//完成入环后开启计时
        temp_1 = 0;
       flag_left = 0;
//        flag_right = 0;

        gpio_set(C6,1); //入环完成后蜂鸣器灭
    }
    if((right > (buf-3)) && flag1&&flag_right)//如果左边电感与右边电感的差值大于20，表示车已入环完成
    {
        /*恢复正常循迹*/
        turn_offset = 3000.0 * (float)(sen_sor[0] - sen_sor[1]) / (float)((sen_sor[0] + sen_sor[1] + mid_value) * mid_value);
        flag = 0; // 入环完成
        flag1 = 0;// 入环完成
        temp = 1;//完成入环后开启计时
        temp_1 = 0;
//        flag_left = 0;
        flag_right = 0;

        gpio_set(C6,1); //入环完成后蜂鸣器灭
    }
    if(temp == 1)//完成入环后开启计时
    {
//       if(turn_offset > 15)        turn_offset = 15;
//       if(turn_offset < -15)       turn_offset = -15;
        count_set ++;
    }
    if(count_set == 600) // 2.5s后再启动圆环识别
    {
       if(turn_offset > 30)        turn_offset = 30;
       if(turn_offset < -30)       turn_offset = -30;
       temp = 0;          
       temp_1 = 1;
       count_set = 0;
       count_num = 0;
       gpio_set(C6,1); //蜂鸣器灭
       
    }                
    return flag;
/*******************************************************************************************/    
    
}


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

    
    

//    next_angle_dot =(float)(5.5*car_real_speed) / ((float)(GYRO_X));
////    next_dot = 9.0 * ((float)(GYRO_X)) / (float)(car_real_speed);     //计算实际转向曲率
//
//    
//      next_dot = 1.0 / next_angle_dot;
//      if(myabs(car_real_old) < 2)     next_dot= 0;       //如果左右轮差速小于8则实际转向曲率为0    
//      if(next_dot > 0.2)         next_dot = 0.2;//实际转向曲率限幅
//    if(next_dot < -0.2)       next_dot = -0.2;
//      pre_R_T = (p1 * turn_offset + p2) / (turn_offset * turn_offset + q1 * turn_offset + q2);
//      pre_R = 1.0 / pre_R_T;
//      if(pre_R > 0.25)         pre_R = 0.25;//期望曲率限幅
//    if(pre_R < -0.25)       pre_R = -0.25;
//    max_value_last = max_value;//保存曲率差值
//      if(myabs((int16)turn_offset) < 1)  pre_R = 0;//如果偏差小于1则期望曲率为0
////    if(pre_R_T > 30)        pre_R_T = 30;//期望曲率限幅
////    if(pre_R_T < -30)       pre_R_T = -30;
//    max_value = (100.0*next_dot)  - (100.0*pre_R);     //转向曲率偏差量
//    err = max_value - max_value_last;
//    if(max_value > 60.5)        max_value = 60.5;//偏差量限幅
//    if(max_value < -60.5)       max_value = -60.5;
    
//    printf("%d",sen_sor[0]);    printf(",");    printf("%d",sen_sor[1]);      printf(",");
//      printf("%d",sen_sor[2]); printf(",");   printf("%d",sen_sor[3]); printf(","); printf("%d\n",sen_sor[4]);
//    printf(","); printf("%d\n",(int)turn_offset);
      //     printf("%d",(int)turn_offset);      printf(","); printf("%d\n",(int)turn_offset_err);
//    printf("%d",(int)next_dot);    printf(",");  printf("%d",(int)pre_R_T);   
//    printf(","); printf("%d",(int)car_real_old);printf(",");
//     printf("%d",(int)(100*next_dot));printf(",");
//    printf("%d",(int)(100*pre_R));printf(","); printf("%d",(int)(max_value));printf(","); 
    //printf("%d\n",(int)turn_offset);
//       for(i=0;i<9;i++)
//       {
//            offset_center[i]=offset_center[i+1];
//   
//        }
//    
//        offset_center[9]=turn_offset;