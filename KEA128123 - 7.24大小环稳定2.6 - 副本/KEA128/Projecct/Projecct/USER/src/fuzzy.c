#include "fuzzy.h"

#define COUNT 7

float fuzzy_p = 0.0;
float fuzzy_d = 0.0;
extern float Direction_P;

void fuzzy_deal(float offset,float offset_rate)
{
    int err_p,err_rate_d;
    int err,err_rate;
    int dat,i;
    float err_rule[7] = {0.0,4.5,9.0,13.5,17.0,23.5,27.0};        //偏差特征值
    float err_rate_rule[7] = {0.0,4.7,5.8,4.4,5.1,6.5,8.5}; //偏差变化率特征值
    
    
    float p_count[7] = {0.91,0.98,1.0,1.14,1.22,1.28,1.35};       //P隶属函数中心值
    float d_count[7] = {3.5,4.5,5,6.5,7,7.5,9.5};        //d隶属函数中心值
    float err_fuzzy[2] = {0.0,0.0};
    float err_rate_fuzzy[2] = {0.0,0.0};
    float U_p_fuzzy[7] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};       //输出P模糊量
    float U_d_fuzzy[7] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};       //输出D模糊量
    
          
    /*模糊规则*/
    int rule_kp[7][7]=
    {
  //ec 0 1 2 3 4 5 6  //e
      {2,1,1,1,1,0,0},//0    3,2,2,1,1,1,0
      {2,2,1,1,1,1,0},//1    2,2,1,1,1,0,0 
      {3,2,2,2,2,1,1},//2    2,2,1,1,1,0,0 
      {4,4,3,3,2,1,1},//3    4,3,2,1,1,1,0 
      {4,4,4,3,2,2,1},//4
      {5,5,5,4,4,3,2},//5
      {6,6,5,5,4,3,3} //6
    };

    int rule_kd[7][7]=
    {
  //ec 0 1 2 3 4 5 6  //e
      {3,3,4,5,5,6,6},//0
      {3,3,4,4,5,5,5},//1
      {2,3,3,4,5,5,5},//2
      {1,2,2,3,3,4,5},//3
      {1,1,2,2,3,3,3},//4
      {0,1,1,1,2,2,3},//5
      {0,0,1,1,1,2,2} //6
    };  
      err_p = (int)(myabs(offset));
      err_rate_d = (int)(myabs(offset_rate));
    
      fuzzy_p = 0.0;
      fuzzy_d = 0.0;
      /*确定p的隶属度*/
      if(err_p < err_rule[0])
      {
          err_fuzzy[0] = 1.0;
          err = 0;
      }
      else if(err_p < err_rule[1])
      {
          err_fuzzy[0] = (err_rule[1] - err_p) / (err_rule[1] - err_rule[0]);
          err = 0;
      }
      else if(err_p < err_rule[2])
      {
          err_fuzzy[0] = (err_rule[2] - err_p) / (err_rule[2] - err_rule[1]);
          err = 1;
      }
      else if(err_p < err_rule[3])
      {
          err_fuzzy[0] = (err_rule[3] - err_p) / (err_rule[3] - err_rule[2]);
          err = 2;
      }
      else if(err_p < err_rule[4])
      {
          err_fuzzy[0] = (err_rule[4] - err_p) / (err_rule[4] - err_rule[3]);
          err = 3;
      }
      else if(err_p < err_rule[5])
      {
          err_fuzzy[0] = (err_rule[5] - err_p) / (err_rule[5] - err_rule[4]);
          err = 4;
      }
      else if(err_p < err_rule[6])
      {
          err_fuzzy[0] = (err_rule[6] - err_p) / (err_rule[6] - err_rule[5]);
          err = 5;
      }
      else
      {
          err_fuzzy[0] = 1.0;
          err = 6;
      }
      err_fuzzy[1] = 1.0 - err_fuzzy[0]; //相加为1
      
      
      
      
      /*确定d的隶属度*/
      if(err_rate_d < err_rate_rule[0])
      {
          err_rate_fuzzy[0] = 1.0;
          err_rate = 0;
      }
      else if(err_rate_d < err_rate_rule[1])
      {
          err_rate_fuzzy[0] = (err_rate_rule[1] - err_rate_d) / (err_rate_rule[1] - err_rate_rule[0]);
          err_rate = 0;
      }
      else if(err_rate_d < err_rate_rule[2])
      {
          err_rate_fuzzy[0] = (err_rate_rule[2] - err_rate_d) / (err_rate_rule[2] - err_rate_rule[1]);
          err_rate = 1;
      }
      else if(err_rate_d < err_rate_rule[3])
      {
          err_rate_fuzzy[0] = (err_rate_rule[3] - err_rate_d) / (err_rate_rule[3] - err_rate_rule[2]);
          err_rate = 2;
      }
      else if(err_rate_d < err_rate_rule[4])
      {
          err_rate_fuzzy[0] = (err_rate_rule[4] - err_rate_d) / (err_rate_rule[4] - err_rate_rule[3]);
          err_rate = 3;
      }
      else if(err_rate_d < err_rate_rule[5])
      {
          err_rate_fuzzy[0] = (err_rate_rule[5] - err_rate_d) / (err_rate_rule[5] - err_rate_rule[4]);
          err_rate = 4;
      }
      else if(err_rate_d < err_rate_rule[6])
      {
          err_rate_fuzzy[0] = (err_rate_rule[6] - err_rate_d) / (err_rate_rule[6] - err_rate_rule[5]);
          err_rate = 5;
      }
      else
      {
          err_rate_fuzzy[0] = 1.0;
          err_rate = 6;
      }
      err_rate_fuzzy[1] = 1.0 - err_rate_fuzzy[0]; //相加为1
      
      
      
      
      
      /*查询P规则表*/
      if(err < (COUNT - 1) && err_rate < (COUNT - 1))   //偏差及偏差变化率均未到边缘处
      {
          dat = rule_kp[err_rate][err];
          U_p_fuzzy[dat] += err_fuzzy[0] * err_rate_fuzzy[0];
          
          dat = rule_kp[err_rate][err+1];
          U_p_fuzzy[dat] += err_fuzzy[1] * err_rate_fuzzy[0];
          
          dat = rule_kp[err_rate+1][err];
          U_p_fuzzy[dat] += err_fuzzy[0] * err_rate_fuzzy[1];
          
          dat = rule_kp[err_rate+1][err+1];
          U_p_fuzzy[dat] += err_fuzzy[1] * err_rate_fuzzy[1];
      }
      else if(err == (COUNT - 1) && err_rate < (COUNT - 1))     //偏差达到边缘
      {
          dat = rule_kp[err_rate][err];
          U_p_fuzzy[dat] += err_fuzzy[0] * err_rate_fuzzy[0];
          dat = rule_kp[err_rate+1][err];
          U_p_fuzzy[dat] += err_fuzzy[0] * err_rate_fuzzy[1];
      }
      else if(err < (COUNT - 1) && err_rate == (COUNT - 1))     //偏差变化率达到边缘
      {
          dat = rule_kp[err_rate][err];
          U_p_fuzzy[dat] += err_fuzzy[0] * err_rate_fuzzy[0];
          dat = rule_kp[err_rate][err+1];
          U_p_fuzzy[dat] += err_fuzzy[1] * err_rate_fuzzy[0];
          
      }
      else                                                      //同时边缘
      {
          dat = rule_kp[err_rate][err];
          U_p_fuzzy[dat] += err_fuzzy[0] * err_rate_fuzzy[0];
      }
      
      
      
      /*查询d规则表*/
      if(err < (COUNT - 1) && err_rate < (COUNT - 1)) 
      {
          dat = rule_kd[err_rate][err];
          U_d_fuzzy[dat] += err_fuzzy[0] * err_rate_fuzzy[0];
          
          dat = rule_kd[err_rate][err+1];
          U_d_fuzzy[dat] += err_fuzzy[1] * err_rate_fuzzy[0];
          
          dat = rule_kd[err_rate+1][err];
          U_d_fuzzy[dat] += err_fuzzy[0] * err_rate_fuzzy[1];
          
          dat = rule_kd[err_rate+1][err+1];
          U_d_fuzzy[dat] += err_fuzzy[1] * err_rate_fuzzy[1];
      }
      else if(err == (COUNT - 1) && err_rate < (COUNT - 1))
      {
          dat = rule_kd[err_rate][err];
          U_d_fuzzy[dat] += err_fuzzy[0] * err_rate_fuzzy[0];
          dat = rule_kd[err_rate+1][err];
          U_d_fuzzy[dat] += err_fuzzy[0] * err_rate_fuzzy[1];
      }
      else if(err < (COUNT - 1) && err_rate == (COUNT - 1))
      {
          dat = rule_kd[err_rate][err];
          U_d_fuzzy[dat] += err_fuzzy[0] * err_rate_fuzzy[0];
          dat = rule_kd[err_rate][err+1];
          U_d_fuzzy[dat] += err_fuzzy[1] * err_rate_fuzzy[0];
      }
      else
      {
          dat = rule_kd[err_rate][err];
          U_d_fuzzy[dat] += err_fuzzy[0] * err_rate_fuzzy[0];
      }

      /*反模糊*/
      for(i = 0;i < COUNT; i++)
      {
          fuzzy_p += U_p_fuzzy[i] * p_count[i];
          fuzzy_d += U_d_fuzzy[i] * d_count[i];
      }
   // printf("%d",(int)(10.0*fuzzy_p));     printf(",");    printf("%d\n",(int)fuzzy_d);
}
