//#include "ADRC.h"
//#include "math.h"
//
//
//double x1 = 0;
//double x2 = 0;
//
//double z1 = 0;
//double z2 = 0;
//double z3 = 0;
//
//double b0 = 120;
//double h = 0.02;
//double h0 = 0.1;
//double r = 4000;
//double beta_01 = 50;
//double beta_02 = 35;
//double beta_03 = 200000;
///***********************************************************************
//*函数名：double ADRC_Sign(double Input)
//*传入数据:
//*返回数据：void
//*作用：速度PID算法
//*作者：王建锋
//*日期:2017/4/23
//************************************************************************/
//double ADRC_Sign(double Input)
//{
//    double OutPut = 0;
//    if(Input > 1E-8)
//    {
//        OutPut = 1;
//    }
//    else if(Input < -1E-8)
//    {
//        OutPut = -1;
//    }
//    else
//    {
//        OutPut = 0;
//    }
//    return OutPut;
//}
///***********************************************************************
//*函数名：double ADRC_Sat(double x,double Delta)
//*传入数据:
//*返回数据：void
//*作用：速度PID算法
//*作者：王建锋
//*日期:2017/4/23
//************************************************************************/
//double ADRC_Sat(double x,double Delta)
//{
//    double OutPut = 0;
//    if(fabs(x) <= Delta)
//    {
//        OutPut = x / Delta;
//    }
//    else
//    {
//        OutPut = ADRC_Sign(x);
//    }
//    return OutPut;
//}
///***********************************************************************
//*函数名：double ADRC_Fhan(double x1,double x2,double r,double h0)
//*传入数据:
//*返回数据：void
//*作用：速度PID算法
//*作者：王建锋
//*日期:2017/4/23
//************************************************************************/
//double ADRC_Fhan(double x1,double x2,double r,double h0)
//{
//    double d = r*h0;
//    double d0 = h0 * d;
//    double y = x1 + h0*x2;
//    double a0 = sqrt(d*d + 8*r*fabs(y));
//    double a = 0;
//    if(fabs(y) > d0)
//    {
//        a = x2 + (a0 - d)/2 * ADRC_Sign(y);
//    }
//    else
//    {
//        a = x2 + y / h0;
//    }
//    
//    return -r * ADRC_Sat(a,d);
//}
///***********************************************************************
//*函数名：void TD(double *x1,double *x2,double Input,double r,double h,double h0)
//*传入数据:
//*返回数据：void
//*作用：速度PID算法
//*作者：王建锋
//*日期:2017/4/23
//************************************************************************/
//void TD(double *x1,double *x2,double Input,double r,double h,double h0)
//{
//    double temp = 0;
//    temp = *x1 + h * *x2;
//    *x2 = *x2 + h * ADRC_Fhan(*x1 - Input,*x2,r,h0);
//    *x1 = temp;
//}
///***********************************************************************
//*函数名：double  ADRC_Nlsef(double e1,double e2,double c,double r,double h)
//*传入数据:
//*返回数据：void
//*作用：速度PID算法
//*作者：王建锋
//*日期:2017/3/23
//************************************************************************/
//double  ADRC_Nlsef(double e1,double e2,double c,double r,double h)
//{
//    return -ADRC_Fhan(e1,e2*c,r,h);
//}
///***********************************************************************
//*函数名：double ADRC_Fal(double error,double Pow,double threshold)
//*传入数据:
//*返回数据：void
//*作用：速度PID算法
//*作者：王建锋
//*日期:2017/4/23
//************************************************************************/
//double ADRC_Fal(double error,double Pow,double threshold)
//{
//    double OutPut = 0;
//    if(fabs(error) > threshold)
//    {
//        OutPut = pow(fabs(error),Pow) * ADRC_Sign(error);
//    }
//    else
//    {
//        OutPut = error/pow(threshold,Pow);
//    }
//    
//    return OutPut;
//}
///***********************************************************************
//*函数名：void ADRC_ES0(double *z1,double *z2,double *z3,double beta_01,double beta_02,double beta_03,double input,double b0,double output,double h,double threshold)
//*传入数据:
//*返回数据：void
//*作用：速度PID算法
//*作者：王建锋
//*日期:2017/4/23
//************************************************************************/
//void ADRC_ES0(double *z1,double *z2,double *z3,double beta_01,double beta_02,double beta_03,double input,double b0,double output,double h,double threshold)
//{
//    double e = *z1 - output;
//    double fe = ADRC_Fal(e,0.5,threshold);
//    double fel = ADRC_Fal(e,0.25,threshold);
//    
//    *z1 = *z1 + h * (*z2 - beta_01 * e);
//    
//    *z2 = *z2 + h * (*z3 - beta_02 * fe + b0 * input);
//    
//    *z3 = *z3 + h * (-beta_03 * fel);
//}
///***********************************************************************
//*函数名：void ADRC_Control()
//*传入数据:
//*返回数据：void
//*作用：速度PID算法
//*作者：王建锋
//*日期:2017/4/23
//************************************************************************/
//void ADRC_Control()
//{
//    double OutPut = 0;
//    double error1 = 0;
//    double error2 = 0;
//    
//    TD(&x1,&x2,121,r,h,h0);
//    error1 = x1 - z1;
//    error2 = x2 - z2;
//    OutPut = ADRC_Nlsef(error1,error2,0.25,10,0.02);
//    OutPut = OutPut - z3/b0;
//    
//    if(OutPut > MAXPWM)
//    {
//        OutPut = MAXPWM;
//    }
//    else if(OutPut < -MAXPWM)
//    {
//        OutPut = -MAXPWM;
//    }
//    MotorControl((int16_t)OutPut,0);
//    
//    ADRC_ES0(&z1,&z2,&z3,beta_01,beta_02,beta_03,OutPut,b0,Speed*0.00876*50,h,0.5);
//}
