#include "pid.h"
#include "niming.h"
#include "CAN_receive.h"
#include "chassis.h"
#include <stdlib.h>
#include <math.h>

extern PID_chassis_TypeDef target,output;
void PID_SET(PID_Typedef *PID, float target, float P, float I, float D)
{
  PID->target_val = target;   //设置初始目标值

  PID->Kp = P;//设置比例值
  PID->Ki = I;//设置积分值
  PID->Kd = D;//设计微分值
}


void PID_Init(PID_Typedef *PID)
{
	PID->target_val = 0.0;
  PID->actual_val=0.0;
  PID->deviation_val = 0.0;
  PID->deviation_val_last = 0.0;
  PID->deviation_val_ll = 0.0;
  
	PID->err = 0.0;
  PID->last_err=0.0;
  PID->before_err = 0.0;
  PID->total_err = 0.0;
  PID->integral = 0.0;
	
  PID->Kp = 0.0;
  PID->Ki = 0.0;
  PID->Kd = 0.0;

}
float err;
float PID_incremental(PID_Typedef *PID, double target, float actual, float limit)
{	
	/*PID运算输出值--PID_OUT; 积分运算值--I_OUT; 比例运算值--P_OUT; 微分运算值--D_OUT; 积分动态限幅值*/
	float increment_val=0, Proportion_val=0, Integral_val=0, Differential_val=0;
  //计算目标值与实际值的误差
	 err=target-actual;
//	if(fabs(target-actual)>30)
//	{
	PID->target_val=target;
	PID->actual_val=actual;
	PID->deviation_val = (target - actual);
//	}
//	else PID->deviation_val =0;
  //PID算法实现
	Proportion_val = PID->Kp * (PID->deviation_val - PID->deviation_val_last);	//比例运算: Kp * (当前误差值 - 上次误差值)
	Integral_val   = PID->Ki * PID->deviation_val;	//积分运算: Ki * 当前误差值
	Differential_val = PID->Kd * (PID->deviation_val - 2*PID->deviation_val_last + PID->deviation_val_ll);	//微分运算: Kd * （当前误差值 - 2*上次误差值 + 上上次误差值）
	
  increment_val = Proportion_val+ Integral_val + Differential_val; //P_OUT +I_OUT +D_OUT
  
  PID->ouput_val += increment_val;//累加计算输出值
  PID->ouput_val = (PID->ouput_val <= -limit) ? -limit : (PID->ouput_val >=  limit) ?  limit : PID->ouput_val;	
  //传递误差
	PID->deviation_val_ll = PID->deviation_val_last;
	PID->deviation_val_last = PID->deviation_val;

	//返回当前实际值
	return PID-> ouput_val;
}


float PID_position(PID_Typedef * PID,float target1, float temp_val,  float confine, float limit)
 {
//float limit_siqu=70
	/*计算目标值和实际值的误差*/
	 PID->target_val=target1;
	 PID->actual_val= temp_val;
	PID->err = target1 -temp_val;
	/*积分累加*/ 
	PID-> total_err += PID->err;
	 
	/*积分限幅*/
	PID->total_err = PID->total_err >= confine ? confine:PID->total_err;
	PID->total_err = PID->total_err <= -confine ? -confine:PID->total_err;
	 
	/*位置式PID的实现*/ 
	PID->ouput_val = PID->Kp * PID->err 
									+ PID->Ki * PID-> total_err 
									+ PID->Kd *( PID->err-PID->last_err);
	      
 	/*输出值限幅*/
	PID->ouput_val = PID->ouput_val >= limit ? limit:PID->ouput_val;
	PID->ouput_val = PID->ouput_val <= -limit ? -limit:PID->ouput_val;
//	 	
//	 if(abs(PID->err) <= abs(limit_siqu))
//	{
//	  PID->total_err=0;
//	  PID->ouput_val=0;
//	} 
//	 
//	 
//	printf("实际=%d,目标=%d, 输出=%d, KP = %d ,KI = %d , KD = %d\r\n",temp_val,target,PID->actual_val,);
	 
	PID->last_err=PID->err;
	return PID->ouput_val;
	
	// PID->actual_val=0;
 }


void pid_set(int16_t taget1,int16_t taget2,int a,int b)
{
		send_sensorData(0,
										taget1, 
		                taget2,		             
		                motor_chassis[a].total_angle,
										motor_chassis[b].speed_rpm,
										0);	
}


