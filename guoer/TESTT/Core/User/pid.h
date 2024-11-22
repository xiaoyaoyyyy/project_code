#ifndef PID_H
#define PID_H
#include "stm32f4xx_hal.h"



typedef struct
{
	uint8_t	EN;									//PID使能
  float target_val;           //目标值
  float actual_val;        	  //实际值
	float ouput_val;
	float actual_val_last;      //上次实际值
	
  float deviation_val,deviation_val_last,deviation_val_ll;  
							//偏差值  //上次偏差值   //上上次偏差
  float Kp,Ki,Kd,integral;  //比例、积分、微分系数 定义积分值
	float err,  last_err,before_err  ,total_err ;   
			//当前误差 上次误差 上上次误差 误差总和
	float _out;

	
}PID_Typedef;//定义PID参数结构体



extern PID_Typedef Chassis_Motor1_Pid,\
									 Chassis_Motor1_angle_Pid,\

										Chassis_Motor2_Pid,\
										Chassis_Motor2_angle_Pid,\

										Chassis_Motor3_Pid,\
										Chassis_Motor3_angle_Pid,\

										Chassis_Motor4_Pid,\
										Chassis_Motor4_angle_Pid,\

						Chassis_Motor1_speed_Pid,\
						Chassis_Motor2_speed_Pid,\
						Chassis_Motor3_speed_Pid,\
						Chassis_Motor4_speed_Pid,\

						 Uplift_Motor3_Pid,\
						Uplift_Motor3_Angle_Pid,	\
					
						Uplift_Motor4_Pid,\
						Uplift_Motor4_Angle_Pid, \
						
						Stretch_Motor5_Pid,\
						Stretch_Motor5_Angle_Pid,\
						
						Stretch_Motor6_Pid,\
						Stretch_Motor6_Angle_Pid,\
											
						Pitch_Motor7_Pid,\
						Pitch_Motor7_Angle_Pid;\




void pid_set(int16_t taget1,int16_t taget2,int a,int b);
void PID_Init(PID_Typedef *PID);
void PID_SET(PID_Typedef *PID, float target, float P, float I, float D);
float PID_incremental(PID_Typedef *PID, double target, float actual, float limit);
float PID_position(PID_Typedef * PID, float temp_val, float target1, float confine, float limit);
#endif
