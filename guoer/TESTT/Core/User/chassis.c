#include "chassis.h"
#include "pid.h"
#include "CAN_receive.h"
#include "usart.h"
#include "sbus.h"
#include "mode_choic.h"
#include "superstratum.h"
#include <math.h>
#include "auto.h"
#define WHEEL_PERIMETER 1	//轮子周长
#define CHASSIS_DECELE_RATIO 19		//3508减速比
#define Radius  20
#define PI 3.1415926535

chassis_6020_speed AGV_speed;
int get_acoder_flag_=0;
int16_t sup_motor_T[4];
int16_t sup_motor_O[4];
double  x,y,v=0;
float out_6020_angle[4],fangxiang,last_fangxiang,angle_6020,ecd_value=0;
int16_t out_3508_speed[4];

fp32 CHASSIS_6020_1_Y_ANGLE =4096;//看装配情况
fp32 CHASSIS_6020_2_Y_ANGLE =4096;
fp32 CHASSIS_6020_3_Y_ANGLE =4096;
fp32 CHASSIS_6020_4_Y_ANGLE =4096;

int8_t drct=1;
//bool flag=1;


//int x = 800;
extern SensorData receivedDataa[2];
PID_Typedef Chassis_Motor1_Pid,\
						Chassis_Motor2_Pid,\
						Chassis_Motor3_Pid,\
						Chassis_Motor4_Pid,\
		
						Chassis_Motor1_angle_Pid,\
						Chassis_Motor2_angle_Pid,\
						Chassis_Motor3_angle_Pid,\
						Chassis_Motor4_angle_Pid,\
						
						Chassis_Motor1_speed_Pid,\
						Chassis_Motor2_speed_Pid,\
						Chassis_Motor3_speed_Pid,\
						Chassis_Motor4_speed_Pid;\
						

PID_chassis_TypeDef target,output;

//▓▓▓▓▓▓▓▓▓▓▓▓▓▓主程序▓▓▓▓▓▓▓▓▓▓▓▓▓▓
void Control_Task(void)
{
	
	if(control_current_State==Manual)	 remote_control_();
	 if(sbus_struct.sbus_channels[3]<240 || sbus_flag==1||control_current_State==N_power  || sbus_struct.frame_lost!=0)
	{ 			//遥控未开启处理
		CAN_cmd_AGV_chassis(0,.0,0,0);
		CAN_cmd_chassis(0,0,0,0);  //ID1-4
	}                                    
	
}
//遥控器代码

void remote_control_()
{
	///底盘模式
		 if(control_current==M_chas)
	{
		usa_count=0;
	//遥控器只控制底盘
		if(sbus_struct.sbus_channels[3]!=1024&&sbus_flag!=1)//前后
	{y=-Steer(sbus_struct.sbus_channels[3]-1024,max/1.3);}	else y=0; // 1807为max速
		
	if(sbus_struct.sbus_channels[4]!=1024&&sbus_flag!=1)//左右
	{x=-Steer(sbus_struct.sbus_channels[4]-1024,max/1.3);} else x=0;// 1807为max速
	

	if(sbus_struct.sbus_channels[2]!=1024&&sbus_flag!=1)//速度
	{v=Steer(sbus_struct.sbus_channels[2]-1024,1);}	else v=0; // 1807为max速
		//spin_control(Initial_angle - All_Data.Ang_Z, &y, &x);//旋转矩阵运算,现角度 - 初始角度

	

//		Ded_Limit(x);        //遥控中值死区
//		Ded_Limit(y);
//		Ded_Limit(v);
//  
//	
		chassis_limit(&output,10000);//底盘电流限幅x
		
	
	    Target_speed(&target,x,y,v,0);   
	    Speed_loop();
		}
	CAN_cmd_AGV_chassis(output.ChassisWheel_Aangle_Speed_1,output.ChassisWheel_Aangle_Speed_2,output.ChassisWheel_Aangle_Speed_3,output.ChassisWheel_Aangle_Speed_4);
	CAN_cmd_chassis(output.ChassisWheel_Speed_1,output.ChassisWheel_Speed_2,output.ChassisWheel_Speed_3,output.ChassisWheel_Speed_4);  //ID1-4
 
	//pid_set(out_6020_angle[0],output.ChassisWheel_Aangle_1,4,4);

}


//▓▓▓▓▓▓▓▓▓▓▓▓▓▓初始化/功能函数▓▓▓▓▓▓▓▓▓▓▓▓▓▓
 void CHASSIS_Init(void)
{
	PID_Init(&Chassis_Motor1_Pid);
	PID_Init(&Chassis_Motor2_Pid);
	PID_Init(&Chassis_Motor3_Pid);
	PID_Init(&Chassis_Motor4_Pid);

}
//PID初始化
void CHASSIS_PID_Init(void)
{
	//设置初始目标值为0，安全
	int target_value = 0;

	target.ChassisWheel_Speed_1 = target_value;
	target.ChassisWheel_Speed_2 = target_value;
	target.ChassisWheel_Speed_3 = target_value;
	target.ChassisWheel_Speed_4 = target_value;

	//设置PID参数值
	PID_SET(&Chassis_Motor1_Pid, target.ChassisWheel_Speed_1,  	5.3, 0.1, 0.0);	//ID: =    //1-4号3508PID
	PID_SET(&Chassis_Motor2_Pid, target.ChassisWheel_Speed_2,   5.3, 0.1, 0.0);	//ID:2
	PID_SET(&Chassis_Motor3_Pid, target.ChassisWheel_Speed_3,   5.3, 0.1, 0.0);	//ID:3   
	PID_SET(&Chassis_Motor4_Pid, target.ChassisWheel_Speed_4,   5.3, 0.1, 0.0);	//ID:4
	
	PID_SET(&Chassis_Motor1_angle_Pid, target.ChassisWheel_Aangle_1,   12, 0,0);	//ID:4					//1-4号6020PID
	PID_SET(&Chassis_Motor2_angle_Pid, target.ChassisWheel_Aangle_2,   12, 0,0);	//ID:4
	PID_SET(&Chassis_Motor3_angle_Pid, target.ChassisWheel_Aangle_3,   12, 0,0);	//ID:4
	PID_SET(&Chassis_Motor4_angle_Pid, target.ChassisWheel_Aangle_4,   12, 0,0);	//ID:4
	//1,3阻力大
	PID_SET(&Chassis_Motor1_speed_Pid, target.ChassisWheel_Aangle_Speed_1,   110 ,0.5,30);	//ID:4					//1-4号6020PID
	PID_SET(&Chassis_Motor2_speed_Pid, target.ChassisWheel_Aangle_Speed_2,   112, 0.5,30);	//ID:4
	PID_SET(&Chassis_Motor3_speed_Pid, target.ChassisWheel_Aangle_Speed_3,   116, 0.5,28);	//ID:4
	PID_SET(&Chassis_Motor4_speed_Pid, target.ChassisWheel_Aangle_Speed_4,   115 , 0,28);	//ID:4
	
}
//PID赋值
void Speed_loop(void)
{

	output.ChassisWheel_Speed_1 = PID_incremental(&Chassis_Motor1_Pid, target.ChassisWheel_Speed_1, motor_chassis[0].speed_rpm, Motor_limiter_chassis);//3508电机1-4号
	output.ChassisWheel_Speed_2 = PID_incremental(&Chassis_Motor2_Pid, target.ChassisWheel_Speed_2, motor_chassis[1].speed_rpm, Motor_limiter_chassis);
	output.ChassisWheel_Speed_3 = PID_incremental(&Chassis_Motor3_Pid, target.ChassisWheel_Speed_3, motor_chassis[2].speed_rpm, Motor_limiter_chassis);							   
	output.ChassisWheel_Speed_4 = PID_incremental(&Chassis_Motor4_Pid, target.ChassisWheel_Speed_4, motor_chassis[3].speed_rpm, Motor_limiter_chassis);
// 1  3 
	output.ChassisWheel_Aangle_1 = PID_position(&Chassis_Motor1_angle_Pid, out_6020_angle[0], motor_chassis[4].total_angle,1000,20000);//6020电机1-4号
	output.ChassisWheel_Aangle_2 = PID_position(&Chassis_Motor2_angle_Pid, out_6020_angle[1], motor_chassis[5].total_angle,1000,20000);
	output.ChassisWheel_Aangle_3 = PID_position(&Chassis_Motor3_angle_Pid, out_6020_angle[2], motor_chassis[6].total_angle,1000,20000);
	output.ChassisWheel_Aangle_4 = PID_position(&Chassis_Motor4_angle_Pid, out_6020_angle[3], motor_chassis[7].total_angle,1000,20000);
	
	output.ChassisWheel_Aangle_Speed_1 = PID_position(&Chassis_Motor1_speed_Pid, output.ChassisWheel_Aangle_1, motor_chassis[4].speed_rpm,1000,20000);//速度环
	output.ChassisWheel_Aangle_Speed_2 = PID_position(&Chassis_Motor2_speed_Pid, output.ChassisWheel_Aangle_2, motor_chassis[5].speed_rpm,1000,20000);
	output.ChassisWheel_Aangle_Speed_3 = PID_position(&Chassis_Motor3_speed_Pid, output.ChassisWheel_Aangle_3, motor_chassis[6].speed_rpm,1000,20000);
  output.ChassisWheel_Aangle_Speed_4 = PID_position(&Chassis_Motor4_speed_Pid, output.ChassisWheel_Aangle_4, motor_chassis[7].speed_rpm,1000,20000);
}

  int16_t init6020_1=0 ;
	int16_t	init6020_2=180;
	int16_t init6020_3=0;
  int16_t init6020_4=180;
	int cut=0;int z_angle=0;
void Target_speed(PID_chassis_TypeDef *obj, double vx,  double vy, double vz,double angle)
{
	
if(angle==0)
{
  if(vx!=0||vy!=0)fangxiang=atan2(vx,-vy)* 180.0f / 3.1415;		 
	 else fangxiang=0;

	if((fangxiang-last_fangxiang)>350)
	{
		cut += 1;
	}
	else if ((fangxiang-last_fangxiang)<-350) 
	{
		cut -= 1;		
	}
 
	angle_6020=fangxiang;
	
	
	out_6020_angle[0]=init6020_1-angle_6020;
	out_6020_angle[1]=init6020_2-angle_6020;
	out_6020_angle[2]=init6020_3-angle_6020;
	out_6020_angle[3]=init6020_4-angle_6020;


	if(vx==0&&vy==0)
	{
		out_6020_angle[0]=init6020_1+45;
		out_6020_angle[1]=init6020_2-45;
		out_6020_angle[2]=init6020_3+45;
		out_6020_angle[3]=init6020_4-45;

	 obj->ChassisWheel_Speed_1  = (vz *3715) ;
	 obj->ChassisWheel_Speed_2  = (vz *3715);
	 obj->ChassisWheel_Speed_3  =-(vz *3715);
	 obj->ChassisWheel_Speed_4  =-(vz *3715);
	}
	else
	{
	 obj->ChassisWheel_Speed_1  =(vz *3715);
	 obj->ChassisWheel_Speed_2  =(vz *3715);
	 obj->ChassisWheel_Speed_3  =(vz *3715);
	 obj->ChassisWheel_Speed_4  =(vz *3715
		);
	}
	last_fangxiang=fangxiang;
}
else{
	  fangxiang=angle;
	
	 init6020_1=0; 
   init6020_2=180;
   init6020_3=0;
   init6020_4=180;
	
	out_6020_angle[0]=init6020_1-fangxiang;
	out_6020_angle[1]=init6020_2-fangxiang;
	out_6020_angle[2]=init6020_3-fangxiang;
	out_6020_angle[3]=init6020_4-fangxiang;

	 obj->ChassisWheel_Speed_1  =(vz *370.3582189574093);
	 obj->ChassisWheel_Speed_2  =(vz *370.3582189574093);
	 obj->ChassisWheel_Speed_3  =(vz *370.3582189574093);
	 obj->ChassisWheel_Speed_4  =(vz *370.3582189574093);
}



}
float Steer(int16_t speed,int16_t speed_limit)
{
	// 遥控器1807为max速
	return Scale((float)speed, 0, 783, (float)speed_limit);
}
int angle_to_ecd(float angle) {
    return (int)(angle * 22.75);
}

//优化
float Scale(float input, float inputMin, float inputMax, float  output_lim ){ 
  float output;
  if (inputMin < inputMax)
    output = ((input - inputMin) / (inputMax - inputMin))*output_lim ;
  else
    output = ((inputMin - input) / (inputMin - inputMax))*output_lim;
 
  return output;
}

void chassis_limit(PID_chassis_TypeDef *obj,float limit)
{
	obj->ChassisWheel_Speed_1 = (obj->ChassisWheel_Speed_1 >= limit)?limit:(obj->ChassisWheel_Speed_1 < -limit)?-limit:obj->ChassisWheel_Speed_1;
	obj->ChassisWheel_Speed_2 = (obj->ChassisWheel_Speed_2 >= limit)?limit:(obj->ChassisWheel_Speed_2 < -limit)?-limit:obj->ChassisWheel_Speed_2;
	obj->ChassisWheel_Speed_3 = (obj->ChassisWheel_Speed_3 >= limit)?limit:(obj->ChassisWheel_Speed_3 < -limit)?-limit:obj->ChassisWheel_Speed_3;
	obj->ChassisWheel_Speed_4 = (obj->ChassisWheel_Speed_4 >= limit)?limit:(obj->ChassisWheel_Speed_4 < -limit)?-limit:obj->ChassisWheel_Speed_4;
	obj->ChassisWheel_Speed_5 = (obj->ChassisWheel_Speed_5 >= limit)?limit:(obj->ChassisWheel_Speed_5 < -limit)?-limit:obj->ChassisWheel_Speed_5;
	obj->ChassisWheel_Speed_6 = (obj->ChassisWheel_Speed_6 >= limit)?limit:(obj->ChassisWheel_Speed_6 < -limit)?-limit:obj->ChassisWheel_Speed_6;
	obj->ChassisWheel_Speed_7 = (obj->ChassisWheel_Speed_7 >= limit)?limit:(obj->ChassisWheel_Speed_7 < -limit)?-limit:obj->ChassisWheel_Speed_7;
	obj->ChassisWheel_Speed_8 = (obj->ChassisWheel_Speed_8 >= limit)?limit:(obj->ChassisWheel_Speed_8 < -limit)?-limit:obj->ChassisWheel_Speed_8;
}
/**
 * @brief	dea区限制（防止误触）
 * @param	--obj:遥控回馈值
					--sensitivity：灵敏度，越大灵敏度越底
 * @ratval  none
 * @attention 遥控回馈值需先经过此函数再进行使用
*/
void Ded_Limit(int obj)
{
	  if(obj>15||obj<-15){obj=obj;}else{obj=0;}
}






