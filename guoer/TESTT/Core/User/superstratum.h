#ifndef _SUPERSTRATUM_H
#define _SUPERSTRATUM_H
#include "stm32f4xx_hal.h"
#include "pid.h"
#include "main.h"
#include "CAN_receive.h"
#include "chassis.h"
#include "sbus.h"
#include "niming.h"
#include "mode_choic.h"
#include <stdbool.h>
typedef struct{

	int16_t _Pitch_Angle;
	int16_t _Pitch;

	int16_t _Uplift_L;
	int16_t _Uplift_Angle_L;
	
	int16_t _Uplift_R;
	int16_t _Uplift_Angle_R;
	
	int16_t _Stretch;
	int16_t _Stretch_Angle;
	
}PID_ring_take_TypeDef;

typedef struct{
//Lift,Stretch,pit
float previous[3];
float current[3];
float	tar_angle[5];
	
}ANGLE_TypeDef;

extern int get_acoder_flag;
extern int  Lif_L_Init ;
extern int   Lif_R_Init   ;
extern int  Str_L_Init   ;
extern int  Str_R_Init   ;
extern int   Pit_Init   ;
extern int16_t M_flag,M_flag2;

void Hair_ring_Init(void);
void	ALL_PID_SET(void);
void Uplift_L(int up_angle);
void Uplift_R(int up_angle);
void Stretch(int S_angle);
void Pitch(int P_angle);
void control_sup_task(void);
void remote_control(void);
void auto_control(void);
void GET_InitPinstate(void);
extern void diancifa(bool a);
extern void motion(int Target_angle,PID_Typedef*Angle_pid,PID_Typedef*Speed_pid,int motor_);




float calculateJoystickIncrement(float currentInput, float previousInput);
float angle_limit(float angle_,float angle_min,float angle_max);
#define	Motor_limiter_ring_take	12000	//	电机3508最大值
#define Motor_limiter_ring_2006 12000 //电机2006最大值

#endif
