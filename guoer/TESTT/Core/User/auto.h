#ifndef _AUTO_H
#define _AUTO_H

#include "main.h"
#include "pid.h"
#include "CAN_receive.h"
#include "chassis.h"
#include "sbus.h"
#include "niming.h"
#include "mode_choic.h"
#include "superstratum.h"
#include "usart.h"

extern int usa_count;
extern uint8_t super_control_State;
extern int16_t A_flag,A_flag2;
extern int Lif_L_Init,Lif_R_Init,Str_L_Init,Str_R_Init,Pit_Init1; 
extern bool Sinit_flag,Sinit_flag1,Pinit_flag,Pinit_flag1;
extern int init_flag;
extern int mark1,mark2,mark3,mark4;
extern  uint8_t message[2][8];
typedef enum{
	
	Init=0,
	None_,
	Getting_motion1,
	Getting_motion2,
	Getting_motion2_B,
	Getting_BACK,
	Got,
	Put,
	throw_ball,
 
}super_control_auto_stack;//×´Ì¬»ú


typedef enum{
	 
 INIT=0,
 UP,
 DOWN_PROTRUDE,
 BACK, 

}super_control_state ;

#define DEBUG_GIMBAL
//#define MONTION_TIME

#define M_PI 3.141592
#define up_ball_init    0
#define up_ball_getting  16000
#define up_ball_got    	 1000 
#define up_ball_put      18000

#define stre_ball_go      21400
#define stre_can_pit      -2000
#define stre_ball_init     -0
#define Stre_ball_init     -200
#define stre_ball_put     000
#define Stre_ball_put     -400

#define pit_ball_init   40000
#define pit_ball_get    91000  
#define pit_ball_put    00    
#define pit_ball_V_min    45000    
#define pit_ball_V_max    60000    
#define Pit_ball_put    000    


#define LIF_L_OFFSET  250
#define LIF_R_OFFSET  150
#define STR_OFFSET  150
#define PIT_OFFSET  2500

#define PUT_DESTINATION 2
#define GET_DESTINATION 1
#define BASKET_DESTINATION 0 
#define GET_TASK 3
#define PUT_TASK 2 
#define NONE_TASK 0
void value_init(void);
void put_basket_choic(void);
void Auto_control_task(void);
void auto_control_state(void);
void control_sup(int up_l_L,int up_R_L,int stre_L ,int stre_R,int pit);
void fifio_Init(void);
void Auto_init(void);
void Key_judgement(int a);
void Auto_chassic_mode(void);
extern bool ball_g;
extern void GET_InitPinstate(void);
extern bool Pinit_flag2,Sinit_flag2;
int map_value(int value, int in_min, int in_max, int out_min, int out_max) ;
void usart_send_message(UART_HandleTypeDef *huart,int a,int b);		
float calculateVelocity(float flag,float realtime_distance,float Acceleration, float maxVelocity) ;
double calculate_auto_tar_angle(double t_angle, double distance,double liqiujuli);
#endif
