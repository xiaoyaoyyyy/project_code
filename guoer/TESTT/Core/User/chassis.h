#ifndef _CHASSIS_H
#define _CHASSIS_H
#include "stm32f4xx_hal.h"


 typedef struct{
	double ChassisWheel_Speed_1;
	double ChassisWheel_Speed_2;
	double ChassisWheel_Speed_3;
	double ChassisWheel_Speed_4;
	double ChassisWheel_Speed_5;
	double ChassisWheel_Speed_6;
	double ChassisWheel_Speed_7;
	double ChassisWheel_Speed_8;
	 
	double ChassisWheel_Aangle_1;
	double ChassisWheel_Aangle_2;
	double ChassisWheel_Aangle_3;
	double ChassisWheel_Aangle_4;
	double ChassisWheel_Aangle_5;
	double ChassisWheel_Aangle_6;
	double ChassisWheel_Aangle_7;
	double ChassisWheel_Aangle_8;
	
	double ChassisWheel_Aangle_Speed_1;
	double ChassisWheel_Aangle_Speed_2;	 
	double ChassisWheel_Aangle_Speed_3;
	double ChassisWheel_Aangle_Speed_4;
	 
}PID_chassis_TypeDef;//底盘 车轮速度
 

typedef struct
{
    float vx;
    float vy;
    float vw;
} chassis_6020_speed;

typedef float fp32;
typedef double fp64;

typedef uint8_t 	u8;
typedef uint16_t 	u16;
typedef uint32_t 	u32;

#define	WHEELBASE   1
#define	WHEELTRACK   200
#define PI 3.14159265358979323846
#define	PERIMETER    0.98*PI
#define	CHASSIS_DECELE_RATIO   (1.0f/19.0f)  //电机减数比
#define WHEEL_RPM_RATION  (60.0f/(PERIMETER*CHASSIS_DECELE_RATIO))
#define RADIAN_COEF  57.3f

#define	Motor_limiter_chassis	8000		//底盘电机限幅最大电流
#define max                    20	//速度最大电流 

extern double  x,y,v;
extern int get_acoder_flag_;
extern float out_6020_angle[4],fangxiang,last_fangxiang,angle_6020,ecd_value;

void Target_speed(PID_chassis_TypeDef *obj, double vx,  double vy, double vz,double angle);
void CHASSIS_Init(void);
void CHASSIS_PID_Init(void);
void Speed_loop(void);
void control(void);
void Control_Task(void);
void chassis_limit(PID_chassis_TypeDef *obj,float limit);
void Ded_Limit(int obj);
float Scale(float input, float inputMin, float inputMax, float  output_lim );
float Steer(int16_t speed,int16_t speed_limit);
float Scale1(float input, float inputMin, float inputMax, float  output_lim );
void remote_control_(void);
void auto_control_(void);
int angle_to_ecd(float angle);
void angle_6020_cacl(chassis_6020_speed *speed,fp32* angle);
void AGV_3508_calc(chassis_6020_speed *speed,int16_t* out_speed);




#endif

