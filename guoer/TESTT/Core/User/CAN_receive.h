/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "stm32f4xx_hal.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2
extern int aba ;
//extern void a();
/* CAN send and receive ID */
typedef enum
{
	
	
	CAN1_CHASSIS1_ALL_ID = 0x200,
    CAN1_3508_M1_ID = 0x201,
    CAN1_3508_M2_ID = 0x202,
    CAN1_3508_M3_ID = 0x203,
    CAN1_3508_M4_ID = 0x204,
	
   CAN1_CHASSIS2_ALL_ID = 0x1FF,
     CAN1_3508_M5_ID = 0x205,
     CAN1_3508_M6_ID = 0x206,
     CAN1_3508_M7_ID = 0x207,
     CAN1_3508_M8_ID = 0x208,
	
	
	
    CAN_GIMBAL1_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
	
   CAN_GIMBAL2_ALL_ID = 0x1FF,
     CAN_2006_M5_ID = 0x205,
     CAN_2006_M6_ID = 0x206,
     CAN_3508_M7_ID = 0x207,
     CAN_3508_M8_ID = 0x208,

} can_msg_id_e;


#define FILTER_BUF	5									//过滤器缓冲区
#define ENCODER_COMPENSATION	7.324f	//编码器补偿

#define MAX_ENCODER		8192.0f					//编码器最大值
#define ENCODER_ANGLE	(MAX_ENCODER / 360.0f)	//编码器角度
#define MAX_MOTOR_CURRENT	16384				// 3508 电机最大电流
#define MAX_MOTOR_CURRENT_2006 10000	// 2006 电机最大电流
#define MAX_ANGLE 19*360.0f

	typedef struct
	{
			uint16_t ecd;            //电机转子角度
			int16_t speed_rpm;       //电机转子转速，
			int16_t given_current;   //控制电流
			uint8_t temperate;       //温度
			int16_t last_ecd;
		
		  uint16_t real_angle;
		
		int32_t rount_cnt;			//The total number turns   总转数（电机圈数）
		int32_t total_angle;		//The total anlge of the motor	电机的总角度
		int16_t rate;						//加速度
		int16_t last_rate;			//上一个速度
		int16_t filter[FILTER_BUF];		//encoder filter buffer	 编码器过滤器缓冲
		uint8_t filter_cnt;			//filter 过滤器圈数
		
	} motor_measure_t;

typedef struct
{
			uint16_t ecd;            //电机转子角度
			int16_t speed_rpm;       //电机转子转速，
			int16_t given_current;   //控制电流
			uint8_t temperate;       //温度
			int16_t last_ecd;
		
		  uint16_t real_angle;
		
		int32_t rount_cnt;			//The total number turns   总转数（电机圈数）
		int32_t total_angle;		//The total anlge of the motor	电机的总角度
		int16_t rate;						//加速度
		int16_t last_rate;			//上一个速度
		int16_t filter[FILTER_BUF];		//encoder filter buffer	 编码器过滤器缓冲
		uint8_t filter_cnt;			//filter 过滤器圈数
		
	} motor_mesure_t1;
	
extern motor_measure_t motor_supestratum[8];
extern motor_measure_t motor_chassis[8];

/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
extern void CAN_cmd_gimbal1(int16_t motor1, int16_t motor2, int16_t ulift_l, int16_t ulift_r);
extern void CAN_cmd_gimbal2(int16_t str_l, int16_t str_r, int16_t pitch, int16_t motor4);
extern void CAN_cmd_AGV_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);


#endif
