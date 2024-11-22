/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
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


#define FILTER_BUF	5									//������������
#define ENCODER_COMPENSATION	7.324f	//����������

#define MAX_ENCODER		8192.0f					//���������ֵ
#define ENCODER_ANGLE	(MAX_ENCODER / 360.0f)	//�������Ƕ�
#define MAX_MOTOR_CURRENT	16384				// 3508 ���������
#define MAX_MOTOR_CURRENT_2006 10000	// 2006 ���������
#define MAX_ANGLE 19*360.0f

	typedef struct
	{
			uint16_t ecd;            //���ת�ӽǶ�
			int16_t speed_rpm;       //���ת��ת�٣�
			int16_t given_current;   //���Ƶ���
			uint8_t temperate;       //�¶�
			int16_t last_ecd;
		
		  uint16_t real_angle;
		
		int32_t rount_cnt;			//The total number turns   ��ת�������Ȧ����
		int32_t total_angle;		//The total anlge of the motor	������ܽǶ�
		int16_t rate;						//���ٶ�
		int16_t last_rate;			//��һ���ٶ�
		int16_t filter[FILTER_BUF];		//encoder filter buffer	 ����������������
		uint8_t filter_cnt;			//filter ������Ȧ��
		
	} motor_measure_t;

typedef struct
{
			uint16_t ecd;            //���ת�ӽǶ�
			int16_t speed_rpm;       //���ת��ת�٣�
			int16_t given_current;   //���Ƶ���
			uint8_t temperate;       //�¶�
			int16_t last_ecd;
		
		  uint16_t real_angle;
		
		int32_t rount_cnt;			//The total number turns   ��ת�������Ȧ����
		int32_t total_angle;		//The total anlge of the motor	������ܽǶ�
		int16_t rate;						//���ٶ�
		int16_t last_rate;			//��һ���ٶ�
		int16_t filter[FILTER_BUF];		//encoder filter buffer	 ����������������
		uint8_t filter_cnt;			//filter ������Ȧ��
		
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
  * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      pitch: (0x206) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      shoot: (0x207) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      rev: (0x208) ������������Ƶ���
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
  * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
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
  * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ����yaw 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ����pitch 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ���ز������ 2006�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          ���ص��̵�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);


#endif
