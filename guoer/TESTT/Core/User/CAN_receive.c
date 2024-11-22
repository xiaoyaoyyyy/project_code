/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is hcan1 interrupt function  to receive motor data,
  *             and hcan1 send function to send motor current to control motor.
  *             ������hcan1�жϽ��պ��������յ������,hcan1���ͺ������͵���������Ƶ��.
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


#include "main.h"
#include "CAN_receive.h"


extern CAN_HandleTypeDef  hcan1,hcan2;

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
		
	/**
 * @brief	��ȡ������ܽǶ�
 * @param	--obj: ����������ݽṹ��
 *				--rx:  hcan1 �������ݣ���Ϣ���ṹ��
 * @retval	none
*/
void get_MotorTotalAngle(motor_measure_t* obj)
{	
	
	int16_t ecd_err = obj->last_ecd - obj->ecd;	// ��һ��λ�� - ��һ��λ��= ������������֮��Ĳ�ֵ��

	/*��-6000��+6000��˵���������һȦ*/
	if (ecd_err > 6000) {
		obj->rount_cnt++;			//��Ȧ����һȦ
		obj->rate = ecd_err;	// + MAX_ENCODER;
	} else if (ecd_err < -6000) { 
		obj->rount_cnt--;			//��Ȧ����һȦ
		obj->rate = ecd_err;	// - MAX_ENCODER;
	} else {
		obj->rate = ecd_err;	//��Ȧ������
	}
			
	obj->total_angle = obj->rount_cnt * MAX_ENCODER + obj->ecd ;	//������ת��ֵ���ۼ�ֵ�������Ȧ����
	
	obj->total_angle = obj->total_angle / ENCODER_ANGLE;	//�ۻ�ֵת��Ϊ�Ƕ�
	
	obj->last_ecd = obj->ecd;		//������������
	
	
	
	/* ������ٶ� */
	int32_t sum = 0;
	uint8_t i;
	obj->filter[obj->filter_cnt++] = obj->rate;
	if (obj->filter_cnt >= FILTER_BUF)
		obj->filter_cnt = 0;
	for (i = 0; i < FILTER_BUF; i++)
		sum += obj->filter[i];
	
	obj->rate = -(sum / FILTER_BUF);
	
	obj->rate = obj->rate * ENCODER_COMPENSATION;	//�������Ĳ���
	
}
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
;
�������, 0:���̵��1 3508���,  1:���̵��2 3508���,2:���̵��3 3508���,3:���̵��4 3508���;
*/
motor_measure_t motor_supestratum[8];
motor_measure_t motor_chassis[8];


static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];

/**
  * @brief          hal hcan1 fifo call back, receive motor data
  * @param[in]      hcan, the point to hcan1 handle
  * @retval         none
  */
/**
  * @brief          hal��hcan1�ص�����,���յ������
  * @param[in]      hcan:hcan1���ָ��
  * @retval         none
  */
	int a,b = 0 ,mark=0;
	uint32_t interruptFlags1,interruptFlags2,interruptFlags3,interruptFlags4;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
	 if(hcan->Instance ==CAN1)
	{
		mark=1;
	  HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case CAN1_3508_M1_ID:
        case CAN1_3508_M2_ID:
        case CAN1_3508_M3_ID:
        case CAN1_3508_M4_ID:
        case CAN1_3508_M5_ID:
        case CAN1_3508_M6_ID:
        case CAN1_3508_M7_ID:
				case CAN1_3508_M8_ID:
        {
            static uint8_t i = 0;
            //get motor id
            i = rx_header.StdId - CAN_3508_M1_ID;
            get_motor_measure(&motor_chassis[i], rx_data);
						get_MotorTotalAngle(&motor_chassis[i]);
            break;
					
        }
			
    }
		//	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	}	

	 if(hcan->Instance ==CAN2)
{
	
	interruptFlags1 = hcan2.Instance->MSR;
	interruptFlags3	= hcan->Instance->IER;
		if (HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0) == 0)
		{
			a++;
		}
    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        case CAN_2006_M5_ID:
        case CAN_2006_M6_ID:
        case CAN_3508_M7_ID:
				case CAN_3508_M8_ID:
        {
            static uint8_t i = 0;
            //get motor id
            i = rx_header.StdId - CAN_3508_M1_ID;
            get_motor_measure(&motor_supestratum[i], rx_data);
						get_MotorTotalAngle(&motor_supestratum[i]);
            break;
					
        }
			
    }
  interruptFlags2= hcan2.Instance->MSR;
	interruptFlags4=hcan->Instance->IER;
			//	__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
	}	
	
	
	
}
/**
  * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      yaw : (0x205) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      pitch: (0x206) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      shoot: (0x207) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      rev: (0x208) ������������Ƶ���
  * @retval         none
  */
int abcdef,abcdef1;
 long h1,h2;
void CAN_cmd_gimbal1(int16_t motor1, int16_t motor2, int16_t ulift_l, int16_t ulift_r)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL1_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (motor1 >> 8);
    gimbal_can_send_data[1] = motor1;
    gimbal_can_send_data[2] = (motor2 >> 8);
    gimbal_can_send_data[3] = motor2;
    gimbal_can_send_data[4] = (ulift_l >> 8);
    gimbal_can_send_data[5] = ulift_l;
    gimbal_can_send_data[6] = (ulift_r >> 8);
    gimbal_can_send_data[7] = ulift_r;
   if( HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box)==HAL_OK)
		 {
			 abcdef++;
		 }
		// h1&=&hcan2->instance
		 
		 
}


void CAN_cmd_gimbal2(int16_t str_l, int16_t str_r, int16_t pitch, int16_t motor4)
{  
		
    uint32_t send_mail_box=CAN_TX_MAILBOX0 ;
    chassis_tx_message.StdId = CAN_GIMBAL2_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = str_l >> 8;
    chassis_can_send_data[1] = str_l;
    chassis_can_send_data[2] = str_r >> 8;
    chassis_can_send_data[3] = str_r;
    chassis_can_send_data[4] = pitch >> 8;
    chassis_can_send_data[5] = pitch;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;
		
	HAL_CAN_GetState(&hcan2);

	HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
if (HAL_CAN_GetError(&hcan2) != HAL_CAN_ERROR_NONE) {
    uint32_t error_code = HAL_CAN_GetError(&hcan2);

    // �������
    switch (error_code) {
        case HAL_CAN_ERROR_TIMEOUT:
            // ����ʱ����
				
            break;
         
        // ����������...
        default:
            // ����δ֪������
            break;
    }

 
}
	
	HAL_CAN_GetState(&hcan2);
	
/*
	//	HAL_Delay(5);
   if((HAL_CAN_AddTxMessage(&can1, &chassis_tx_message, chassis_can_send_data, &send_mail_box)!=HAL_OK))
	 {
		 aba=2;
	 }
	  while (HAL_CAN_GetTxMailboxesFreeLevel(&can1) != 3);
	 //else aba=1;
	if(HAL_CAN_GetTxMailboxesFreeLevel(&CHASSIS_CAN)==0)
	 {
		  aba=1;
	 }
	 else if(HAL_CAN_GetTxMailboxesFreeLevel(&CHASSIS_CAN)==1)	
	 {
		 aba=2;
	 }
	 else if(HAL_CAN_GetTxMailboxesFreeLevel(&CHASSIS_CAN)==2)
	 {
		 aba=3;
	 } else if(HAL_CAN_GetTxMailboxesFreeLevel(&CHASSIS_CAN)==3)
	 {
		 aba=4;
	 }
	// printf(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));
	  //while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3);
	 for(int i=0;i<8;i++)
	 {
	     printf("%d\n", chassis_can_send_data[i]);
	 }
				*/		 
}
void CAN_cmd_AGV_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId =	CAN1_CHASSIS2_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{  // aba=1;
		
    uint32_t send_mail_box=CAN_TX_MAILBOX0 ;
    chassis_tx_message.StdId = CAN1_CHASSIS1_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;
	
		
	HAL_CAN_GetState(&hcan1);

	if(HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box)==HAL_OK)
	{
		abcdef1++;
	}
  
}


/**
  * @brief          send hcan1 packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����IDΪ0x700��hcan1��,��������3508��������������ID
  * @param[in]      none
  * @retval         none
  */
//void CAN_cmd_chassis_reset_ID(void)
//{
//   uint32_t send_mail_box ;
//    chassis_tx_message.StdId = 0x700;
//    chassis_tx_message.IDE = CAN_ID_STD;
//    chassis_tx_message.RTR = CAN_RTR_DATA;
//    chassis_tx_message.DLC = 0x08;
//    chassis_can_send_data[0] = 0;
//    chassis_can_send_data[1] = 0;
//    chassis_can_send_data[2] = 0;
//    chassis_can_send_data[3] = 0;
//    chassis_can_send_data[4] = 0;
//    chassis_can_send_data[5] = 0;
//    chassis_can_send_data[6] = 0;
//    chassis_can_send_data[7] = 0;

//    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
//}



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
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_supestratum[4];
}

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
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_supestratum[5];
}


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
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_supestratum[6];
}


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
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_supestratum[(i & 0x03)];
}
