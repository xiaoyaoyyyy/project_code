#ifndef __HWT101_H
#define __HWT101_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "usart.h"
#include "string.h"

extern uint8_t  kRxBuffer[22];
typedef struct
{
	short data[3];
	short T;
}Data;	//����ǰ����

typedef struct
{
	//���ٶ�
	float Pal_X;	//X��
	float Pal_Y;	//Y��
	float Pal_Z;	//Z��
	float Pal_T;	//�¶�
	//�Ƕ�
	float Ang_X;	//X��
	float Ang_Y;	//Y��
	float Ang_Z;	//Z��
	float Ang_T;	//�¶�
	
}all_data;	//����������

extern Data Palstance;   //���ٶ����ݽṹ��
extern Data	Angle;		    //�Ƕ����ݽṹ��
extern all_data	All_Data;

extern uint8_t CLEAR[5];		//����ָ��
extern uint8_t SLEEP[5];		//�����������
extern uint8_t RATE_10[5];	//���ûش�����10Hz(��������Ժ��ٸ�ģ�������ϵ����Ч)
extern uint8_t RATE_20[5];	//���ûش�����20Hz
extern uint8_t RATE_50[5];	//���ûش�����50Hz
extern uint8_t RATE_100[5];	//���ûش�����100Hz
extern uint8_t GYRO_ON[5];				//�����������Զ�У׼
extern uint8_t GYRO_OFF[5];				//�ر��������Զ�У׼
extern uint8_t CALSW_OFF[5];			//�˳�У׼ģʽ
extern uint8_t CALSW_speed[5];		//������ٶ�У׼ģʽ
extern uint8_t BAUD_9600[5];					//���ô��ڲ�����(9600)
extern uint8_t BAUD_115200[5];				//���ô��ڲ�����(115200)
extern uint8_t SAVE_current[5];     	//���ֵ�ǰ����
extern uint8_t SAVE_recover[5];				//�ָ�Ĭ�ϣ����������ò�����
extern uint8_t DIRECTION_vertical[5];	//����Ϊˮƽ��װ
extern uint8_t DIRECTION_standard[5];	//����Ϊ��ֱ��װ
extern uint8_t CALIYAW[5];	//Z��Ƕ�����

void SYSTEM_Init_only(UART_HandleTypeDef *huart, uint8_t* content);
void SYSTEM_Init_most(UART_HandleTypeDef *huart);
void HWT605_Data(uint16_t mode);
void HWT605_printf(void);
void Data_reception(unsigned char ucData);
void Data_reception_DMA(unsigned char ucData);
void Data_reception_DMA_only(unsigned char ucData);
void USER_GYRO_IDLE_IRQHandler(UART_HandleTypeDef *huart);
#ifdef __cplusplus
}
#endif

#endif /*__HWT605*/
