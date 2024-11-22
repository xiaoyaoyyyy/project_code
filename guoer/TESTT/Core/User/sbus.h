/***********************ң�ذ�������**********************
//�ֶ�ģʽ//
		SWA:��ȫ���ƽ���

		SWB��
		�ֶ�����צ״̬���ϣ��ţ��У��У�
		SWC��
		�ֶ���ץȡ״̬ѡ��(������),ע���ʱΪ���Զ�״̬(RR:���¶�Ӧװ��λ��ץȡλ��ER�������¶�Ӧװ��λ��̧��λ��ץȡλ)
		
		SWD��Ħ���ֿ���

		�����ң��̰�����
		
		������:�����ǵ���

		�����󣺵����ƶ�
		�����ң����̷���
		
		
//�Զ�ģʽ//
		SWA:��ȫ���ƽ���
	�Զ�ģʽλ��ѡ���
	***********************
	*		SWB		SWC		λ��	*
	*		��		��		��		*
	*		��		��		����	*
	*		��		��		��		*
	*		��		��		����	*
	*		��		��		��		*
	***********************
		SWD���ֶ��Զ�ѡ��
		�����ң��̰�����
		������:����ѡ��(��ѡ��������)
		�����󣺵����ƶ�
		�����ң����̷���
*****************************************************/

#ifndef SBUS_H
#define SBUS_H
#include "stm32f4xx_hal.h"
#include <string.h> 
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
//sbus��ر���
#define SBUS_LEN      25		//���ݳ���
#define SBUS_CHANNELS 16		//ͨ������ң��ֻ��10ͨ����SBUSЭ��֧��16ͨ����


#define Remote_switch_Up		240		//ң�ؿ���ͨ��������״̬
#define Remote_switch_Middle	1024	//ң�ؿ���ͨ��������״̬
#define Remote_switch_Down		1807	//ң�ؿ���ͨ��������״̬

//****ִ��״̬****//

//��ȫ����//
#define lock							sbus_struct.sbus_channels[5]  == Remote_switch_Up //SWA
#define unlock						sbus_struct.sbus_channels[5]  == Remote_switch_Down //SWA
//�Զ�״̬//
#define left position								(sbus_struct.sbus_channels[6]	== Remote_switch_Up && sbus_struct.sbus_channels[7]==Remote_switch_Up)//SWB&SWC
#define left_and_middle position		(sbus_struct.sbus_channels[6]	== Remote_switch_Up && sbus_struct.sbus_channels[7]==Remote_switch_Middle)//SWB&SWC
#define middle position							(sbus_struct.sbus_channels[6]	== Remote_switch_Middle && sbus_struct.sbus_channels[7]==Remote_switch_Middle)//SWB&SWC	
#define right_and_middle position		(sbus_struct.sbus_channels[6]	== Remote_switch_Down && sbus_struct.sbus_channels[7]==Remote_switch_Middle)//SWB&SWC
#define right position							(sbus_struct.sbus_channels[6]	== Remote_switch_Down && sbus_struct.sbus_channels[7]==Remote_switch_Down)//SWB&SWC	


//�Ӿ��Լ��������ݴ�����ϰ���//
#define jiguangchengxu              (sbus_struct.sbus_channels[5]  == Remote_switch_Down &&  sbus_struct.sbus_channels[8]	== Remote_switch_Up)//SWA��%SWD��
#define shijuechuli                	(sbus_struct.sbus_channels[5]  == Remote_switch_Down &&  sbus_struct.sbus_channels[8]	== Remote_switch_Down)//SWA��%SWD��


#define Auto_Mode										sbus_struct.sbus_channels[8]	== Remote_switch_Down //SWD

#define select_target_up 						sbus_struct.sbus_channels[10] == Remote_switch_Up//����ť��
#define select_target_down 					sbus_struct.sbus_channels[10] == Remote_switch_Down//����ť����
//�ֶ�״̬//
#define Take_ring										sbus_struct.sbus_channels[6]	== Remote_switch_Up//SWB
#define Take_ring_up								sbus_struct.sbus_channels[6]	== Remote_switch_Middle//SWB
#define Put_ring										sbus_struct.sbus_channels[6]	== Remote_switch_Down//SWB

#define put_up_position							sbus_struct.sbus_channels[7]	== Remote_switch_Up//SWC
#define put_middle_position					sbus_struct.sbus_channels[7]	== Remote_switch_Middle//SWC
#define put_down_position						sbus_struct.sbus_channels[7]	== Remote_switch_Down//SWC

#define Hand_Mode										sbus_struct.sbus_channels[8]	== Remote_switch_Up //SWD
#define fire_shoot 									sbus_struct.sbus_channels[9] 	== Remote_switch_Down//�Ұ�������
#define select_pitch_angle 					sbus_struct.sbus_channels[10]//����ť

//sbusЭ��ṹ��
typedef struct sbus{
	
    int16_t sbus_channels[SBUS_CHANNELS+1];	//ͨ��ֵ
    uint8_t frame_lost;						//ң�ض�ʧ�źű�־λ��δ��ʧΪ0
    uint8_t failsafe_activated;				
    uint8_t digital[2];
	
}sbus_t;
extern int  rx_R1,rx_R2,rx_V1,rx_V2;
extern uint8_t sbus_flag;
extern sbus_t sbus_struct;		
extern uint16_t landmark;
extern uint8_t remote_buf[25];//�������ݻ���
void USER_UART1_IDLE_IRQHandler(UART_HandleTypeDef *huart);//�����жϻص�����
void Memset(uint8_t* Remote_buff,int x,uint16_t Len);//���������
extern bool rx_R2_flag;
void sbus_decoder(uint8_t* frame);

#endif //!SBUS_H

