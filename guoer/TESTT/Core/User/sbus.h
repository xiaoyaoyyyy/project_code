/***********************遥控按键分配**********************
//手动模式//
		SWA:安全控制解锁

		SWB：
		手动：夹爪状态（上：放，中：夹）
		SWC：
		手动：抓取状态选择(上中下),注意此时为半自动状态(RR:上下对应装填位和抓取位；ER：上中下对应装填位、抬升位和抓取位)
		
		SWD：摩擦轮开启

		按键右：短按发射
		
		滚轮右:俯仰角调整

		拨杆左：底盘移动
		拨杆右：底盘方向
		
		
//自动模式//
		SWA:安全控制解锁
	自动模式位置选择表：
	***********************
	*		SWB		SWC		位置	*
	*		上		上		左		*
	*		上		中		左中	*
	*		中		中		中		*
	*		下		中		右中	*
	*		下		下		右		*
	***********************
		SWD：手动自动选择
		按键右：短按发射
		滚轮右:柱子选择(可选所有柱子)
		拨杆左：底盘移动
		拨杆右：底盘方向
*****************************************************/

#ifndef SBUS_H
#define SBUS_H
#include "stm32f4xx_hal.h"
#include <string.h> 
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
//sbus相关变量
#define SBUS_LEN      25		//数据长度
#define SBUS_CHANNELS 16		//通道（此遥控只有10通道，SBUS协议支持16通道）


#define Remote_switch_Up		240		//遥控开关通道处于上状态
#define Remote_switch_Middle	1024	//遥控开关通道处于中状态
#define Remote_switch_Down		1807	//遥控开关通道处于下状态

//****执行状态****//

//安全锁定//
#define lock							sbus_struct.sbus_channels[5]  == Remote_switch_Up //SWA
#define unlock						sbus_struct.sbus_channels[5]  == Remote_switch_Down //SWA
//自动状态//
#define left position								(sbus_struct.sbus_channels[6]	== Remote_switch_Up && sbus_struct.sbus_channels[7]==Remote_switch_Up)//SWB&SWC
#define left_and_middle position		(sbus_struct.sbus_channels[6]	== Remote_switch_Up && sbus_struct.sbus_channels[7]==Remote_switch_Middle)//SWB&SWC
#define middle position							(sbus_struct.sbus_channels[6]	== Remote_switch_Middle && sbus_struct.sbus_channels[7]==Remote_switch_Middle)//SWB&SWC	
#define right_and_middle position		(sbus_struct.sbus_channels[6]	== Remote_switch_Down && sbus_struct.sbus_channels[7]==Remote_switch_Middle)//SWB&SWC
#define right position							(sbus_struct.sbus_channels[6]	== Remote_switch_Down && sbus_struct.sbus_channels[7]==Remote_switch_Down)//SWB&SWC	


//视觉以及激光数据处理组合按键//
#define jiguangchengxu              (sbus_struct.sbus_channels[5]  == Remote_switch_Down &&  sbus_struct.sbus_channels[8]	== Remote_switch_Up)//SWA下%SWD上
#define shijuechuli                	(sbus_struct.sbus_channels[5]  == Remote_switch_Down &&  sbus_struct.sbus_channels[8]	== Remote_switch_Down)//SWA下%SWD下


#define Auto_Mode										sbus_struct.sbus_channels[8]	== Remote_switch_Down //SWD

#define select_target_up 						sbus_struct.sbus_channels[10] == Remote_switch_Up//右旋钮上
#define select_target_down 					sbus_struct.sbus_channels[10] == Remote_switch_Down//右旋钮往上
//手动状态//
#define Take_ring										sbus_struct.sbus_channels[6]	== Remote_switch_Up//SWB
#define Take_ring_up								sbus_struct.sbus_channels[6]	== Remote_switch_Middle//SWB
#define Put_ring										sbus_struct.sbus_channels[6]	== Remote_switch_Down//SWB

#define put_up_position							sbus_struct.sbus_channels[7]	== Remote_switch_Up//SWC
#define put_middle_position					sbus_struct.sbus_channels[7]	== Remote_switch_Middle//SWC
#define put_down_position						sbus_struct.sbus_channels[7]	== Remote_switch_Down//SWC

#define Hand_Mode										sbus_struct.sbus_channels[8]	== Remote_switch_Up //SWD
#define fire_shoot 									sbus_struct.sbus_channels[9] 	== Remote_switch_Down//右按键按下
#define select_pitch_angle 					sbus_struct.sbus_channels[10]//右旋钮

//sbus协议结构体
typedef struct sbus{
	
    int16_t sbus_channels[SBUS_CHANNELS+1];	//通道值
    uint8_t frame_lost;						//遥控丢失信号标志位，未丢失为0
    uint8_t failsafe_activated;				
    uint8_t digital[2];
	
}sbus_t;
extern int  rx_R1,rx_R2,rx_V1,rx_V2;
extern uint8_t sbus_flag;
extern sbus_t sbus_struct;		
extern uint16_t landmark;
extern uint8_t remote_buf[25];//接收数据缓存
void USER_UART1_IDLE_IRQHandler(UART_HandleTypeDef *huart);//串口中断回调函数
void Memset(uint8_t* Remote_buff,int x,uint16_t Len);//清除缓冲区
extern bool rx_R2_flag;
void sbus_decoder(uint8_t* frame);

#endif //!SBUS_H

