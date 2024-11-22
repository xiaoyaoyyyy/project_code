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
}Data;	//解析前数据

typedef struct
{
	//角速度
	float Pal_X;	//X轴
	float Pal_Y;	//Y轴
	float Pal_Z;	//Z轴
	float Pal_T;	//温度
	//角度
	float Ang_X;	//X轴
	float Ang_Y;	//Y轴
	float Ang_Z;	//Z轴
	float Ang_T;	//温度
	
}all_data;	//解析后数据

extern Data Palstance;   //角速度数据结构体
extern Data	Angle;		    //角度数据结构体
extern all_data	All_Data;

extern uint8_t CLEAR[5];		//解锁指令
extern uint8_t SLEEP[5];		//休眠与解休眠
extern uint8_t RATE_10[5];	//设置回传速率10Hz(设置完成以后再给模块重新上电后生效)
extern uint8_t RATE_20[5];	//设置回传速率20Hz
extern uint8_t RATE_50[5];	//设置回传速率50Hz
extern uint8_t RATE_100[5];	//设置回传速率100Hz
extern uint8_t GYRO_ON[5];				//开启陀螺仪自动校准
extern uint8_t GYRO_OFF[5];				//关闭陀螺仪自动校准
extern uint8_t CALSW_OFF[5];			//退出校准模式
extern uint8_t CALSW_speed[5];		//进入加速度校准模式
extern uint8_t BAUD_9600[5];					//设置串口波特率(9600)
extern uint8_t BAUD_115200[5];				//设置串口波特率(115200)
extern uint8_t SAVE_current[5];     	//保持当前配置
extern uint8_t SAVE_recover[5];				//恢复默认（出厂）配置并保存
extern uint8_t DIRECTION_vertical[5];	//设置为水平安装
extern uint8_t DIRECTION_standard[5];	//设置为垂直安装
extern uint8_t CALIYAW[5];	//Z轴角度置零

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
