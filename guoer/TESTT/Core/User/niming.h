#ifndef __NIMING_H
#define __NIMING_H
#include "stm32f4xx_hal.h"


void usart_send_char(UART_HandleTypeDef *huart, uint8_t c);
void usart_niming_report(uint8_t fun,uint8_t*data,uint8_t len);
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz);
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw);
void send_sensorData(int8_t aacx,int16_t aacy,int16_t aacz,int16_t gyrox,int16_t gyroy,int16_t gyroz);

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))	 //取出int型变量的低字节
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))	 //	取存储在此变量下一内存字节的内容，高字节
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

#endif

