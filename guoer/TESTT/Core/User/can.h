#ifndef _CAN_H
#define _CAN_H

#include "stm32f4xx_hal.h"
void MX_CAN1_Init(void);
void MX_CAN2_Init1(void);
extern CAN_HandleTypeDef hcan1,hcan2;

#endif
