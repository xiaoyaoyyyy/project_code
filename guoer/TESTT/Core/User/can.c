
#include "main.h"
#include "can.h"
CAN_HandleTypeDef hcan1,hcan2;
int tset1,tset2=0;
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

	//检测中断是否开启
if (__HAL_CAN_GET_IT_SOURCE(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != RESET)
{
    // 中断已启用
    // 这里可以添加其他需要执行的代码
	tset1=1;
}

CAN_FilterTypeDef  CAN_FilterInitStructure;

	/*CAN1筛选器初始化*/
	CAN_FilterInitStructure.FilterBank=0;	//筛选器组0
	//工作在掩码模式
	CAN_FilterInitStructure.FilterMode=CAN_FILTERMODE_IDMASK;	
	//筛选器位宽为单个32位。
	CAN_FilterInitStructure.FilterScale=CAN_FILTERSCALE_32BIT;	
	/* 使能筛选器，按照标志的内容进行比对筛选，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */
	//要筛选的ID高位
	CAN_FilterInitStructure.FilterIdHigh= 0x0000;	
	//要筛选的ID低位 
	CAN_FilterInitStructure.FilterIdLow= 0x0000; 
	//筛选器高16位每位必须匹配
	CAN_FilterInitStructure.FilterMaskIdHigh= 0x0000;			
	//筛选器低16位每位必须匹配
	CAN_FilterInitStructure.FilterMaskIdLow= 0x0000;			
	//筛选器被关联到FIFO0
	CAN_FilterInitStructure.FilterFIFOAssignment=CAN_FILTER_FIFO0 ;	
	//使能筛选器
	CAN_FilterInitStructure.FilterActivation=ENABLE;
//	CAN_FilterInitStructure.SlaveStartFilterBank = 14;//从筛选器组	
	HAL_CAN_ConfigFilter(&hcan1,&CAN_FilterInitStructure);

HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); 
HAL_CAN_Start(&hcan1);
	//检测中断是否开启
if (__HAL_CAN_GET_IT_SOURCE(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != RESET)
{
    // 中断已启用
    // 这里可以添加其他需要执行的代码
	tset2=1;
}

}
int aaaaaa=0;
void MX_CAN2_Init1(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
   hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 9;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    aaaaaa=3;  }
  /* USER CODE BEGIN CAN1_Init 2 */

	//检测中断是否开启
if (__HAL_CAN_GET_IT_SOURCE(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != RESET)
{
    // 中断已启用
    // 这里可以添加其他需要执行的代码
	tset1=1;
}

CAN_FilterTypeDef  CAN_FilterInitStructure1;

	/*CAN1筛选器初始化*/
	CAN_FilterInitStructure1.FilterBank=14;	//筛选器组14
	//工作在掩码模式
	CAN_FilterInitStructure1.FilterMode=CAN_FILTERMODE_IDMASK;	
	//筛选器位宽为单个32位。
	CAN_FilterInitStructure1.FilterScale=CAN_FILTERSCALE_32BIT;	
	/* 使能筛选器，按照标志的内容进行比对筛选，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */
	//要筛选的ID高位
	CAN_FilterInitStructure1.FilterIdHigh= 0x0000;	
	//要筛选的ID低位 
	CAN_FilterInitStructure1.FilterIdLow= 0x0000; 
	//筛选器高16位每位必须匹配
	CAN_FilterInitStructure1.FilterMaskIdHigh= 0x0000;			
	//筛选器低16位每位必须匹配
	CAN_FilterInitStructure1.FilterMaskIdLow= 0x0000;			
	//筛选器被关联到FIFO1
	CAN_FilterInitStructure1.FilterFIFOAssignment=CAN_FILTER_FIFO0 ;	
	//使能筛选器
	CAN_FilterInitStructure1.FilterActivation=ENABLE;
//	CAN_FilterInitStructure1.SlaveStartFilterBank = 14;//从筛选器组	
	HAL_CAN_ConfigFilter(&hcan2,&CAN_FilterInitStructure1);

HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); 
HAL_CAN_Start(&hcan2);
	//检测中断是否开启
if (__HAL_CAN_GET_IT_SOURCE(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != RESET)
{
    // 中断已启用
    // 这里可以添加其他需要执行的代码
	//tset2=1;
}


}

void fifio_Init(void)
{
//	CAN_FilterTypeDef  CAN_FilterInitStructure;

//	/*CAN1筛选器初始化*/
//	CAN_FilterInitStructure.FilterBank=0;	//筛选器组0
//	//工作在掩码模式
//	CAN_FilterInitStructure.FilterMode=CAN_FILTERMODE_IDMASK;	
//	//筛选器位宽为单个32位。
//	CAN_FilterInitStructure.FilterScale=CAN_FILTERSCALE_32BIT;	
//	/* 使能筛选器，按照标志的内容进行比对筛选，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */
//	//要筛选的ID高位
//	CAN_FilterInitStructure.FilterIdHigh= 0x0000;	
//	//要筛选的ID低位 
//	CAN_FilterInitStructure.FilterIdLow= 0x0000; 
//	//筛选器高16位每位必须匹配
//	CAN_FilterInitStructure.FilterMaskIdHigh= 0x0000;			
//	//筛选器低16位每位必须匹配
//	CAN_FilterInitStructure.FilterMaskIdLow= 0x0000;			
//	//筛选器被关联到FIFO0
//	CAN_FilterInitStructure.FilterFIFOAssignment=CAN_FILTER_FIFO0 ;	
//	//使能筛选器
//	CAN_FilterInitStructure.FilterActivation=ENABLE;
//	CAN_FilterInitStructure.SlaveStartFilterBank = 14;//从筛选器组	
//	HAL_CAN_ConfigFilter(&hcan1,&CAN_FilterInitStructure);
//	
//	/*CAN2筛选器初始化*/
//	CAN_FilterInitStructure.FilterBank=14;	//筛选器组0
//	//工作在掩码模式
//	CAN_FilterInitStructure.FilterMode=CAN_FILTERMODE_IDMASK;	
//	//筛选器位宽为单个32位。
//	CAN_FilterInitStructure.FilterScale=CAN_FILTERSCALE_32BIT;	
//	/* 使能筛选器，按照标志的内容进行比对筛选，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO1。 */
//	//要筛选的ID高位
//	CAN_FilterInitStructure.FilterIdHigh= 0x0000;	
//	//要筛选的ID低位 
//	CAN_FilterInitStructure.FilterIdLow= 0x0000; 
//	//筛选器高16位每位必须匹配
//	CAN_FilterInitStructure.FilterMaskIdHigh= 0x0000;			
//	//筛选器低16位每位必须匹配
//	CAN_FilterInitStructure.FilterMaskIdLow= 0x0000;			
//	//筛选器被关联到FIFO1
//	CAN_FilterInitStructure.FilterFIFOAssignment=CAN_FILTER_FIFO1 ;	
//	//使能筛选器
//	CAN_FilterInitStructure.FilterActivation=ENABLE;	
//	CAN_FilterInitStructure.SlaveStartFilterBank = 14;		
//	HAL_CAN_ConfigFilter(&hcan2,&CAN_FilterInitStructure);
//	
//	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); 
//	HAL_CAN_Start(&hcan1);
//	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING); 	
//	HAL_CAN_Start(&hcan2);
	
}
