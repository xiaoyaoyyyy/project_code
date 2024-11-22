
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

	//����ж��Ƿ���
if (__HAL_CAN_GET_IT_SOURCE(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != RESET)
{
    // �ж�������
    // ����������������Ҫִ�еĴ���
	tset1=1;
}

CAN_FilterTypeDef  CAN_FilterInitStructure;

	/*CAN1ɸѡ����ʼ��*/
	CAN_FilterInitStructure.FilterBank=0;	//ɸѡ����0
	//����������ģʽ
	CAN_FilterInitStructure.FilterMode=CAN_FILTERMODE_IDMASK;	
	//ɸѡ��λ��Ϊ����32λ��
	CAN_FilterInitStructure.FilterScale=CAN_FILTERSCALE_32BIT;	
	/* ʹ��ɸѡ�������ձ�־�����ݽ��бȶ�ɸѡ����չID�������µľ����������ǵĻ��������FIFO0�� */
	//Ҫɸѡ��ID��λ
	CAN_FilterInitStructure.FilterIdHigh= 0x0000;	
	//Ҫɸѡ��ID��λ 
	CAN_FilterInitStructure.FilterIdLow= 0x0000; 
	//ɸѡ����16λÿλ����ƥ��
	CAN_FilterInitStructure.FilterMaskIdHigh= 0x0000;			
	//ɸѡ����16λÿλ����ƥ��
	CAN_FilterInitStructure.FilterMaskIdLow= 0x0000;			
	//ɸѡ����������FIFO0
	CAN_FilterInitStructure.FilterFIFOAssignment=CAN_FILTER_FIFO0 ;	
	//ʹ��ɸѡ��
	CAN_FilterInitStructure.FilterActivation=ENABLE;
//	CAN_FilterInitStructure.SlaveStartFilterBank = 14;//��ɸѡ����	
	HAL_CAN_ConfigFilter(&hcan1,&CAN_FilterInitStructure);

HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); 
HAL_CAN_Start(&hcan1);
	//����ж��Ƿ���
if (__HAL_CAN_GET_IT_SOURCE(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != RESET)
{
    // �ж�������
    // ����������������Ҫִ�еĴ���
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

	//����ж��Ƿ���
if (__HAL_CAN_GET_IT_SOURCE(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != RESET)
{
    // �ж�������
    // ����������������Ҫִ�еĴ���
	tset1=1;
}

CAN_FilterTypeDef  CAN_FilterInitStructure1;

	/*CAN1ɸѡ����ʼ��*/
	CAN_FilterInitStructure1.FilterBank=14;	//ɸѡ����14
	//����������ģʽ
	CAN_FilterInitStructure1.FilterMode=CAN_FILTERMODE_IDMASK;	
	//ɸѡ��λ��Ϊ����32λ��
	CAN_FilterInitStructure1.FilterScale=CAN_FILTERSCALE_32BIT;	
	/* ʹ��ɸѡ�������ձ�־�����ݽ��бȶ�ɸѡ����չID�������µľ����������ǵĻ��������FIFO0�� */
	//Ҫɸѡ��ID��λ
	CAN_FilterInitStructure1.FilterIdHigh= 0x0000;	
	//Ҫɸѡ��ID��λ 
	CAN_FilterInitStructure1.FilterIdLow= 0x0000; 
	//ɸѡ����16λÿλ����ƥ��
	CAN_FilterInitStructure1.FilterMaskIdHigh= 0x0000;			
	//ɸѡ����16λÿλ����ƥ��
	CAN_FilterInitStructure1.FilterMaskIdLow= 0x0000;			
	//ɸѡ����������FIFO1
	CAN_FilterInitStructure1.FilterFIFOAssignment=CAN_FILTER_FIFO0 ;	
	//ʹ��ɸѡ��
	CAN_FilterInitStructure1.FilterActivation=ENABLE;
//	CAN_FilterInitStructure1.SlaveStartFilterBank = 14;//��ɸѡ����	
	HAL_CAN_ConfigFilter(&hcan2,&CAN_FilterInitStructure1);

HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); 
HAL_CAN_Start(&hcan2);
	//����ж��Ƿ���
if (__HAL_CAN_GET_IT_SOURCE(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != RESET)
{
    // �ж�������
    // ����������������Ҫִ�еĴ���
	//tset2=1;
}


}

void fifio_Init(void)
{
//	CAN_FilterTypeDef  CAN_FilterInitStructure;

//	/*CAN1ɸѡ����ʼ��*/
//	CAN_FilterInitStructure.FilterBank=0;	//ɸѡ����0
//	//����������ģʽ
//	CAN_FilterInitStructure.FilterMode=CAN_FILTERMODE_IDMASK;	
//	//ɸѡ��λ��Ϊ����32λ��
//	CAN_FilterInitStructure.FilterScale=CAN_FILTERSCALE_32BIT;	
//	/* ʹ��ɸѡ�������ձ�־�����ݽ��бȶ�ɸѡ����չID�������µľ����������ǵĻ��������FIFO0�� */
//	//Ҫɸѡ��ID��λ
//	CAN_FilterInitStructure.FilterIdHigh= 0x0000;	
//	//Ҫɸѡ��ID��λ 
//	CAN_FilterInitStructure.FilterIdLow= 0x0000; 
//	//ɸѡ����16λÿλ����ƥ��
//	CAN_FilterInitStructure.FilterMaskIdHigh= 0x0000;			
//	//ɸѡ����16λÿλ����ƥ��
//	CAN_FilterInitStructure.FilterMaskIdLow= 0x0000;			
//	//ɸѡ����������FIFO0
//	CAN_FilterInitStructure.FilterFIFOAssignment=CAN_FILTER_FIFO0 ;	
//	//ʹ��ɸѡ��
//	CAN_FilterInitStructure.FilterActivation=ENABLE;
//	CAN_FilterInitStructure.SlaveStartFilterBank = 14;//��ɸѡ����	
//	HAL_CAN_ConfigFilter(&hcan1,&CAN_FilterInitStructure);
//	
//	/*CAN2ɸѡ����ʼ��*/
//	CAN_FilterInitStructure.FilterBank=14;	//ɸѡ����0
//	//����������ģʽ
//	CAN_FilterInitStructure.FilterMode=CAN_FILTERMODE_IDMASK;	
//	//ɸѡ��λ��Ϊ����32λ��
//	CAN_FilterInitStructure.FilterScale=CAN_FILTERSCALE_32BIT;	
//	/* ʹ��ɸѡ�������ձ�־�����ݽ��бȶ�ɸѡ����չID�������µľ����������ǵĻ��������FIFO1�� */
//	//Ҫɸѡ��ID��λ
//	CAN_FilterInitStructure.FilterIdHigh= 0x0000;	
//	//Ҫɸѡ��ID��λ 
//	CAN_FilterInitStructure.FilterIdLow= 0x0000; 
//	//ɸѡ����16λÿλ����ƥ��
//	CAN_FilterInitStructure.FilterMaskIdHigh= 0x0000;			
//	//ɸѡ����16λÿλ����ƥ��
//	CAN_FilterInitStructure.FilterMaskIdLow= 0x0000;			
//	//ɸѡ����������FIFO1
//	CAN_FilterInitStructure.FilterFIFOAssignment=CAN_FILTER_FIFO1 ;	
//	//ʹ��ɸѡ��
//	CAN_FilterInitStructure.FilterActivation=ENABLE;	
//	CAN_FilterInitStructure.SlaveStartFilterBank = 14;		
//	HAL_CAN_ConfigFilter(&hcan2,&CAN_FilterInitStructure);
//	
//	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); 
//	HAL_CAN_Start(&hcan1);
//	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING); 	
//	HAL_CAN_Start(&hcan2);
	
}
