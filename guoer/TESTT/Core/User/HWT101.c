/**
  ******************************************************************************
  * @file    HWT605.h
  * @author  robocon electronic control
						 ����¡
  * @version v2.0
  * @date    2023��3��30��
  * @brief  	HWT605���������ݲɼ�ģ��Ŀ���
  ******************************************************************************
  * @attention		ά������������ͨ�ô���Э�������ע�ⵥ��Ͷ���������DMA���ս���Э�鲻ͬ
  *
  ******************************************************************************
  */
#include "HWT101.h"	//����ͷ�ļ��������.h���ĵ�����

Data Palstance;	//���ٶ����ݽṹ��
Data Angle;			//�Ƕ����ݽṹ��
all_data All_Data;		//������д�������ݵĽṹ��
uint8_t  kRxBuffer[22];
//ģ���ʼ���������������װ�����￪ʼ
uint8_t ALG[5]   = {0xFF,0xAA,0x24,0x01,0x00};  //ת���������㷨
uint8_t CLEAR[5] = {0xFF,0xAA,0x69,0x88,0xB5};	//����ָ��
uint8_t SLEEP[5] = {0xFF,0xAA,0x22,0x01,0x00};	//�����������
uint8_t RATE_10[5]  = {0xFF,0xAA,0x03,0x06,0x00};	//���ûش�����10MHz ����������Ժ��ٸ�ģ�������ϵ����Ч��
uint8_t RATE_20[5]  = {0xFF,0xAA,0x03,0x07,0x00};	//���ûش�����20MHz ����������Ժ��ٸ�ģ�������ϵ����Ч��
uint8_t RATE_50[5]  = {0xFF,0xAA,0x03,0x08,0x00};	//���ûش�����50MHz ����������Ժ��ٸ�ģ�������ϵ����Ч��
uint8_t RATE_100[5] = {0xFF,0xAA,0x03,0x09,0x00}; //���ûش�����100MHz����������Ժ��ٸ�ģ�������ϵ����Ч��
uint8_t GYRO_ON[5]  = {0xFF,0xAA,0x63,0x00,0x00};	//�����������Զ�У׼
uint8_t GYRO_OFF[5] = {0xFF,0xAA,0x63,0x01,0x00};	//�ر��������Զ�У׼
uint8_t BAUD_9600[5]   = {0xFF,0xAA,0x04,0x02,0x00};	//���ô��ڲ�����(9600)
uint8_t BAUD_115200[5] = {0xFF,0xAA,0x04,0x06,0x00};	//���ô��ڲ�����(115200)
uint8_t CALSW_speed[5] = {0xFF,0xAA,0x01,0x01,0x00};	//������ٶ�У׼ģʽ
uint8_t CALSW_OFF[5] 	 = {0xFF,0xAA,0x01,0x00,0x00};	//�˳�У׼ģʽ
uint8_t SAVE_current[5]  = {0xFF,0xAA,0x00,0x00,0x00};	//���ֵ�ǰ����
uint8_t SAVE_recover[5]  = {0xFF,0xAA,0x00,0x01,0x00};	//�ָ�Ĭ�ϣ����������ò�����
uint8_t DIRECTION_vertical[5] = {0xFF,0xAA,0x23,0x00,0x00};	//����Ϊˮƽ��װ
uint8_t DIRECTION_standard[5] = {0xFF,0xAA,0x23,0x01,0x00};	//����Ϊ��ֱ��װ
uint8_t CALSW[5] = {0xFF,0xAA,0x01,0x04,0x00};	//����Z��Ƕ�����
uint8_t CALIYAW[5] = {0xFF,0xAA,0x76,0x00,0x00};	//����Z��Ƕ�����
//ģ���ʼ���������������װ���������


/***********************ģ�����ú���**********************
* @function ������õĺ���
*						@arg SYSTEM_Init_only: һ����������
*						@arg SYSTEM_Init_most: �����������
* @usage	��ģ����г�ʼ������
* @method	���Ͷ�Ӧ���������ݶ�ģ���������
* @param  ����ʹ�õĲ���
*         @arg SLEEP: ����������ߣ����͸�ָ��ģ��������ߣ�������״̬���ٷ���һ�Σ�ģ��Ӵ���״̬���빤��״̬����
*         @arg RATE_10: ���ûش�����Ϊ10Hz
*         @arg RATE_20: ���ûش�����Ϊ20Hz
*         @arg RATE_50: ���ûش�����Ϊ50Hz
*         @arg RATE_100:���ûش�����Ϊ100Hz
*         @arg GYRO_ON: �����������Զ�У׼
*         @arg GYRO_OFF:�ر��������Զ�У׼
*         @arg CALSW_OFF:  �˳�У׼ģʽ
*         @arg CALSW_speed:������ٶ�У׼ģʽ
*         @arg BAUD_9600:  ���ô��ڲ�����(9600)
*         @arg BAUD_115200:���ô��ڲ�����(115200)
*         @arg SAVE_recover: �ָ�Ĭ�ϣ����������ò�����
*         @arg DIRECTION_vertical: ����Ϊˮƽ��װ
*         @arg DIRECTION_standard: ����Ϊ��ֱ��װ
* @attention ��������Ϊ���籣�棬����һ�κ󣬺���ʹ�ÿ��Բ�������
*****************************************************/
//��һ����ʱʹ��
//���ûָ�Ĭ�ϣ�����������ֻ��ʹ�ô˺���
void SYSTEM_Init_only(UART_HandleTypeDef *huart, uint8_t* content)
{
//	HAL_UART_Transmit(huart,CLEAR,5,0xFF); 					 //����ָ��
	HAL_UART_Transmit(huart,content,5,0xFF);				 //���ò�������
//	if(content == SAVE_recover)
//		return;
//	else
//		HAL_UART_Transmit(huart,SAVE_current,5,0xFF);	 //��������
}

//�˺���Ϊ������������ʱ���ã�����ʹ�ô˺�����������û����(�����鷳)
void SET_configure(UART_HandleTypeDef *huart, uint8_t* content)
{
	HAL_UART_Transmit(huart,content,5,0xFF);	 		 //���ò�������
}

//��������ʱʹ�ã���Ҫ��ǰ�ں�������ǰ
//�˺�����֧�����ûָ�Ĭ�ϣ�����������
void SYSTEM_Init_most(UART_HandleTypeDef *huart)
{
	HAL_UART_Transmit(huart,CLEAR,5,0xFF); 				 //����ָ��(����ʼ����)
	//�޸����ô����￪ʼ
//	SET_configure(huart,RATE_20);									 //���ûش�����Ϊ20Hz
//	SET_configure(huart,BAUD_115200);				 			 //���ô��ڲ�����(115200)
	SET_configure(huart,CALSW);								//Z��Ƕ�����
//	SET_configure(huart,DIRECTION_vertical); 			 //����Ϊˮƽ��װ
	//�޸����õ��������
	HAL_UART_Transmit(huart,SAVE_current,5,0xFF);	 //��������(���ô��ڹر�)
}

/************************���ݽ���***********************
* @explain		ÿ�ν���һλ���ݣ�������Ƿ���ϱ�׼
* @variable		aRxBuffer�����ݻ�����
							ucData: �ӻ�������������
*******************************************************/
//ͨ�����������ݽ���(�жϽ���)
void Data_reception(unsigned char ucData)
{
	static unsigned char aRxBuffer[250];	//���建����
	static unsigned char aRxnum = 0;
	aRxBuffer[aRxnum++] = ucData;
	if(aRxBuffer[0] != 0x55)			//�����ͷ
	{
		aRxnum=0;
		return;
	}
	if (aRxnum < 11)							//�������ݳ���
	{
		return;
	}//���ݲ���11������
	else
	{
		switch (aRxBuffer[1])				//�ж���������
		{
			case 0x52: memcpy(&Palstance,&aRxBuffer[2],8);
			HWT605_Data(0x52);
			break;			//���ٶ�
			
			case 0x53: memcpy(&Angle,&aRxBuffer[2],8);
			HWT605_Data(0x53);
			break;			//�Ƕ�
		}
		aRxnum = 0;	//��ջ�����
	}
}
////���������ǽ���(DMA�����жϽ���)
//void Data_reception_DMA(unsigned char ucData)
//{

//	if(kRxBuffer[11]==0x55&&kRxBuffer[12]==0x52)
//	{		
//			memcpy(&Palstance,&kRxBuffer[13],8);//���ٶ�
//			HWT605_Data(0x52);
//	}
//	if(kRxBuffer[22]==0x55&&kRxBuffer[23]==0x53) 
//	{
//		memcpy(&Angle,&kRxBuffer[24],8);//�Ƕ�
//		HWT605_Data(0x53);
//	}
//	
//}
//���������ǽ���(DMA�����жϽ���)
int mar4=0;
void Data_reception_DMA_only(unsigned char ucData)
{

	if(kRxBuffer[0]==0x55&&kRxBuffer[1]==0x52)
	{		
			memcpy(&Palstance,&kRxBuffer[2],8);//���ٶ�
			HWT605_Data(0x52);
	}
	if(kRxBuffer[11]==0x55&&kRxBuffer[12]==0x53) 
	{
		mar4 = 1;
		memcpy(&Angle,&kRxBuffer[13],8);//�Ƕ�
		HWT605_Data(0x53);
	}
	
}
/***********************���ݽ���**********************
* @usage	�Խ��յ������ݽ��н�������
					(������������ݲ���Ҫ�õ�������û�н���)
*****************************************************/
void HWT605_Data(uint16_t mode)
{
	/**********���ٶ�����**********/
	if(mode == 0x52)
	{
		All_Data.Pal_X = Palstance.data[0]/32768.0*2000.0;  //X ��
		All_Data.Pal_Y = Palstance.data[1]/32768.0*2000.0;  //Y ��
		All_Data.Pal_Z = Palstance.data[2]/32768.0*2000.0;  //Z ��
		All_Data.Pal_T = Palstance.T/100.0;  						    //�¶�
	}
	/**********�Ƕ�����**********/
	if(mode == 0x53)
	{
		All_Data.Ang_X = Angle.data[0]/32768.0*180.0;	 //X ��
		All_Data.Ang_Y = Angle.data[1]/32768.0*180.0;  //Y ��
		All_Data.Ang_Z = Angle.data[2]/32768.0*180.0;  //Z ��
		All_Data.Ang_T = Angle.T/100.0;  						   //�¶�
	}
}

/***********************���ݽ���**********************
* @usage	DMA�����жϴ���
*****************************************************/
int mar1=0,mar2=0,mar3=0;
void USER_GYRO_IDLE_IRQHandler(UART_HandleTypeDef *huart)
{
	mar1 =1;
	static unsigned int HWT605_length = 0;
	HAL_UART_DMAStop(huart);
	if (huart->Instance == USART3)
	{
		mar2=1;
		 __HAL_UART_CLEAR_IDLEFLAG(&huart3);	//��������жϱ�־λ

		HWT605_length = sizeof(kRxBuffer) - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx); //����������ݳ���
//		printf("���ݳ���:%d\r\n",__HAL_DMA_GET_COUNTER(&hdma_usart3_rx));	
//		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
		if(HWT605_length==22)//���������ǽ���22λ������33λ
		{
			mar3=1;
			Data_reception_DMA_only(*kRxBuffer);	//���������ǽ���
			memset(kRxBuffer, 0, sizeof(kRxBuffer));	//��ջ�����
		}
		
	HAL_UART_Receive_DMA(huart, kRxBuffer, sizeof(kRxBuffer));
	}
}
