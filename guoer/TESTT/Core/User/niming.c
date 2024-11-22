#include "niming.h"
#include "usart.h"
int debug_usart;
//����1����1���ַ� 
//c:Ҫ���͵��ַ�
void usart_send_char(UART_HandleTypeDef *huart, uint8_t c)
{
	
  while(HAL_UART_Transmit(huart,&c,1,1)!=HAL_OK)
	{
		debug_usart++;
	}
	
}


//�������ݸ�����������λ�����(V2.6�汾)
//fun:������. 0XA0~0XAF
//data:���ݻ�����,���28�ֽ�!!
//len:data����Ч���ݸ���
void usart_niming_report(uint8_t fun,uint8_t*data,uint8_t len)
{
	int i;
	uint8_t send_buf[32];
	if(len>28)return;	//���28�ֽ�����
	send_buf[len+4]=0;	//У���������
	send_buf[len+5]=0;	//����У��������
	send_buf[0]=0xAA;//֡ͷ
	send_buf[1]=0xFF;//Ŀ���ַ
	send_buf[2]=fun;//������
	send_buf[3]=len;//���ݳ���
	for(i=0;i<len;i++)send_buf[4+i]=data[i];			//��������
	for(i=0;i<len+4;i++)
	{
  	send_buf[len+4]+=send_buf[i];	//����У���	
		send_buf[len+5]+=send_buf[len+4];	//����У���		
	}
	for(i=0;i<len+6;i++)usart_send_char(&huart2,send_buf[i]);	//�������ݵ�����2
}
void send_sensorData(int8_t aacx,int16_t aacy,int16_t aacz,int16_t gyrox,int16_t gyroy,int16_t gyroz)
{
	uint8_t tbuf[12]; 
	
	tbuf[0]=aacx;

  tbuf[1]=aacy&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	
  tbuf[3]=aacz&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	
	tbuf[5]=gyrox&0XFF;
	tbuf[6]=(gyrox>>8)&0XFF;
	
  tbuf[7]=gyroy&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	
	 tbuf[9]=gyroz&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	
	
	
// uint8_t *gyroxBytes = (uint8_t*)&gyrox;
//    tbuf[6] = gyroxBytes[0];
//    tbuf[7] = gyroxBytes[1];
//    tbuf[8] = gyroxBytes[2];
//    tbuf[9] = gyroxBytes[3];
	
	usart_niming_report(0XF1,tbuf,11);	
}


