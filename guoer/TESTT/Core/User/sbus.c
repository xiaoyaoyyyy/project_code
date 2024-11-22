/**
  ******************************************************************************
  * @file    sbus.c
  * @author  robocon electronic control 
  * @version V_1.0.0
  * @date    2023年2月18日
  * @brief   有关D文件，用于遥控接收数据解码
  *          
  ******************************************************************************
  * @attention  
  *	@Rely    
  *          
  *          
  ******************************************************************************
  */
#include "sbus.h"
#include "usart.h"

sbus_t sbus_struct;

uint8_t sbus_flag =1;
uint8_t remote_buf[25];//接收数据缓存区
bool receiving_data ,receiving_data1,rx_R2_flag= false;
int  rx_R1,rx_R2,rx_V1,rx_V2=0;
uint8_t rx_buffer_R[17];
uint8_t rx_buffer_V[19];
uint32_t sbus_data_length=0;		
int test_r1,test_r2,test_r3,test_r4,test_r5,test_r6;
SensorData receivedDataa[2];

/**
 * @brief	串口中断回调函数
 * @param		--huart：串口端口w
 * @ratval  none
 * @attention 在"stm32f4xx_it.c"文件中 USART1_IRQHandler() 函数中调用
*/

void reset_uart(UART_HandleTypeDef *huart) {
    // 先禁用UART接收
    HAL_UART_AbortReceive(huart);

    // 清空接收缓冲区
    memset(remote_buf, 0, sizeof(remote_buf));

    // 重新初始化UART外设
    HAL_UART_DeInit(huart);
    HAL_UART_Init(huart);

    // 重新启用UART接收中断
    HAL_UART_Receive_IT(huart, remote_buf, sizeof(remote_buf));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{		
		
	//aaaa=huart->RxXferCount;
    if (huart->Instance == USART1)
		{

			sbus_data_length = sizeof(remote_buf);  // - aaaa; //计算接收数据长度
	    if(sbus_data_length==25 && remote_buf[0]==0x0F && remote_buf[24]==0x00)//数据帧头帧尾判断数据是否异常
			{test_r1++;
				sbus_decoder(remote_buf);
			 	Memset(remote_buf, 0, 25);	//清空缓冲区
				HAL_UART_Receive_IT(huart, remote_buf, 25);
			}
			else
			{
				test_r2++;
				
	      reset_uart(huart);
			}
			
    }
		//雷达
				if (huart->Instance == USART2)
			{	
					g_uasat1_rx_flag =1;
				if (rx_buffer_R[0] == FRAME_HEADER1) {
							// 如果帧头正确，继续接收剩余的数据
								test_r5++;
								processReceivedData_R(huart,RADAR,R_NUMBER);
								last_receive_time = HAL_GetTick();
								receiving_data = true;
					//		Memset(rx_buffer[RADAR],0,R_FRAME_SIZE_BYTES);
							HAL_UART_Receive_IT(huart, rx_buffer_R, R_FRAME_SIZE_BYTES);
						}	
					else {			
							test_r6++;
						//    HAL_UART_AbortReceive(huart);// 先禁用UART接收							
									memset(rx_buffer_R, 0, sizeof(rx_buffer_R));// 清空接收缓冲区								
					//				HAL_UART_DeInit(huart);  // 重新初始化UART外设
					//				HAL_UART_Init(huart);
									// 重新启用UART接收中断
									HAL_UART_Receive_IT(huart, rx_buffer_R, sizeof(rx_buffer_R));
					}
			 }

		 
		//视觉
		 	if (huart->Instance == USART6)
			{
				   g_uasat1_rx_flag =1;
				if (rx_buffer_V[0] == FRAME_HEADER1) {
test_r3++		;	
					 rx_V1=rx_buffer_V[17];
					 rx_V2=rx_buffer_V[18];
						// 如果帧头正确，继续接收剩余的数据
        processReceivedData_V(huart,VISION,V_NUMBER);
							last_receive_time1 = HAL_GetTick();
							receiving_data1 = true;
					 Memset(rx_buffer_V,0,V_FRAME_SIZE_BYTES);
					 
						HAL_UART_Receive_IT(huart, rx_buffer_V, sizeof(rx_buffer_V));			// 如果帧头不正确，重新开始接收
				}	
				else {				 
												// 先禁用UART接收
							HAL_UART_AbortReceive(huart);
test_r4++;
							// 清空接收缓冲区
							memset(rx_buffer_V, 0, sizeof(rx_buffer_V));

							// 重新初始化UART外设
							HAL_UART_DeInit(huart);
							HAL_UART_Init(huart);

							// 重新启用UART接收中断
							HAL_UART_Receive_IT(huart, rx_buffer_V, sizeof(rx_buffer_V));
				}
		
		}
}


/**
 * @brief	清除缓冲区
 * @param	--Remote_buff：缓存区变量名
					--data：缓存区替换数据
					--Len： 缓存区大小
 * @ratval  none
 * @attention none
*/
void Memset(uint8_t* Remote_buff,int data,uint16_t Len)
{
   for(;Len>0;Len--)
	{
	   Remote_buff[Len]=data;	
	}
}

void sbus_decoder(uint8_t* frame) {
	
	sbus_flag = 0;	
  sbus_struct.sbus_channels[ 1] = (frame[ 1] >> 0 | frame[ 2] << 8) & 0x7ff;
	sbus_struct.sbus_channels[ 2] = (frame[ 2] >> 3 | frame[ 3] << 5) & 0x7ff;
	sbus_struct.sbus_channels[ 3] = (frame[ 3] >> 6 | frame[ 4] << 2 | frame[5] << 10) & 0x7ff;
	sbus_struct.sbus_channels[ 4] = (frame[ 5] >> 1 | frame[ 6] << 7) & 0x7ff;
	sbus_struct.sbus_channels[ 5] = (frame[ 6] >> 4 | frame[ 7] << 4) & 0x7ff;
	sbus_struct.sbus_channels[ 6] = (frame[ 7] >> 7 | frame[ 8] << 1 | frame[9] << 9) & 0x7ff;
	sbus_struct.sbus_channels[ 7] = (frame[ 9] >> 2 | frame[10] << 6) & 0x7ff;
	sbus_struct.sbus_channels[ 8] = (frame[10] >> 5 | frame[11] << 3) & 0x7ff;
	
	sbus_struct.sbus_channels[ 9] = (frame[12] << 0 | frame[13] << 8) & 0x7ff;
	sbus_struct.sbus_channels[10] = (frame[13] >> 3 | frame[14] << 5) & 0x7ff;
	sbus_struct.sbus_channels[11] = (frame[15] >> 6 | frame[16] << 2 | frame[17] << 10) & 0x7ff;
	sbus_struct.sbus_channels[12] = (frame[17] >> 1 | frame[18] << 7) & 0x7ff;
	sbus_struct.sbus_channels[13] = (frame[18] >> 4 | frame[19] << 4) & 0x7ff;
	sbus_struct.sbus_channels[14] = (frame[19] >> 7 | frame[20] << 1 | frame[19] << 9) & 0x7ff;
	sbus_struct.sbus_channels[15] = (frame[20] >> 2 | frame[21] << 6) & 0x7ff;
	sbus_struct.sbus_channels[16] = (frame[21] >> 5 | frame[22] << 3) & 0x7ff;
	
	sbus_struct.digital[0] = frame[23] & 0x01;	//ditital channel 17.
	sbus_struct.digital[1] = frame[23] & 0x02;	//ditital channel 18.
	
	sbus_struct.frame_lost = (frame[23] & 0x04)?1:0;	//frame is mission.
	sbus_struct.failsafe_activated = (frame[23] & 0x08)?1:0;	//status bit fault.

}

/* 
一帧字节25个字节，22个数据，1帧头1帧尾1标准位
一共16个通道，1个通道11bit（位）16*11 = 22*8
帧头0x0f
*/

void processReceivedData_R(UART_HandleTypeDef *huart,int row,int number) 
{
		double receivedData[number];

     for (int i = 0; i < number; i++) {
        memcpy(&receivedData[i], &rx_buffer_R[(i * 8) + 1], sizeof(double));

        // Check if the system is big-endian, reverse if necessary
        uint32_t testValue = 1;
        if (*((uint8_t*)&testValue) != 1) {
            reverse_bytes(&receivedData[i], sizeof(double));
        }
    }
		//		have_arrivied=0;
        receivedDataa[row].A = receivedData[0];
				receivedDataa[row].B = receivedData[1];
				receivedDataa[row].C = receivedData[2];
				receivedDataa[row].D = receivedData[3];
				receivedDataa[row].E = receivedData[4];
		
}
void processReceivedData_V(UART_HandleTypeDef *huart,int row,int number) 
{
		double receivedData[number];

     for (int i = 0; i < number; i++) {
        memcpy(&receivedData[i], &rx_buffer_V[(i * 8) + 1], sizeof(double));

        // Check if the system is big-endian, reverse if necessary
        uint32_t testValue = 1;
        if (*((uint8_t*)&testValue) != 1) {
            reverse_bytes(&receivedData[i], sizeof(double));
        }
    }
		//		have_arrivied=0;
        receivedDataa[row].A = receivedData[0];
				receivedDataa[row].B = receivedData[1];
				receivedDataa[row].C = receivedData[2];
				receivedDataa[row].D = receivedData[3];
				receivedDataa[row].E = receivedData[4];
		
}


void reverse_bytes(void *ptr, size_t size) {
    uint8_t *bytes = (uint8_t *)ptr;
    size_t i, j;
    for (i = 0, j = size - 1; i < j; i++, j--) {
        uint8_t temp = bytes[i];
        bytes[i] = bytes[j];
        bytes[j] = temp;
    }
}
void checkRxBufferTimeout(void) {
    if (receiving_data) {
        uint32_t current_time = HAL_GetTick(); // 获取当前时间（以毫秒为单位）
        if ((current_time - last_receive_time) > TIMEOUT_THRESHOLD) 
					{
            // 如果超过了阈值且不再接收数据，将 rx_buffer 清零
            memset(rx_buffer_R, 0, R_FRAME_SIZE_BYTES);
            receiving_data = false;
			   		processReceivedData_R(&huart2,RADAR,R_NUMBER);
					
        }
    }
		  if (receiving_data1) {
        uint32_t current_time = HAL_GetTick(); // 获取当前时间（以毫秒为单位）
        if ((current_time - last_receive_time1) > 1800) 
					{
            // 如果超过了阈值且不再接收数据，将 rx_buffer 清零
            memset(rx_buffer_V, 0, V_FRAME_SIZE_BYTES);
            receiving_data1 = false;
			   		processReceivedData_V(&huart6,VISION,V_NUMBER);
					
        }
    }
}

