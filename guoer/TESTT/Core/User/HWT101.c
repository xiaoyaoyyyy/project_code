/**
  ******************************************************************************
  * @file    HWT605.h
  * @author  robocon electronic control
						 刘易隆
  * @version v2.0
  * @date    2023年3月30日
  * @brief  	HWT605陀螺仪数据采集模块的开发
  ******************************************************************************
  * @attention		维特智能陀螺仪通用传输协议解析，注意单轴和多轴陀螺仪DMA接收解析协议不同
  *
  ******************************************************************************
  */
#include "HWT101.h"	//其余头文件均存放在.h的文档里面

Data Palstance;	//角速度数据结构体
Data Angle;			//角度数据结构体
all_data All_Data;		//存放所有处理后数据的结构体
uint8_t  kRxBuffer[22];
//模块初始化配置内容数组封装从这里开始
uint8_t ALG[5]   = {0xFF,0xAA,0x24,0x01,0x00};  //转换成六轴算法
uint8_t CLEAR[5] = {0xFF,0xAA,0x69,0x88,0xB5};	//解锁指令
uint8_t SLEEP[5] = {0xFF,0xAA,0x22,0x01,0x00};	//休眠与解休眠
uint8_t RATE_10[5]  = {0xFF,0xAA,0x03,0x06,0x00};	//设置回传速率10MHz （设置完成以后再给模块重新上电后生效）
uint8_t RATE_20[5]  = {0xFF,0xAA,0x03,0x07,0x00};	//设置回传速率20MHz （设置完成以后再给模块重新上电后生效）
uint8_t RATE_50[5]  = {0xFF,0xAA,0x03,0x08,0x00};	//设置回传速率50MHz （设置完成以后再给模块重新上电后生效）
uint8_t RATE_100[5] = {0xFF,0xAA,0x03,0x09,0x00}; //设置回传速率100MHz（设置完成以后再给模块重新上电后生效）
uint8_t GYRO_ON[5]  = {0xFF,0xAA,0x63,0x00,0x00};	//开启陀螺仪自动校准
uint8_t GYRO_OFF[5] = {0xFF,0xAA,0x63,0x01,0x00};	//关闭陀螺仪自动校准
uint8_t BAUD_9600[5]   = {0xFF,0xAA,0x04,0x02,0x00};	//设置串口波特率(9600)
uint8_t BAUD_115200[5] = {0xFF,0xAA,0x04,0x06,0x00};	//设置串口波特率(115200)
uint8_t CALSW_speed[5] = {0xFF,0xAA,0x01,0x01,0x00};	//进入加速度校准模式
uint8_t CALSW_OFF[5] 	 = {0xFF,0xAA,0x01,0x00,0x00};	//退出校准模式
uint8_t SAVE_current[5]  = {0xFF,0xAA,0x00,0x00,0x00};	//保持当前配置
uint8_t SAVE_recover[5]  = {0xFF,0xAA,0x00,0x01,0x00};	//恢复默认（出厂）配置并保存
uint8_t DIRECTION_vertical[5] = {0xFF,0xAA,0x23,0x00,0x00};	//设置为水平安装
uint8_t DIRECTION_standard[5] = {0xFF,0xAA,0x23,0x01,0x00};	//设置为垂直安装
uint8_t CALSW[5] = {0xFF,0xAA,0x01,0x04,0x00};	//多轴Z轴角度置零
uint8_t CALIYAW[5] = {0xFF,0xAA,0x76,0x00,0x00};	//单轴Z轴角度置零
//模块初始化配置内容数组封装到这里结束


/***********************模块配置函数**********************
* @function 允许调用的函数
*						@arg SYSTEM_Init_only: 一个参数配置
*						@arg SYSTEM_Init_most: 多个参数配置
* @usage	对模块进行初始化配置
* @method	发送对应的数组数据对模块进行设置
* @param  可以使用的参数
*         @arg SLEEP: 休眠与解休眠（发送该指令模块进入休眠（待机）状态，再发送一次，模块从待机状态进入工作状态。）
*         @arg RATE_10: 设置回传速率为10Hz
*         @arg RATE_20: 设置回传速率为20Hz
*         @arg RATE_50: 设置回传速率为50Hz
*         @arg RATE_100:设置回传速率为100Hz
*         @arg GYRO_ON: 开启陀螺仪自动校准
*         @arg GYRO_OFF:关闭陀螺仪自动校准
*         @arg CALSW_OFF:  退出校准模式
*         @arg CALSW_speed:进入加速度校准模式
*         @arg BAUD_9600:  设置串口波特率(9600)
*         @arg BAUD_115200:设置串口波特率(115200)
*         @arg SAVE_recover: 恢复默认（出厂）配置并保存
*         @arg DIRECTION_vertical: 设置为水平安装
*         @arg DIRECTION_standard: 设置为垂直安装
* @attention 所有配置为掉电保存，配置一次后，后续使用可以不用配置
*****************************************************/
//单一配置时使用
//设置恢复默认（出厂）配置只能使用此函数
void SYSTEM_Init_only(UART_HandleTypeDef *huart, uint8_t* content)
{
//	HAL_UART_Transmit(huart,CLEAR,5,0xFF); 					 //解锁指令
	HAL_UART_Transmit(huart,content,5,0xFF);				 //配置参数设置
//	if(content == SAVE_recover)
//		return;
//	else
//		HAL_UART_Transmit(huart,SAVE_current,5,0xFF);	 //保持配置
}

//此函数为后面整体配置时调用，单独使用此函数进行配置没有用(操作麻烦)
void SET_configure(UART_HandleTypeDef *huart, uint8_t* content)
{
	HAL_UART_Transmit(huart,content,5,0xFF);	 		 //配置参数设置
}

//多数配置时使用，需要提前在函数配置前
//此函数不支持设置恢复默认（出厂）配置
void SYSTEM_Init_most(UART_HandleTypeDef *huart)
{
	HAL_UART_Transmit(huart,CLEAR,5,0xFF); 				 //解锁指令(允许开始配置)
	//修改配置从这里开始
//	SET_configure(huart,RATE_20);									 //设置回传速率为20Hz
//	SET_configure(huart,BAUD_115200);				 			 //设置串口波特率(115200)
	SET_configure(huart,CALSW);								//Z轴角度置零
//	SET_configure(huart,DIRECTION_vertical); 			 //设置为水平安装
	//修改配置到这里结束
	HAL_UART_Transmit(huart,SAVE_current,5,0xFF);	 //保持配置(配置窗口关闭)
}

/************************数据接收***********************
* @explain		每次接收一位数据，并检测是否符合标准
* @variable		aRxBuffer：数据缓存区
							ucData: 从缓存区调出数据
*******************************************************/
//通用陀螺仪数据解析(中断接收)
void Data_reception(unsigned char ucData)
{
	static unsigned char aRxBuffer[250];	//定义缓存区
	static unsigned char aRxnum = 0;
	aRxBuffer[aRxnum++] = ucData;
	if(aRxBuffer[0] != 0x55)			//检验包头
	{
		aRxnum=0;
		return;
	}
	if (aRxnum < 11)							//检验数据长度
	{
		return;
	}//数据不满11个返回
	else
	{
		switch (aRxBuffer[1])				//判断数据类型
		{
			case 0x52: memcpy(&Palstance,&aRxBuffer[2],8);
			HWT605_Data(0x52);
			break;			//角速度
			
			case 0x53: memcpy(&Angle,&aRxBuffer[2],8);
			HWT605_Data(0x53);
			break;			//角度
		}
		aRxnum = 0;	//清空缓存区
	}
}
////多轴陀螺仪解析(DMA空闲中断接收)
//void Data_reception_DMA(unsigned char ucData)
//{

//	if(kRxBuffer[11]==0x55&&kRxBuffer[12]==0x52)
//	{		
//			memcpy(&Palstance,&kRxBuffer[13],8);//角速度
//			HWT605_Data(0x52);
//	}
//	if(kRxBuffer[22]==0x55&&kRxBuffer[23]==0x53) 
//	{
//		memcpy(&Angle,&kRxBuffer[24],8);//角度
//		HWT605_Data(0x53);
//	}
//	
//}
//单轴陀螺仪解析(DMA空闲中断接收)
int mar4=0;
void Data_reception_DMA_only(unsigned char ucData)
{

	if(kRxBuffer[0]==0x55&&kRxBuffer[1]==0x52)
	{		
			memcpy(&Palstance,&kRxBuffer[2],8);//角速度
			HWT605_Data(0x52);
	}
	if(kRxBuffer[11]==0x55&&kRxBuffer[12]==0x53) 
	{
		mar4 = 1;
		memcpy(&Angle,&kRxBuffer[13],8);//角度
		HWT605_Data(0x53);
	}
	
}
/***********************数据解析**********************
* @usage	对接收到的数据进行解析处理
					(由于其余的数据不需要用到，所以没有解析)
*****************************************************/
void HWT605_Data(uint16_t mode)
{
	/**********角速度数据**********/
	if(mode == 0x52)
	{
		All_Data.Pal_X = Palstance.data[0]/32768.0*2000.0;  //X 轴
		All_Data.Pal_Y = Palstance.data[1]/32768.0*2000.0;  //Y 轴
		All_Data.Pal_Z = Palstance.data[2]/32768.0*2000.0;  //Z 轴
		All_Data.Pal_T = Palstance.T/100.0;  						    //温度
	}
	/**********角度数据**********/
	if(mode == 0x53)
	{
		All_Data.Ang_X = Angle.data[0]/32768.0*180.0;	 //X 轴
		All_Data.Ang_Y = Angle.data[1]/32768.0*180.0;  //Y 轴
		All_Data.Ang_Z = Angle.data[2]/32768.0*180.0;  //Z 轴
		All_Data.Ang_T = Angle.T/100.0;  						   //温度
	}
}

/***********************数据接收**********************
* @usage	DMA空闲中断处理
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
		 __HAL_UART_CLEAR_IDLEFLAG(&huart3);	//清除空闲中断标志位

		HWT605_length = sizeof(kRxBuffer) - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx); //计算接收数据长度
//		printf("数据长度:%d\r\n",__HAL_DMA_GET_COUNTER(&hdma_usart3_rx));	
//		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
		if(HWT605_length==22)//单轴陀螺仪解析22位，多轴33位
		{
			mar3=1;
			Data_reception_DMA_only(*kRxBuffer);	//多轴陀螺仪解析
			memset(kRxBuffer, 0, sizeof(kRxBuffer));	//清空缓冲区
		}
		
	HAL_UART_Receive_DMA(huart, kRxBuffer, sizeof(kRxBuffer));
	}
}
