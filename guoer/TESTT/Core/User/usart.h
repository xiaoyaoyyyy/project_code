#ifndef _USART_H_
#define _USART_H_

#include "main.h"
#include <stdbool.h>

#define FRAME_HEADER1 0xA5
#define FRAME_HEADER2 0xDD
#define FRAME_TAIL 0x5A
#define R_NUMBER 	2
#define V_NUMBER 	2
#define R_FRAME_SIZE_BYTES (sizeof(double) * R_NUMBER)+1	
#define V_FRAME_SIZE_BYTES (sizeof(double) * V_NUMBER)+3	
#define ROW  2
#define RADAR  0
#define VISION 1
#define ZIZHUAN 2
#define qianjin 3
typedef struct {
    double A;
    double B;
    double C;
    double D;
    double E;	
    double F;
} SensorData;



#define GTIM_TIMX_INT                       TIM4
#define GTIM_TIMX_INT_IRQn                  TIM4_IRQn
#define GTIM_TIMX_INT_IRQHandler            TIM4_IRQHandler
#define GTIM_TIMX_INT_CLK_ENABLE()          do{ __HAL_RCC_TIM4_CLK_ENABLE(); }while(0)  
#define TIMEOUT_THRESHOLD 2400


void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);
void Auto_chassic_mode(void);
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart3_rx;
//extern uint8_t g_rx_buffer[4];
extern uint8_t g_uasat1_rx_flag;
extern uint8_t rx_buffer_R[17];
extern uint8_t rx_buffer_V[19];
extern void usart_init(uint32_t baudrate);
void processReceivedData_R(UART_HandleTypeDef *huart,int row,int number) ;
void processReceivedData_V(UART_HandleTypeDef *huart,int row,int number) ;
extern void MX_DMA_Init(void);
extern void reverse_bytes(void *ptr, size_t size) ;
extern void checkRxBufferTimeout(void);
double bytesToDouble(const uint8_t* bytes);
extern uint32_t last_receive_time, last_receive_time1;
extern int have_arrivied ;
extern bool receiving_data1,receiving_data ;

#endif

