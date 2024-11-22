/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "main.h"
#include "cmsis_os.h"
#include "mode_choic.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "chassis.h"
#include "superstratum.h"
#include "usart.h"
#include "auto.h"
#include "add.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;
osThreadId myTask04Handle;
osThreadId myTask05Handle;
osThreadId myTask06Handle;
osThreadId myTask07Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Protect_Task(void const * argument);
void Chassic_Task(void const * argument);
void Superstratum_Task(void const * argument);
void Mode_Task(void const * argument);
void Auto_Task(void const * argument);
void Add_Task(void const * argument);
void FMS2_Task(void const * argument);
void Key_scan(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
int abcde=0;
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, Protect_Task, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, Chassic_Task,osPriorityNormal, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);
	
	osThreadDef(myTask03, Superstratum_Task,osPriorityNormal, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);
	
	osThreadDef(myTask04, Mode_Task,osPriorityNormal, 0, 128);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);
	
	osThreadDef(myTask05, Auto_Task,osPriorityNormal, 0, 128);
  myTask05Handle = osThreadCreate(osThread(myTask05), NULL);
	
		osThreadDef(myTask06, Add_Task,osPriorityNormal, 0, 128);
  myTask06Handle = osThreadCreate(osThread(myTask06), NULL);


if (myTask02Handle != NULL) {
    // 任务创建失败的处理代码
	abcde=1;
}


}

/* USER CODE BEGIN Header_Protect_Task */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Protect_Task */
void Protect_Task(void const * argument)
{
  /* USER CODE BEGIN Protect_Task */
  /* Infinite loop */
  for(;;)
  {
//		SYSTEM_Init_most(&huart5);//陀螺仪初始化
//		printf("角度Z:%f\r\n",All_Data.Ang_Z);	
    osDelay(50);//50ms
  }
  /* USER CODE END Protect_Task */
}

/* USER CODE BEGIN Header_Chassic_Task */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassic_Task */

int testt=0;
void Chassic_Task(void const * argument)
{
  /* USER CODE BEGIN Chassic_Task */
	uint32_t preViousWakeTime =osKernelSysTick();
  /* Infinite loop */
  for(;;)
  {
//		if(lock) set_m3508_disable();
//		else set_m3508_enable();

		Control_Task();//底盘控制任务
		
		osDelay(10);
//    osDelayUntil(&preViousWakeTime,5);//绝对延时
  }
  /* USER CODE END Chassic_Task */
}

/* USER CODE BEGIN Header_Shoot_Task */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shoot_Task */
void Superstratum_Task(void const * argument)
{
 
  for(;;)
  {
		control_sup_task();

		osDelay(10);
//		osDelayUntil(&preViousWakeTime,2);//绝对延时
  }
 
}



void Mode_Task(void const * argument)
{
  /* USER CODE BEGIN Shoot_Task */
//	uint32_t preViousWakeTime =osKernelSysTick();
  /* Infinite loop */
  for(;;)
  {
		testt++;
  mode_judge();
	//	All_cur();//发送5-8电流值
		osDelay(2);
//		osDelayUntil(&preViousWakeTime,2);//绝对延时
  }
	
}
/* USER CODE BEGIN Header_Put_Task */
/*
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Put_Task */
void Auto_Task(void const * argument)
{
  /* USER CODE BEGIN Put_Task */
	uint32_t preViousWakeTime =osKernelSysTick();
  /* Infinite loop */
  for(;;)
  {
		Auto_control_task();
		//Auto_control_task();
		osDelay(2);
//		All_cur();//发送5-8电流值
    //osDelayUntil(&preViousWakeTime,1);//绝对延时
  }
  /* USER CODE END Put_Task */
}

/* USER CODE BEGIN Header_FMS_Task */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FMS_Task */
void Add_Task(void const * argument)
{
  /* USER CODE BEGIN FMS_Task */
  /* Infinite loop */
  for(;;)
  {
		
		sup_add_task();
    osDelay(20);
  }
  /* USER CODE END FMS_Task */
}

/* USER CODE BEGIN Header_FMS2_Task */
/**
* @brief Function implementing the myTask06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FMS2_Task */
void FMS2_Task(void const * argument)
{
  /* USER CODE BEGIN FMS2_Task */
  /* Infinite loop */
  for(;;)
  {
	//	Put_FSM();//夹取抬升状态机
    osDelay(30);
  }
  /* USER CODE END FMS2_Task */
}

/* USER CODE BEGIN Header_Key_scan */
/**
* @brief Function implementing the myTask07 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Key_scan */
void Key_scan(void const * argument)
{
  /* USER CODE BEGIN Key_scan */
  /* Infinite loop */
  for(;;)
  {
	//	key_state_scan();
    osDelay(30);
  }
  /* USER CODE END Key_scan */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
