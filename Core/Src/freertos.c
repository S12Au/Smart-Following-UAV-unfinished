/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "GY86.h"
#include "Delay.h"
#include "OLED.h"
#include "Queue.h"
#include "usart.h"
#include "semphr.h"
#include "i2c.h"
#include "uart_task.h"
#include "getPPM_task.h"
#include "flightcontrol_task.h"
#include <string.h>
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Get_GYROandACCEL_Task(void *params);
void Get_MAG_Task(void *params);
void Get_Pressure_Task(void *params);
void Show_Task(void *params);
void Uart_Debug_Task(void *params);
void Get_PPM_Task(void *params);
void Flight_Control_Task(void *params);
void K230_Control_Task(void *params);
void USART6_Echo_Test_Task(void *params);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  QueueGYROACCEL = xQueueCreate(1, sizeof(struct GYRO_ACCEL_Data));
  QueueMAG = xQueueCreate(1, sizeof(struct MAG_Data));
  QueuePressure = xQueueCreate(1, sizeof(struct Pressure_Data));
	QueuePPM = xQueueCreate(10, sizeof(struct PPM_Data));
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
	
	/* 传感器采集任务 - 按重要性分配优先级 */
	xTaskCreate(Get_GYROandACCEL_Task,"Get_GYROandACCEL_Task",128,NULL,osPriorityAboveNormal,NULL);	// 陀螺仪/加速度计：最高（500Hz，姿态控制依赖）
	xTaskCreate(Get_MAG_Task,"Get_MAG_Task",128,NULL,osPriorityNormal,NULL);							// 磁力计：中等（50Hz，航向参考）
	xTaskCreate(Get_Pressure_Task,"Get_Pressure_Task",128,NULL,osPriorityBelowNormal,NULL);			// 气压计：最低（25Hz，高度辅助）
	
	xTaskCreate(Uart_Debug_Task,"Uart_Debug_Task",128,NULL,osPriorityBelowNormal,NULL);
	xTaskCreate(Get_PPM_Task,"Get_PPM_Task",128,NULL,osPriorityNormal,NULL);
	
	xTaskCreate(Flight_Control_Task,"Flight_Control_Task",256,NULL,osPriorityHigh,NULL);				// 飞行控制：非常高（控制核心）
	
	//xTaskCreate(USART6_Echo_Test_Task,"USART6_Echo_Test_Task",128,NULL,osPriorityNormal,NULL);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Get_GYROandACCEL_Task(void *params)
{
	Get_GYROandACCEL();
}

void Get_MAG_Task(void *params)
{
	Get_MAG();
}

void Get_Pressure_Task(void *params)
{
	Get_Pressure();
}

void Show_Task(void *params)
{
	OLED_Show();
}

void Uart_Debug_Task(void *params)
{
	Uart_Send_Task();
}

void Get_PPM_Task(void *params)
{
	getPPM_Task();
}

void Flight_Control_Task(void *params)
{
  FlightControl_Task(params);
}

void K230_Control_Task(void *params)
{
	
}

void USART6_Echo_Test_Task(void *params)
{
  uint8_t rxBuffer[128];
  uint16_t rxLength = 0;
  const char *readyMsg = "USART6 echo test ready\r\n";

  HAL_UART_Transmit(&huart6, (uint8_t *)readyMsg, strlen(readyMsg), 1000);

  for (;;)
  {
    if (HAL_UARTEx_ReceiveToIdle(&huart6, rxBuffer, sizeof(rxBuffer), &rxLength, 100) == HAL_OK)
    {
      if (rxLength > 0)
      {
        HAL_UART_Transmit(&huart6, rxBuffer, rxLength, 1000);
      }
    }
    osDelay(1);
  }
}
/* USER CODE END Application */

