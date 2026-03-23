#include "FreeRTOS.h"
#include "queue.h"
#include "GY86.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "semphr.h"
#include "tim.h"
#include "i2c.h"
#include "getPPM_task.h"

#define YES_FIRST	1
#define NO_FIRST 	0
#define PPM_SYNC_GAP_US 4000u

static uint32_t g_last_capture = 0;
static uint32_t g_now_capture;
static uint16_t g_ppmCh[PPM_CHANNEL_COUNT];
static uint8_t g_ppmChi = 0;
static uint8_t g_first = YES_FIRST;
static uint8_t g_test = 0;
QueueHandle_t QueuePPM;
BaseType_t xHigherPriorityTaskWoken = pdFALSE;

static struct PPM_Data idata;

void getPPM_Task(){
	// 确保定时器时钟已经使能
	__HAL_RCC_TIM2_CLK_ENABLE();
	
	// 启动输入捕获中断
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);  // 启动通道2输入捕获中断
	
	// 添加调试信息
	printf("PPM Task Started!\r\n");
	
	// 任务主体可以为空或者添加其他逻辑
	for(;;) {
		vTaskDelay(1000); // 避免任务过快循环
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	xHigherPriorityTaskWoken = pdFALSE;
	if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
		uint32_t pulseWidth;

		g_now_capture = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
		pulseWidth = g_now_capture - g_last_capture;

		if (pulseWidth > PPM_SYNC_GAP_US)
		{
			g_last_capture = g_now_capture;
			g_ppmChi = 0;
			return;
		}

		if (g_ppmChi < PPM_CHANNEL_COUNT)
		{
			idata.ppmCh[g_ppmChi] = (uint16_t)pulseWidth;
			g_ppmChi++;

			if (g_ppmChi == PPM_CHANNEL_COUNT)
			{
				xQueueSendFromISR(QueuePPM, &idata, &xHigherPriorityTaskWoken);
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
				g_ppmChi = 0;
			}
		}
		else
		{
			/* 异常情况下丢弃当前帧，避免通道索引越界。 */
			g_ppmChi = 0;
		}

		g_last_capture = g_now_capture;
	}
}