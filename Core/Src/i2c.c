/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
#include "FreeRTOS.h"
#include "Queue.h"
#include "semphr.h"
#include "usart.h"
#include "stdio.h"
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */
#define I2C_TX_DONE 0
#define I2C_TX_BUSY 1
#define I2C_RX_DONE 0
#define I2C_RX_BUSY 1

uint8_t i2c_txflag = I2C_TX_DONE;
uint8_t i2c_rxflag = I2C_RX_DONE;

/* ==================== I2C 双锁机制 ====================
 * I2CMutex: 互斥锁，保护 I2C 总线独占访问（带优先级继承）
 * I2CDoneSemaphore: 二值信号量，通知 DMA 传输完成（同步用）
 * ================================================ */
SemaphoreHandle_t I2CMutex = NULL;           // 互斥锁 - 用于总线互斥访问
SemaphoreHandle_t I2CDoneSemaphore = NULL;   // 二值信号量 - 用于 DMA 完成同步
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init */
	// 创建 I2C 双锁机制
	I2CMutex = xSemaphoreCreateMutex();
	I2CDoneSemaphore = xSemaphoreCreateBinary();
	
	if( (I2CMutex != NULL) && (I2CDoneSemaphore != NULL) ) {
    // 释放二值信号量，表示可以开始第一次 DMA 传输
		// 注意：这里释放是为了配合"先启动 DMA，再等待完成"的模式
		// 但实际上第一次传输不需要等待这个信号量（因为是初始状态）
		// 真正的同步信号量在 DMA 回调中释放
		xSemaphoreGive(I2CDoneSemaphore);
	}
	/* USER CODE END I2C1_Init */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* I2C1 DMA Init */
    /* I2C1_RX Init */
    hdma_i2c1_rx.Instance = DMA1_Stream0;
    hdma_i2c1_rx.Init.Channel = DMA_CHANNEL_1;
    hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_i2c1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_i2c1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(i2cHandle,hdmarx,hdma_i2c1_rx);

    /* I2C1_TX Init */
    hdma_i2c1_tx.Instance = DMA1_Stream6;
    hdma_i2c1_tx.Init.Channel = DMA_CHANNEL_1;
    hdma_i2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c1_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_i2c1_tx.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_i2c1_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
    hdma_i2c1_tx.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_i2c1_tx.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_i2c1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(i2cHandle,hdmatx,hdma_i2c1_tx);

    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);

    /* I2C1 DMA DeInit */
    HAL_DMA_DeInit(i2cHandle->hdmarx);
    HAL_DMA_DeInit(i2cHandle->hdmatx);

    /* I2C1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
 * @brief I2C DMA 传输完成回调函数集合
 * 所有回调函数统一释放 I2CDoneSemaphore（二值信号量）通知任务 DMA 已完成
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(I2CDoneSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(I2CDoneSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(I2CDoneSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(I2CDoneSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // 错误时也释放信号量，防止任务永久阻塞
        xSemaphoreGiveFromISR(I2CDoneSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// 添加 DMA 传输完成回调函数
void HAL_I2C_MasterTxDmaCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(I2CDoneSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_I2C_MasterRxDmaCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(I2CDoneSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_I2C_MemTxDmaCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(I2CDoneSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_I2C_MemRxDmaCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(I2CDoneSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
/* USER CODE END 1 */
