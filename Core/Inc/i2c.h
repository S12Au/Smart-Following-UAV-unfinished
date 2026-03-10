/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "freertos.h"
#include "semphr.h"
/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN Private defines */
#define I2C_TX_DONE 0
#define I2C_TX_BUSY 1
#define I2C_RX_DONE 0
#define I2C_RX_BUSY 1
/* USER CODE END Private defines */

void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */
extern uint8_t i2c_txflag;
extern uint8_t i2c_rxflag;

/* ==================== I2C 双锁机制声明 ====================
 * I2CMutex: 互斥锁，保护 I2C 总线独占访问（带优先级继承）
 * I2CDoneSemaphore: 二值信号量，通知 DMA 传输完成（同步用）
 * ================================================ */
extern SemaphoreHandle_t I2CMutex;           // 互斥锁 - 用于总线互斥访问
extern SemaphoreHandle_t I2CDoneSemaphore;   // 二值信号量 - 用于 DMA 完成同步
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

