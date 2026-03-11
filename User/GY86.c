#include "stm32f4xx.h"
#include "gpio.h"
#include "Delay.h"
#include "FreeRTOS.h"
#include "Queue.h"
#include "semphr.h"
#include "i2c.h"


/* ==================== I2C 通信超时与重试配置 ==================== */
/* 超时时间配置（基于设备特性和 FreeRTOS tick） */
#define I2C_TIMEOUT_FAST_DEVICE     pdMS_TO_TICKS(5)    // MPU6050/HMC5883L：正常<1ms，5ms 安全裕量
#define I2C_TIMEOUT_PRESSURE        pdMS_TO_TICKS(30)   // BMP180/MS5611：包含 ADC 转换时间（最大 25.5ms）
#define I2C_TIMEOUT_INIT            pdMS_TO_TICKS(50)   // 初始化阶段可稍长

/* 重试机制配置 */
#define I2C_MAX_RETRY_COUNT         3                   // 最大重试次数
/* ============================================================ */


#define MPU6050_DEVICE_ADRESS 	0x68		//MPU6050 设备地址
#define MPU6050_PWR_MGMT_1 		0x6B  	// 电源管理1寄存器
#define MPU6050_PWR_MGMT_2 		0x6C  	// 电源管理2寄存器
#define MPU6050_SMPLRT_DIV 		0x19		//配置陀螺仪输出速率的分频系数
#define MPU6050_CONFIG 				0x1A		//配置数字低通滤波器(DLPF)的带宽
#define MPU6050_GYRO_CONFIG 		0x1B		//设置陀螺仪的自检功能和量程
#define MPU6050_ACCEL_CONFIG 		0x1C		//设置加速度计的自检功能、量程和高速滤波器
#define MPU6050_INT_ENABLE 		0x38		//中断使能
#define MPU6050_WHO_AM_I 			0x75
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_ACCEL_XOUT_L 0x3C
#define MPU6050_REG_ACCEL_YOUT_H 0x3D
#define MPU6050_REG_ACCEL_YOUT_L 0x3E
#define MPU6050_REG_ACCEL_ZOUT_H 0x3F
#define MPU6050_REG_ACCEL_ZOUT_L 0x40
#define MPU6050_REG_TEMP_OUT_H   0x41
#define MPU6050_REG_TEMP_OUT_L   0x42
#define MPU6050_REG_GYRO_XOUT_H  0x43
#define MPU6050_REG_GYRO_XOUT_L  0x44
#define MPU6050_REG_GYRO_YOUT_H  0x45
#define MPU6050_REG_GYRO_YOUT_L  0x46
#define MPU6050_REG_GYRO_ZOUT_H  0x47
#define MPU6050_REG_GYRO_ZOUT_L  0x48

// MPU6050辅助I2C相关寄存器定义
#define MPU6050_USER_CTRL       	0x6A    	// 用户控制寄存器
#define MPU6050_INT_PIN_CFG     	0x37    	// INT引脚/旁路使能配置寄存器
#define MPU6050_I2C_MST_CTRL    	0x24    	// I2C主控模式控制寄存器
#define MPU6050_I2C_SLV0_ADDR   	0x25    	// I2C从机0地址寄存器
#define MPU6050_I2C_SLV0_REG    	0x26    	// I2C从机0寄存器地址
#define MPU6050_I2C_SLV0_CTRL   	0x27    	// I2C从机0控制寄存器
#define MPU6050_I2C_SLV0_DO     	0x63    	// I2C从机0写数据寄存器
#define MPU6050_EXT_SENS_DATA_00 0x49   	// 外部传感器数据寄存器00

#define HMC5883L_ADDRESS 			0x1E 			// HMC5883L的I2C地址（7位地址）
#define HMC5883L_REG_CONFA 		0x00 			// 配置寄存器A
#define HMC5883L_REG_CONFB 		0x01 			// 配置寄存器B
#define HMC5883L_REG_MODE 			0x02 			// 模式寄存器
#define HMC5883L_X_OUT_H 			0x03 	
#define HMC5883L_X_OUT_L 			0x04
#define HMC5883L_Z_OUT_H 			0x05
#define HMC5883L_Z_OUT_L 			0x06
#define HMC5883L_Y_OUT_H 			0x07
#define HMC5883L_Y_OUT_L 			0x08
#define HMC5883L_MODE_CONTINOUS 	0x00 			// 连续模式
#define HMC5883L_AVERAGING_8 		0x40 			// 8次平均值
#define HMC5883L_RATE_15 			0x03 			// 15Hz的数据输出率

#define MS5611_DEVICE_ADRESS 0X77
#define MS5611_CMD_RESET         0x1E    // 复位命令
/* PROM读取命令 - 工厂校准系数 */
#define MS5611_CMD_READ_PROM     0x10    // PROM读取起始地址
#define MS5611_PROM_C1           (MS5611_CMD_READ_PROM + 0)  // 压力灵敏度
#define MS5611_PROM_C2           (MS5611_CMD_READ_PROM + 2)  // 压力偏移
#define MS5611_PROM_C3           (MS5611_CMD_READ_PROM + 4)  // 温度压力系数
#define MS5611_PROM_C4           (MS5611_CMD_READ_PROM + 6)  // 温度偏移系数
#define MS5611_PROM_C5           (MS5611_CMD_READ_PROM + 8)  // 参考温度
#define MS5611_PROM_C6           (MS5611_CMD_READ_PROM + 10) // 温度灵敏度
// 气压转换（不同过采样率）
#define MS5611_CMD_ADC_READ      0x00    // ADC读取命令
#define MS5611_CMD_CONV_D1_256   0x40    // 气压转换，OSR=256
#define MS5611_CMD_CONV_D1_512   0x42    // 气压转换，OSR=512
#define MS5611_CMD_CONV_D1_1024  0x44    // 气压转换，OSR=1024
#define MS5611_CMD_CONV_D1_2048  0x46    // 气压转换，OSR=2048
#define MS5611_CMD_CONV_D1_4096  0x48    // 气压转换，OSR=4096
// 温度转换（不同过采样率）
#define MS5611_CMD_CONV_D2_256   0x50    // 温度转换，OSR=256
#define MS5611_CMD_CONV_D2_512   0x52    // 温度转换，OSR=512
#define MS5611_CMD_CONV_D2_1024  0x54    // 温度转换，OSR=1024
#define MS5611_CMD_CONV_D2_2048  0x56    // 温度转换，OSR=2048
#define MS5611_CMD_CONV_D2_4096  0x58    // 温度转换，OSR=4096


#define BMP180_DEVICE_ADDR       0x77

// 寄存器地址
#define BMP180_REG_ID          	0xD0	 	// 芯片 ID 寄存器
#define BMP180_REG_VERSION     	0xD1   	// 版本寄存器（只读）
#define BMP180_REG_CTRL_MEAS   	0xF4   	// 控制测量寄存器
#define BMP180_REG_ADC_OUT_MSB 	0xF6   	// ADC 输出 MSB（3字节：F6, F7, F8）

// 校准系数寄存器（PROM，从 0xAA 到 0xBF，共 11 个 16-bit 值）
#define BMP180_PROM_START_ADDR 	0xAA
#define BMP180_PROM_DATA_LEN   	22    	// 11 个 16-bit = 22 字节

// 测量命令（写入 CTRL_MEAS 寄存器）
#define BMP180_CMD_TEMP        	0x2E   	// 启动温度测量
#define BMP180_CMD_PRESSURE_ULTRA_LOW_POWER  	0x34  // OSS=0
#define BMP180_CMD_PRESSURE_STANDARD         	0x74  // OSS=1
#define BMP180_CMD_PRESSURE_HIGH_RES         	0xB4  // OSS=2
#define BMP180_CMD_PRESSURE_ULTRA_HIGH_RES   	0xF4  // OSS=3

// 芯片 ID 固定值
#define BMP180_CHIP_ID         	0x55

// 过采样设置（OSS: Oversampling Setting）
#define BMP180_OSS_ULTRA_LOW   	0   	// 转换时间 4.5ms, 精度 ～0.06 hPa
#define BMP180_OSS_STANDARD    	1   	// 转换时间 7.5ms, 精度 ～0.03 hPa
#define BMP180_OSS_HIGH        	2   	// 转换时间 13.5ms, 精度 ～0.015 hPa
#define BMP180_OSS_ULTRA_HIGH  	3   	// 转换时间 25.5ms, 精度 ～0.009 hPa

uint16_t c[7];
int16_t g_GYRO[3];
int16_t g_ACCEL[3];
int16_t g_MAG[3];
uint32_t g_pressure;
QueueHandle_t QueueGYROACCEL;
QueueHandle_t QueueMAG;
QueueHandle_t QueuePressure;

struct GYRO_ACCEL_Data {
	int16_t gyro[3];
	int16_t accel[3];
};

struct MAG_Data {
	int16_t mag[3];
};

struct Pressure_Data {
	uint32_t pressure;
};

/**
 * @brief 带重试机制的 I2C DMA 读取（同步等待完成）
 *        使用互斥锁 + 二值信号量双锁机制
 * @param hi2c: I2C 句柄
 * @param DevAddress: 设备地址
 * @param MemAddress: 寄存器地址
 * @param MemAddSize: 地址长度
 * @param pData: 数据缓冲区
 * @param Size: 数据长度
 * @param timeout: 单次超时时间
 * @param max_retry: 最大重试次数
 * @retval HAL_StatusTypeDef 状态
 */
static HAL_StatusTypeDef I2C_Mem_Read_DMA_WithRetry(
    I2C_HandleTypeDef *hi2c,
    uint16_t DevAddress,
    uint16_t MemAddress,
    uint16_t MemAddSize,
    uint8_t *pData,
    uint16_t Size,
    TickType_t timeout,
    uint8_t max_retry  // 新增：最大重试次数
) {
	HAL_StatusTypeDef status = HAL_ERROR;
	uint8_t retry_count = 0;

	while (retry_count < max_retry) {
		if (xSemaphoreTake(I2CMutex, timeout) == pdTRUE) {
			// 清理上一次遗留的完成信号，避免误判 DMA 已完成。
			(void)xSemaphoreTake(I2CDoneSemaphore, 0);

			status = HAL_I2C_Mem_Read_DMA(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size);
			if (status == HAL_OK) {
				if (xSemaphoreTake(I2CDoneSemaphore, timeout) == pdTRUE) {
					xSemaphoreGive(I2CMutex);
					return HAL_OK;
				}
			}

			xSemaphoreGive(I2CMutex);
			retry_count++;
		} else {
			retry_count++;
		}

		if (retry_count < max_retry) {
			vTaskDelay(pdMS_TO_TICKS(1));
		}
	}

	return HAL_ERROR;
}

/**
 * @brief 带重试机制的 I2C DMA 写入（同步等待完成）
 *        使用互斥锁 + 二值信号量双锁机制
 * @param hi2c: I2C 句柄
 * @param DevAddress: 设备地址
 * @param MemAddress: 寄存器地址
 * @param MemAddSize: 地址长度
 * @param pData: 数据缓冲区
 * @param Size: 数据长度（字节数）
 * @param max_retry: 最大重试次数
 * @retval HAL_StatusTypeDef 状态
 */
static HAL_StatusTypeDef I2C_Mem_Write_DMA_WithRetry(
	I2C_HandleTypeDef *hi2c,
	uint16_t DevAddress,
	uint16_t MemAddress,
	uint16_t MemAddSize,
	uint8_t *pData,
	uint16_t Size,
	TickType_t timeout,
	uint8_t max_retry  // 新增：最大重试次数
) {
	HAL_StatusTypeDef status = HAL_ERROR;
	uint8_t retry_count = 0;

	while (retry_count < max_retry) {
		if (xSemaphoreTake(I2CMutex, timeout) == pdTRUE) {
			(void)xSemaphoreTake(I2CDoneSemaphore, 0);

			status = HAL_I2C_Mem_Write_DMA(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size);
			if (status == HAL_OK) {
				if (xSemaphoreTake(I2CDoneSemaphore, timeout) == pdTRUE) {
					xSemaphoreGive(I2CMutex);
					return HAL_OK;
				}
			}

			xSemaphoreGive(I2CMutex);
			retry_count++;
		} else {
			retry_count++;
		}

		if (retry_count < max_retry) {
			vTaskDelay(pdMS_TO_TICKS(1));
		}
	}

	return HAL_ERROR;
}

/**
 * @brief 带重试机制的 I2C DMA 主机发送（同步等待完成）
 *        使用互斥锁 + 二值信号量双锁机制
 * @param hi2c: I2C 句柄
 * @param DevAddress: 设备地址
 * @param pData: 数据缓冲区
 * @param Size: 数据长度（字节数）
 * @param max_retry: 最大重试次数
 * @retval HAL_StatusTypeDef 状态
 */
static HAL_StatusTypeDef I2C_Master_Transmit_DMA_WithRetry(
	I2C_HandleTypeDef *hi2c,
	uint16_t DevAddress,
	uint8_t *pData,
	uint16_t Size,
	TickType_t timeout,
	uint8_t max_retry  // 新增：最大重试次数
) {
	HAL_StatusTypeDef status = HAL_ERROR;
	uint8_t retry_count = 0;

	while (retry_count < max_retry) {
		if (xSemaphoreTake(I2CMutex, timeout) == pdTRUE) {
			(void)xSemaphoreTake(I2CDoneSemaphore, 0);

			status = HAL_I2C_Master_Transmit_DMA(hi2c, DevAddress, pData, Size);
			if (status == HAL_OK) {
				if (xSemaphoreTake(I2CDoneSemaphore, timeout) == pdTRUE) {
					xSemaphoreGive(I2CMutex);
					return HAL_OK;
				}
			}

			xSemaphoreGive(I2CMutex);
			retry_count++;
		} else {
			retry_count++;
		}

		if (retry_count < max_retry) {
			vTaskDelay(pdMS_TO_TICKS(1));
		}
	}

	return HAL_ERROR;
}

/**
 * @brief 带重试机制的 I2C DMA 主机接收（同步等待完成）
 *        使用互斥锁 + 二值信号量双锁机制
 * @param hi2c: I2C 句柄
 * @param DevAddress: 设备地址
 * @param pData: 数据缓冲区
 * @param Size: 数据长度（字节数）
 * @param max_retry: 最大重试次数
 * @retval HAL_StatusTypeDef 状态
 */
static HAL_StatusTypeDef I2C_Master_Receive_DMA_WithRetry(
	I2C_HandleTypeDef *hi2c,
	uint16_t DevAddress,
	uint8_t *pData,
	uint16_t Size,
	TickType_t timeout,
	uint8_t max_retry  // 新增：最大重试次数
) {
	HAL_StatusTypeDef status = HAL_ERROR;
	uint8_t retry_count = 0;

	while (retry_count < max_retry) {
		if (xSemaphoreTake(I2CMutex, timeout) == pdTRUE) {
			(void)xSemaphoreTake(I2CDoneSemaphore, 0);

			status = HAL_I2C_Master_Receive_DMA(hi2c, DevAddress, pData, Size);
			if (status == HAL_OK) {
				if (xSemaphoreTake(I2CDoneSemaphore, timeout) == pdTRUE) {
					xSemaphoreGive(I2CMutex);
					return HAL_OK;
				}
			}

			xSemaphoreGive(I2CMutex);
			retry_count++;
		} else {
			retry_count++;
		}

		if (retry_count < max_retry) {
			vTaskDelay(pdMS_TO_TICKS(1));
		}
	}

	return HAL_ERROR;
}

/* iic 发送一个字节 */
void IIC_Write(uint8_t addr,uint8_t RegAddr,uint8_t data){
	uint8_t buffer = data;
	(void)I2C_Mem_Write_DMA_WithRetry(&hi2c1, addr << 1, RegAddr, I2C_MEMADD_SIZE_8BIT,
									  &buffer, 1, I2C_TIMEOUT_FAST_DEVICE, I2C_MAX_RETRY_COUNT);
}

void IIC_Write_MS5611(uint8_t addr, uint8_t cmd){
	uint8_t buf = cmd;
	(void)I2C_Master_Transmit_DMA_WithRetry(&hi2c1, addr << 1, &buf, 1, I2C_TIMEOUT_PRESSURE,
											I2C_MAX_RETRY_COUNT);
}

uint8_t IIC_Read(uint8_t addr,uint8_t RegAddr){
	uint8_t data = 0;
	if (I2C_Mem_Read_DMA_WithRetry(&hi2c1, addr << 1, RegAddr, I2C_MEMADD_SIZE_8BIT,
								   &data, 1, I2C_TIMEOUT_FAST_DEVICE, I2C_MAX_RETRY_COUNT) == HAL_OK) {
		return data;
    }

	return 0;
}

uint32_t MS5611_Read(uint8_t cmd){
	uint32_t data = 0;
	uint8_t buffer[3] = {0};
	uint8_t cmd_buf = cmd;

	if (I2C_Master_Transmit_DMA_WithRetry(&hi2c1, MS5611_DEVICE_ADRESS << 1,
										  &cmd_buf, 1, I2C_TIMEOUT_PRESSURE,
										  I2C_MAX_RETRY_COUNT) != HAL_OK) {
		return 0;
	}

	if (I2C_Master_Receive_DMA_WithRetry(&hi2c1, MS5611_DEVICE_ADRESS << 1,
										 buffer, 3, I2C_TIMEOUT_PRESSURE,
										 I2C_MAX_RETRY_COUNT) != HAL_OK) {
		return 0;
	}

	data = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
	return data;
}

/* 通过 MPU6050 辅助 I2C 读取 HMC5883L 三轴磁力计数据 */
void HMC5883L_Read_MAG(int16_t *arr){
   uint8_t data[6];

   // HMC5883L 是低频传感器（50Hz），使用标准重试次数
   if (I2C_Mem_Read_DMA_WithRetry(&hi2c1, MPU6050_DEVICE_ADRESS << 1,
								  MPU6050_EXT_SENS_DATA_00, I2C_MEMADD_SIZE_8BIT,
								  data, 6, I2C_TIMEOUT_FAST_DEVICE, I2C_MAX_RETRY_COUNT) == HAL_OK) {
		arr[0] = (data[0] << 8) | data[1];	//X 轴
		arr[2] = (data[2] << 8) | data[3];	//Z 轴
		arr[1] = (data[4] << 8) | data[5];	//Y 轴
   }
}

/* 通过 MPU6050 辅助 I2C 写入 HMC5883L 寄存器 */
void HMC5883L_WriteReg(uint8_t reg, uint8_t data) {
	// 设置从机 0 写 HMC5883L 的寄存器地址
	IIC_Write(MPU6050_DEVICE_ADRESS, MPU6050_I2C_SLV0_ADDR, (uint8_t)(HMC5883L_ADDRESS << 1));
    vTaskDelay(pdMS_TO_TICKS(1));
	 
    // 设置要写的寄存器地址
	IIC_Write(MPU6050_DEVICE_ADRESS, MPU6050_I2C_SLV0_REG, reg);
    vTaskDelay(pdMS_TO_TICKS(1));
	 
    // 设置要写入的数据
	IIC_Write(MPU6050_DEVICE_ADRESS, MPU6050_I2C_SLV0_DO, data);
    vTaskDelay(pdMS_TO_TICKS(1));
	 
    // 启用传输 1 字节
	IIC_Write(MPU6050_DEVICE_ADRESS, MPU6050_I2C_SLV0_CTRL, 0x81);
	 vTaskDelay(pdMS_TO_TICKS(1));
}

/* 通过 MPU6050 辅助 I2C 设置 HMC5883L 自动读取 */
void HMC5883L_SetupAutoRead(void) {
	// 配置从机 0 读取 HMC5883L 的数据
	IIC_Write(MPU6050_DEVICE_ADRESS, MPU6050_I2C_SLV0_ADDR, (uint8_t)(HMC5883L_ADDRESS | 0x80));
	 vTaskDelay(pdMS_TO_TICKS(1));

	IIC_Write(MPU6050_DEVICE_ADRESS, MPU6050_I2C_SLV0_REG, HMC5883L_X_OUT_H);
	 vTaskDelay(pdMS_TO_TICKS(1));

	IIC_Write(MPU6050_DEVICE_ADRESS, MPU6050_I2C_SLV0_CTRL, 0x86);  // 启用并传输 6 字节
	 vTaskDelay(pdMS_TO_TICKS(1));
}

void MPU6050_Init(){
	IIC_Write(MPU6050_DEVICE_ADRESS, MPU6050_PWR_MGMT_1, 0x01);
	IIC_Write(MPU6050_DEVICE_ADRESS, MPU6050_PWR_MGMT_2, 0x00);
	IIC_Write(MPU6050_DEVICE_ADRESS, MPU6050_SMPLRT_DIV, 0x01);
	IIC_Write(MPU6050_DEVICE_ADRESS, MPU6050_CONFIG, 0x02);
	IIC_Write(MPU6050_DEVICE_ADRESS, MPU6050_GYRO_CONFIG, 0x18);   // 设置陀螺仪量程为 2000
	IIC_Write(MPU6050_DEVICE_ADRESS, MPU6050_ACCEL_CONFIG, 0x00);  // 设置加速度计量程为 2
	IIC_Write(MPU6050_DEVICE_ADRESS, MPU6050_INT_ENABLE, 0x00);
}

void MPU6050_Read_GYRO(int16_t *arr){
	uint8_t data[6];

	// MPU6050 是高频传感器（500Hz），数据更新快，只需重试 1 次
	if (I2C_Mem_Read_DMA_WithRetry(&hi2c1, MPU6050_DEVICE_ADRESS << 1,
			MPU6050_REG_GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, data, 6,
			I2C_TIMEOUT_FAST_DEVICE, 1) == HAL_OK) {
		arr[0] = (data[0] << 8) | data[1];
		arr[1] = (data[2] << 8) | data[3];
		arr[2] = (data[4] << 8) | data[5];
	}
}

void MPU6050_Read_ACCEL(int16_t *arr){
	uint8_t data[6];

	// MPU6050 是高频传感器（500Hz），数据更新快，只需重试 1 次
	if (I2C_Mem_Read_DMA_WithRetry(&hi2c1, MPU6050_DEVICE_ADRESS << 1,
			MPU6050_REG_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, data, 6,
			I2C_TIMEOUT_FAST_DEVICE, 1) == HAL_OK) {
		arr[0] = (data[0] << 8) | data[1];
		arr[1] = (data[2] << 8) | data[3];
		arr[2] = (data[4] << 8) | data[5];
	}
}

uint8_t MPU6050_Check(){
	uint8_t id = 0;
	// ID 检查只需尝试 1 次
	(void)I2C_Mem_Read_DMA_WithRetry(&hi2c1, MPU6050_DEVICE_ADRESS << 1, MPU6050_WHO_AM_I,
									 I2C_MEMADD_SIZE_8BIT, &id, 1, I2C_TIMEOUT_FAST_DEVICE, 1);

    return (id == 0x68) ? 1 : 0;
}

// 检查 HMC5883L 的 ID（旁路模式下直接读取）
uint8_t HMC5883L_CheckID() {
    static uint8_t id_a, id_b, id_c;

	// 启用旁路模式以便直接读取 HMC5883L
	IIC_Write(MPU6050_DEVICE_ADRESS, MPU6050_INT_PIN_CFG, 0x02);
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // 直接读取 HMC5883L 的 ID 寄存器
    id_a = IIC_Read(HMC5883L_ADDRESS, 0x0A);
    vTaskDelay(pdMS_TO_TICKS(1));
    id_b = IIC_Read(HMC5883L_ADDRESS, 0x0B);
    vTaskDelay(pdMS_TO_TICKS(1));
    id_c = IIC_Read(HMC5883L_ADDRESS, 0x0C);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    
 
    
    // HMC5883L 的 ID 应该是 0x48('H') 0x34('4') 0x33('3')
    return (id_a == 0x48 && id_b == 0x34 && id_c == 0x33);
}

void HMC5883L_Init(){
	// 按照手册步骤初始化 HMC5883L
	
	// 第一步：确保硬件连接正确（已在 PCB 上完成）
	
	// 第二步：配置外部模块（通过旁路模式）
	// 启用旁路模式，让 MCU 可以直接与 HMC5883L 通信
	// 重置 I2C 主控
	IIC_Write(MPU6050_DEVICE_ADRESS, MPU6050_USER_CTRL, 0x80);
	vTaskDelay(pdMS_TO_TICKS(1));
	
	// 启用旁路模式
	IIC_Write(MPU6050_DEVICE_ADRESS, MPU6050_INT_PIN_CFG, 0x02);
	vTaskDelay(pdMS_TO_TICKS(1));
	
	
	
	// 直接配置 HMC5883L（旁路模式下 MCU 可以直接与其通信）
	// 配置 HMC5883L 寄存器
	IIC_Write(HMC5883L_ADDRESS, HMC5883L_REG_CONFA, 0x78);  // 8-sample avg, 15Hz data rate
	vTaskDelay(pdMS_TO_TICKS(1));
	IIC_Write(HMC5883L_ADDRESS, HMC5883L_REG_CONFB, 0x60);  // 量程
	vTaskDelay(pdMS_TO_TICKS(1));
	IIC_Write(HMC5883L_ADDRESS, HMC5883L_REG_MODE, 0x00);   // 开启连续测量模式
	vTaskDelay(pdMS_TO_TICKS(1));
	// 第三步：配置 MPU6050 的 I²C 主控制器
	
	// 禁用旁路模式
	IIC_Write(MPU6050_DEVICE_ADRESS, MPU6050_INT_PIN_CFG, 0x00);
	vTaskDelay(pdMS_TO_TICKS(1));
	
	// 禁用旁路模式，启用 I2C 主控模式
	IIC_Write(MPU6050_DEVICE_ADRESS, MPU6050_USER_CTRL, 0x20);
	vTaskDelay(pdMS_TO_TICKS(3));
	
	// 配置 I2C 主控时钟和延迟
	vTaskDelay(pdMS_TO_TICKS(1));
	
	// 配置从机 0 自动读取 HMC5883L 的数据
	HMC5883L_SetupAutoRead();
	vTaskDelay(pdMS_TO_TICKS(1));
}

void MS5611_Init(){
	IIC_Write_MS5611(MS5611_DEVICE_ADRESS, MS5611_CMD_RESET);
   vTaskDelay(pdMS_TO_TICKS(2)); // 复位后至少等待 2ms（手册建议）
	
	c[1]=37856; 
	c[2]=34500; 
	c[3]=400; 
	c[4]=410; 
	c[5]=24205; 
	c[6]=33958;
	/*
	if (xSemaphoreTake(I2CMutex, pdMS_TO_TICKS(30)) == pdTRUE){
		cmd_buf2 = 0xA0;		//PROM 读取启动命令
		HAL_I2C_Master_Transmit_DMA(&hi2c1, MS5611_DEVICE_ADRESS<<1, 
			&cmd_buf2, sizeof(uint8_t));
		
		if (xSemaphoreTake(I2CDoneSemaphore, pdMS_TO_TICKS(30)) == pdTRUE){
			xSemaphoreGive(I2CMutex);
		} else {
			xSemaphoreGive(I2CMutex);
		}
	}
	
	// 读取校准系数
	uint8_t buffer[2];
	for (uint8_t i = 0; i < 6; i++) {
		 cmd_buf = MS5611_CMD_READ_PROM + (i * 2);
		
		if (xSemaphoreTake(I2CMutex, pdMS_TO_TICKS(30)) == pdTRUE){
			HAL_I2C_Master_Transmit_DMA(&hi2c1, MS5611_DEVICE_ADRESS<<1, &cmd_buf, 1);
			
			if (xSemaphoreTake(I2CDoneSemaphore, pdMS_TO_TICKS(30)) == pdTRUE){
				xSemaphoreGive(I2CMutex);
			} else {
				xSemaphoreGive(I2CMutex);
			}
		}
		vTaskDelay(pdMS_TO_TICKS(1)); 			// 必须延时 1ms，等待传感器内部读取完成！
		if (xSemaphoreTake(I2CMutex, pdMS_TO_TICKS(30)) == pdTRUE){
			HAL_I2C_Master_Receive_DMA(&hi2c1, MS5611_DEVICE_ADRESS<<1, buffer, 2);
			
			if (xSemaphoreTake(I2CDoneSemaphore, pdMS_TO_TICKS(5)) == pdTRUE){
				c[i+1] = (buffer[0] << 8) | buffer[1];
				xSemaphoreGive(I2CMutex);
			} else {
				xSemaphoreGive(I2CMutex);
			}
		}
	}*/
	
}

// 读取MS5611气压值
uint32_t MS5611_Read_Pressure() {
	uint32_t pressure;
	uint32_t D1;
	int32_t dT, TEMP;
	int64_t OFF, SENS;
	
	// 发送压力转换命令，使用OSR=1024获得更高精度
	IIC_Write_MS5611(MS5611_DEVICE_ADRESS, MS5611_CMD_CONV_D1_1024);
	vTaskDelay(pdMS_TO_TICKS(4)); // 等待转换完成
	//Delay_ms(3);
	
	// 读取ADC值
	D1 = MS5611_Read(MS5611_CMD_ADC_READ);
	
	
	// 发送温度转换命令
	IIC_Write_MS5611(MS5611_DEVICE_ADRESS, MS5611_CMD_CONV_D2_1024);
	vTaskDelay(pdMS_TO_TICKS(4)); // 等待转换完成
	//Delay_ms(3);
	
	// 读取温度ADC值
	uint32_t D2 = MS5611_Read(MS5611_CMD_ADC_READ);
	
	// 计算温度和压力
	// 根据MS5611数据手册中的公式进行计算
	dT = D2 - ((uint32_t)c[5] << 8);
	TEMP = 2000 + (((int64_t)dT * c[6]) >> 23);
	OFF = ((int64_t)c[2] << 16) + (((int64_t)c[4] * dT) >> 7);
	SENS = ((int64_t)c[1] << 15) + (((int64_t)c[3] * dT) >> 8);
	
	// 温度补偿
	if (TEMP < 2000) { // 低于20°C时进行二次温度补偿
		int32_t T2, OFF2, SENS2;
		T2 = (dT * dT) >> 31;
		OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
		SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
		
		if (TEMP < -1500) { // 低于-15°C时进行额外补偿
			OFF2 += 7 * ((TEMP + 1500) * (TEMP + 1500));
			SENS2 += 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
		}
		
		TEMP -= T2;
		OFF -= OFF2;
		SENS -= SENS2;
	}
	
	pressure = ((D1 * SENS) / 2097152 - OFF) / 32768;
	return pressure;
}

int16_t ac1, ac2, ac3;
uint16_t ac4, ac5, ac6;
int16_t b1, b2, mb, mc, md;

void BMP180_Init()
{
	vTaskDelay(pdMS_TO_TICKS(11));
	uint8_t prom[22];
	//读取校准系数（初始化时使用标准重试次数）
	if (I2C_Mem_Read_DMA_WithRetry(&hi2c1, BMP180_DEVICE_ADDR << 1, BMP180_PROM_START_ADDR,
			I2C_MEMADD_SIZE_8BIT, prom, 22, I2C_TIMEOUT_INIT, I2C_MAX_RETRY_COUNT) == HAL_OK) {
		ac1 = (int16_t)((prom[0] << 8) | prom[1]);
		ac2 = (int16_t)((prom[2] << 8) | prom[3]);
		ac3 = (int16_t)((prom[4] << 8) | prom[5]);
		ac4 = (uint16_t)((prom[6] << 8) | prom[7]);  // 无符号！
		ac5 = (uint16_t)((prom[8] << 8) | prom[9]);  // 无符号！
		ac6 = (uint16_t)((prom[10]<< 8) | prom[11]); // 无符号！
		b1  = (int16_t)((prom[12]<< 8) | prom[13]);
		b2  = (int16_t)((prom[14]<< 8) | prom[15]);
		mb  = (int16_t)((prom[16]<< 8) | prom[17]);
		mc  = (int16_t)((prom[18]<< 8) | prom[19]);
		md  = (int16_t)((prom[20]<< 8) | prom[21]);
	}
}

// 读取未补偿温度值
uint16_t bmp180_read_ut()
{
   uint8_t temp_data[2];
	uint16_t ut = 0;
    
    // 发送温度测量命令
    IIC_Write(BMP180_DEVICE_ADDR, BMP180_REG_CTRL_MEAS, BMP180_CMD_TEMP);
    
    // 等待转换完成 (最大 4.5ms)
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // 读取温度数据（气压计使用标准重试次数）
	if (I2C_Mem_Read_DMA_WithRetry(&hi2c1, BMP180_DEVICE_ADDR << 1, BMP180_REG_ADC_OUT_MSB,
								   I2C_MEMADD_SIZE_8BIT, temp_data, 2,
								   I2C_TIMEOUT_FAST_DEVICE, I2C_MAX_RETRY_COUNT) == HAL_OK) {
		ut = (temp_data[0] << 8) | temp_data[1];
    }
	 return ut;
}

// 读取未补偿压力值
uint32_t bmp180_read_up()
{
    uint8_t press_data[3];
	uint32_t raw_press = 0;
    
    // 发送压力测量命令，使用标准分辨率
    IIC_Write(BMP180_DEVICE_ADDR, BMP180_REG_CTRL_MEAS, 
              BMP180_CMD_PRESSURE_STANDARD + (BMP180_OSS_ULTRA_LOW << 6));
    
    // 等待转换完成 (最大 7.5ms)
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 读取压力数据（气压计使用标准重试次数）
	if (I2C_Mem_Read_DMA_WithRetry(&hi2c1, BMP180_DEVICE_ADDR << 1, BMP180_REG_ADC_OUT_MSB,
			I2C_MEMADD_SIZE_8BIT, press_data, 3, I2C_TIMEOUT_FAST_DEVICE, I2C_MAX_RETRY_COUNT) == HAL_OK) {
		raw_press = ((uint32_t)press_data[0] << 16) |
						((uint32_t)press_data[1] << 8) |
						(uint32_t)press_data[2];
	}

		 
    // 右移 (8-OSS) 位以获取最终结果
    raw_press >>= (8 - BMP180_OSS_ULTRA_LOW);
    
    return raw_press;
}

uint32_t BMP180_read_pressure(void) {
	 uint32_t pressure;
    uint16_t ut = bmp180_read_ut(); // 读取未补偿温度
    uint32_t up = bmp180_read_up(); // 读取未补偿压力
	//printf("%d\r\n%d\r\n\r\n", ut, up);

    // 温度补偿计算
    int32_t x1 = (((int32_t)ut - (int32_t)ac6) * (int32_t)ac5) >> 15;
    int32_t x2 = ((int32_t)mc << 11) / (x1 + md);
    int32_t b5 = x1 + x2;
    int32_t temperature = ((b5 + 8) >> 4); // 实际温度值，单位0.1摄氏度

    // 气压补偿计算
    int32_t b6 = b5 - 4000;
    x1 = (b2 * (b6 * b6 >> 12)) >> 11;
    x2 = (ac2 * b6) >> 11;
    int32_t x3 = x1 + x2;
    int32_t b3 = ((((int32_t)ac1 * 4 + x3) << BMP180_OSS_ULTRA_LOW) + 2) >> 2;
    x1 = (ac3 * b6) >> 13;
    x2 = (b1 * (b6 * b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    uint32_t b4 = (ac4 * (uint32_t)(x3 + 32768)) >> 15;
    uint32_t b7 = ((uint32_t)up - b3) * (50000 >> BMP180_OSS_ULTRA_LOW);
    int32_t p;
    if (b7 < 0x80000000) {
        p = (b7 << 1) / b4;
    } else {
        p = (b7 / b4) << 1;
    }
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    pressure = p + ((x1 + x2 + 3791) >> 4); // Pa

    return pressure;
}

void Get_GYROandACCEL()
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t xPeriod = pdMS_TO_TICKS(2);
	MPU6050_Init();
	struct GYRO_ACCEL_Data idata;

	while(1)
	{
		MPU6050_Read_GYRO(idata.gyro);
		MPU6050_Read_ACCEL(idata.accel);
		xQueueOverwrite(QueueGYROACCEL, &idata);
		vTaskDelayUntil(&xLastWakeTime, xPeriod);		//固定延时到上一次读取结束的2ms以后
	}
}

void Get_Pressure()
{	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t xPeriod = pdMS_TO_TICKS(40);
	//MS5611_Init();
	BMP180_Init();
	struct Pressure_Data idata;
	while(1)
	{
		//idata.pressure = MS5611_Read_Pressure();
		idata.pressure = BMP180_read_pressure();
		xQueueOverwrite(QueuePressure, &idata);
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
	}
}

void Get_MAG()
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t xPeriod = pdMS_TO_TICKS(20);
	HMC5883L_Init();
	struct MAG_Data idata;

	while(1)
	{
		HMC5883L_Read_MAG(idata.mag);		
		xQueueOverwrite(QueueMAG, &idata);
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
	}
}