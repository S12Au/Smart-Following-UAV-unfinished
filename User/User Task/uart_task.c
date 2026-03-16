#include "FreeRTOS.h"
#include "queue.h"
#include "GY86.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "semphr.h"
#include "i2c.h"
#include "getPPM_task.h"
#include "FlightControl.h"
#include "autoconf.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#define UART1_RX_RING_SIZE 256u

static volatile uint8_t s_uart1RxRing[UART1_RX_RING_SIZE];
static volatile uint16_t s_uart1RxHead = 0;
static volatile uint16_t s_uart1RxTail = 0;
static volatile uint8_t s_uart1RxOverflow = 0;
static uint8_t s_uart1RxByte = 0;

static void uart1RxStartIT(void)
{
	if (HAL_UART_Receive_IT(&huart1, &s_uart1RxByte, 1) != HAL_OK)
	{
		/* 下次任务循环会继续重试启动中断接收 */
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	if (huart->Instance == USART1)
	{
		uint16_t nextHead = (uint16_t)((s_uart1RxHead + 1u) % UART1_RX_RING_SIZE);

		if (nextHead != s_uart1RxTail)
		{
			s_uart1RxRing[s_uart1RxHead] = s_uart1RxByte;
			s_uart1RxHead = nextHead;
		}
		else
		{
			s_uart1RxOverflow = 1u;
		}

		uart1RxStartIT();
	}
}

/**
 * @brief 比较两个字符串是否相等，比较时忽略大小写
 * @param a 字符串 a
 * @param b 字符串 b
 * @return 1 表示相等，0 表示不相等
 */
static int tokenEquals(const char* a, const char* b)
{
	if (a == 0 || b == 0)
	{
		return 0;
	}

	while (*a && *b)
	{
		char ca = *a;
		char cb = *b;

		if (ca >= 'A' && ca <= 'Z')
		{
			ca = (char)(ca - 'A' + 'a');
		}
		if (cb >= 'A' && cb <= 'Z')
		{
			cb = (char)(cb - 'A' + 'a');
		}

		if (ca != cb)
		{
			return 0;
		}

		a++;
		b++;
	}

	return (*a == '\0' && *b == '\0');
}

static int parsePidId(const char* s, FlightPidId_t* out)
{
	if (tokenEquals(s, "ra") || tokenEquals(s, "roll_angle"))
	{
		*out = FLIGHT_PID_ROLL_ANGLE;
		return 1;
	}
	if (tokenEquals(s, "pa") || tokenEquals(s, "pitch_angle"))
	{
		*out = FLIGHT_PID_PITCH_ANGLE;
		return 1;
	}
	if (tokenEquals(s, "rr") || tokenEquals(s, "roll_rate"))
	{
		*out = FLIGHT_PID_ROLL_RATE;
		return 1;
	}
	if (tokenEquals(s, "pr") || tokenEquals(s, "pitch_rate"))
	{
		*out = FLIGHT_PID_PITCH_RATE;
		return 1;
	}
	if (tokenEquals(s, "yr") || tokenEquals(s, "yaw_rate"))
	{
		*out = FLIGHT_PID_YAW_RATE;
		return 1;
	}
	if (tokenEquals(s, "alt") || tokenEquals(s, "altitude"))
	{
		*out = FLIGHT_PID_ALTITUDE;
		return 1;
	}

	return 0;
}

static int parseGainType(const char* s, FlightGainType_t* out)
{
	if (tokenEquals(s, "kp"))
	{
		*out = FLIGHT_GAIN_KP;
		return 1;
	}
	if (tokenEquals(s, "ki"))
	{
		*out = FLIGHT_GAIN_KI;
		return 1;
	}
	if (tokenEquals(s, "kd"))
	{
		*out = FLIGHT_GAIN_KD;
		return 1;
	}

	return 0;
}

static const char* pidName(FlightPidId_t id)
{
	switch (id)
	{
		case FLIGHT_PID_ROLL_ANGLE: return "roll_angle";
		case FLIGHT_PID_PITCH_ANGLE: return "pitch_angle";
		case FLIGHT_PID_ROLL_RATE: return "roll_rate";
		case FLIGHT_PID_PITCH_RATE: return "pitch_rate";
		case FLIGHT_PID_YAW_RATE: return "yaw_rate";
		case FLIGHT_PID_ALTITUDE: return "altitude";
		default: return "unknown";
	}
}

static void printPidAll(void)
{
	FlightPidId_t ids[] = {
		FLIGHT_PID_ROLL_ANGLE,
		FLIGHT_PID_PITCH_ANGLE,
		FLIGHT_PID_ROLL_RATE,
		FLIGHT_PID_PITCH_RATE,
		FLIGHT_PID_YAW_RATE,
		FLIGHT_PID_ALTITUDE
	};
	uint8_t i;

	printf("PID,BEGIN\r\n");
	for (i = 0; i < (uint8_t)(sizeof(ids) / sizeof(ids[0])); i++)
	{
		float kp = 0.0f;
		float ki = 0.0f;
		float kd = 0.0f;
		FlightControl_GetPidGain(ids[i], FLIGHT_GAIN_KP, &kp);
		FlightControl_GetPidGain(ids[i], FLIGHT_GAIN_KI, &ki);
		FlightControl_GetPidGain(ids[i], FLIGHT_GAIN_KD, &kd);
		printf("PID,%s,KP=%.5f,KI=%.5f,KD=%.5f\r\n", pidName(ids[i]), kp, ki, kd);
	}
	printf("PID,END\r\n");
}

static void handleUartCommand(char* line)
{
	char* cmd;
	char* arg1;
	char* arg2;
	char* arg3;
	char* endPtr;

	cmd = strtok(line, " \t");
	if (cmd == 0)
	{
		return;
	}

	if (!tokenEquals(cmd, "pid"))
	{
		return;
	}

	arg1 = strtok(0, " \t");
	if (arg1 == 0)
	{
		printf("PID,ERR,usage: pid show | pid set <ra|pa|rr|pr|yr|alt> <kp|ki|kd> <value>\r\n");
		return;
	}

	if (tokenEquals(arg1, "show"))
	{
		printPidAll();
		return;
	}

	if (tokenEquals(arg1, "set"))
	{
		FlightPidId_t pidId;
		FlightGainType_t gainType;
		float value;

		arg2 = strtok(0, " \t");
		arg3 = strtok(0, " \t");
		char* arg4 = strtok(0, " \t");

		if (arg2 == 0 || arg3 == 0 || arg4 == 0)
		{
			printf("PID,ERR,usage: pid set <ra|pa|rr|pr|yr|alt> <kp|ki|kd> <value>\r\n");
			return;
		}

		if (!parsePidId(arg2, &pidId))
		{
			printf("PID,ERR,unknown pid id\r\n");
			return;
		}

		if (!parseGainType(arg3, &gainType))
		{
			printf("PID,ERR,unknown gain type\r\n");
			return;
		}

		value = strtof(arg4, &endPtr);
		if (endPtr == arg4 || *endPtr != '\0' || !isfinite(value))
		{
			printf("PID,ERR,invalid value\r\n");
			return;
		}

		if (!FlightControl_SetPidGain(pidId, gainType, value))
		{
			FlightPidSetError_t err = FlightControl_GetLastPidSetError();

			switch (err)
			{
				case FLIGHT_PID_SET_ERR_NOT_DISARMED:
					printf("PID,ERR,forbidden:not_disarmed\r\n");
					break;
				case FLIGHT_PID_SET_ERR_FLASH_WRITE:
					printf("PID,ERR,flash_write_failed\r\n");
					break;
				case FLIGHT_PID_SET_ERR_INVALID_INPUT:
					printf("PID,ERR,invalid_input\r\n");
					break;
				default:
					printf("PID,ERR,set failed\r\n");
					break;
			}
			return;
		}

		{
			float kp = 0.0f;
			float ki = 0.0f;
			float kd = 0.0f;
			FlightControl_GetPidGain(pidId, FLIGHT_GAIN_KP, &kp);
			FlightControl_GetPidGain(pidId, FLIGHT_GAIN_KI, &ki);
			FlightControl_GetPidGain(pidId, FLIGHT_GAIN_KD, &kd);
			printf("PID,OK,%s,KP=%.5f,KI=%.5f,KD=%.5f\r\n", pidName(pidId), kp, ki, kd);
		}

		return;
	}

	printf("PID,ERR,unknown subcmd\r\n");
}

/**
 * @brief 处理 UART 接收数据，解析命令并执行相应操作
 * @note 该函数会被主任务循环调用，以处理 UART 接口
 */
static void processUartRx(void)
{
	static char rxLine[96];
	static uint16_t rxLen = 0;
	uint8_t ch;
	uint8_t i;

	uart1RxStartIT();

	if (s_uart1RxOverflow != 0u)
	{
		s_uart1RxOverflow = 0u;
		rxLen = 0;
		printf("PID,ERR,uart_rx_overflow\r\n");
	}

	for (i = 0; i < 32; i++)
	{
		if (s_uart1RxTail == s_uart1RxHead)
		{
			break;
		}

		ch = s_uart1RxRing[s_uart1RxTail];
		s_uart1RxTail = (uint16_t)((s_uart1RxTail + 1u) % UART1_RX_RING_SIZE);

		if (ch == '\r' || ch == '\n')
		{
			if (rxLen > 0)
			{
				rxLine[rxLen] = '\0';
				handleUartCommand(rxLine);
				rxLen = 0;
			}
			continue;
		}

		if (rxLen < (sizeof(rxLine) - 1))
		{
			rxLine[rxLen++] = (char)ch;
		}
		else
		{
			rxLen = 0;
			printf("PID,ERR,line too long\r\n");
		}
	}
}

void Uart_Send_Task()
{	
	FlightDebugData_t dbg;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xPeriod = pdMS_TO_TICKS(1000 / CONFIG_UART_DEBUG_RATE_HZ);

	uart1RxStartIT();

	//printf("FC,st,lk,cal,imuFresh,imuDrop%%,thr,r,p,y,rSp,pSp,ySp,rOut,pOut,yOut,m1,m2,m3,m4\r\n");
	while(1)
	{
		//processUartRx();

		//FlightControl_GetDebugSnapshot(&dbg);
		/*
		printf("FC,%d,%u,%u,%u,%.2f,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%u,%u,%u\r\n",
				(int)dbg.state,
				dbg.linkAlive,
				dbg.sensorCalibrated,
				dbg.imuFreshness,
				dbg.imuDropRatePct,
				dbg.throttle,
				dbg.roll,
				dbg.pitch,
				dbg.yaw,
				dbg.rollSp,
				dbg.pitchSp,
				dbg.yawRateSp,
				dbg.rollOut,
				dbg.pitchOut,
				dbg.yawOut,
				dbg.m1,
				dbg.m2,
				dbg.m3,
				dbg.m4);*/

		
		//vTaskDelayUntil(&xLastWakeTime, xPeriod);
		
		printf("123我是\r\n");
		vTaskDelay(1000);
	}
	
	/*
	struct GYRO_ACCEL_Data GYRO_ACCEL_Rdata;
	struct MAG_Data MAG_Rdata;
	struct Pressure_Data Pressure_Rdata;
	struct PPM_Data PPM_Rdata;
	while(1)
	{
		
		printf("\r\n\r\n");
		
		xQueueReceive(QueueGYROACCEL,&GYRO_ACCEL_Rdata,portMAX_DELAY);
		for(uint8_t i=0; i<3; i++){
			printf("%f ", (float)GYRO_ACCEL_Rdata.gyro[i] / 16.4);
		}
		printf("\r\n");
		for(uint8_t i=0; i<3; i++){
			printf("%f ", (float)GYRO_ACCEL_Rdata.accel[i] / 16384.0f * 9.80665f);
		}
		printf("\r\n");
		
		xQueueReceive(QueueMAG, &MAG_Rdata, portMAX_DELAY);
		for(uint8_t i=0; i<3; i++){
			printf("%d ", MAG_Rdata.mag[i]);
		}
		printf("\r\n");
		
		xQueueReceive(QueuePressure, &Pressure_Rdata, portMAX_DELAY);
		printf("%d\r\n", Pressure_Rdata.pressure);
		
		
		xQueueReceive(QueuePPM, &PPM_Rdata, portMAX_DELAY);
		printf("%d", PPM_Rdata.ppmCh[0]);
		

		vTaskDelay(5);
	
	}*/
	
}