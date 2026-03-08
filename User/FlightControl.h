#ifndef FLIGHT_CONTROL_H_
#define FLIGHT_CONTROL_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    FLIGHT_STATE_DISARMED = 0, // 未解锁，电机安全关闭，禁止飞行
    FLIGHT_STATE_ARMED,         // 已解锁，允许飞行，电机可输出
    FLIGHT_STATE_FAILSAFE       // 失控保护，进入安全模式（如信号丢失或传感器异常）
} FlightState_t;

/**
 * @brief 飞行调试数据结构体
 */
typedef struct
{
	FlightState_t state;
	uint8_t linkAlive;
	uint8_t sensorCalibrated;

	float roll;
	float pitch;
	float yaw;

	float rollSp;			/* 横滚角设定点（目标值） */
	float pitchSp;			/* 俯仰角设定点（目标值） */
	float yawRateSp;		/* 角速度设定点（目标值） */
	
	float rollOut;			/* 横滚通道最终PID输出（用于电机混控） */
	float pitchOut;			/* 俯仰通道最终PID输出（用于电机混控） */
	float yawOut;			/* 偏航通道最终PID输出（用于电机混控） */

	uint16_t throttle;		/* 基础油门指令 */
	uint16_t m1;			/* 电机1输出 */
	uint16_t m2;			/* 电机2输出 */
	uint16_t m3;			/* 电机3输出 */
	uint16_t m4;			/* 电机4输出 */
} FlightDebugData_t;

typedef enum
{
	FLIGHT_PID_ROLL_ANGLE = 0,
	FLIGHT_PID_PITCH_ANGLE,
	FLIGHT_PID_ROLL_RATE,
	FLIGHT_PID_PITCH_RATE,
	FLIGHT_PID_YAW_RATE,
	FLIGHT_PID_ALTITUDE,
	FLIGHT_PID_COUNT
} FlightPidId_t;

typedef enum
{
	FLIGHT_GAIN_KP = 0,
	FLIGHT_GAIN_KI,
	FLIGHT_GAIN_KD
} FlightGainType_t;

void FlightControl_Task(void* params);
void FlightControl_Init(void);
void FlightControl_GetDebugSnapshot(FlightDebugData_t* out);
bool FlightControl_SetPidGain(FlightPidId_t pidId, FlightGainType_t gainType, float value);
bool FlightControl_GetPidGain(FlightPidId_t pidId, FlightGainType_t gainType, float* outValue);

#endif /* FLIGHT_CONTROL_H_ */
