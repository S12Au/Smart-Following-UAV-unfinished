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

typedef struct
{
	FlightState_t state;
	uint8_t linkAlive;
	uint8_t sensorCalibrated;

	float roll;
	float pitch;
	float yaw;

	float rollSp;
	float pitchSp;
	float yawRateSp;

	float rollOut;
	float pitchOut;
	float yawOut;

	uint16_t throttle;
	uint16_t m1;
	uint16_t m2;
	uint16_t m3;
	uint16_t m4;
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
