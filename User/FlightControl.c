#include "FlightControl.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "GY86.h"
#include "getPPM_task.h"
#include "AttitudeEstimator.h"
#include "AttitudeControl.h"
#include "autoconf.h"
#include "pid.h"
#include "physicalConstants.h"
#include "tim.h"
#include "num.h"
#include <math.h>
#include <string.h>

#define FC_DT_MS    (1000 / CONFIG_CONTROLLER_RATE_HZ)
#define PID_FLASH_ADDR       0x08060000U
#define PID_FLASH_MAGIC      0x50494431U
#define PID_FLASH_VERSION    0x00010000U
#define RC_FILTER_ALPHA      0.20f

typedef struct
{
    uint32_t magic;
    uint32_t version;
    float gains[FLIGHT_PID_COUNT][3];
    uint32_t checksum;
} FlightPidPersist_t;

/* ========== 飞控核心模块实例 ========== */
static AttitudeEstimator_t g_estimator;    // Mahony姿态估计器（融合 IMU 数据解算姿态）
static AttitudeController_t g_controller;  // 双环 PID 姿态控制器（角度环 + 角速度环）
static PidObject g_altitudePid;            // 高度 PID 控制器（用于气压定高模式）
static volatile FlightDebugData_t g_debug; // 调试数据（通过串口/OLED 实时输出）
static FlightState_t g_flightState = FLIGHT_STATE_DISARMED;  // 当前飞行状态（解锁/锁定等）

/* ========== 高度控制参数 ========== */
static float g_altitudeSp = 0.0f;          // 目标设定点高度（米，定高模式使用）
static uint8_t g_altHoldActive = 0;        // 定高模式激活标志（1=激活，0=关闭）
static float g_baroRefPressurePa = SEA_LEVEL_PRESSURE;  // 参考气压值（起飞点气压，用于计算相对高度）
static uint8_t g_baroRefReady = 0;         // 参考气压就绪标志（1=已校准，0=未校准）

/* ========== IMU 零偏校准参数 ========== */
static int32_t g_gyroBiasRaw[3] = {0, 0, 0};  // 陀螺仪零偏校正值（LSB，三轴）
static int32_t g_accelBiasRaw[3] = {0, 0, 0}; // 加速度计零偏校正值（LSB，三轴）
static uint8_t g_biasReady = 0;            // 零偏校准就绪标志（1=校准完成，0=校准中）

/* ========== IMU 校准过程变量 ========== */
static int64_t g_gyroSum[3] = {0, 0, 0};   // 陀螺仪累加和（用于滑动平均滤波）
static int64_t g_accelSum[3] = {0, 0, 0};  // 加速度计累加和（用于滑动平均滤波）
static uint16_t g_calibCount = 0;          // 校准采样计数器

static uint32_t calcPersistChecksum(const FlightPidPersist_t* data)
{
    const uint8_t* bytes = (const uint8_t*)data;
    uint32_t sum = 0xA5A5A5A5U;
    uint32_t i;

    for (i = 0; i < (uint32_t)(sizeof(FlightPidPersist_t) - sizeof(uint32_t)); i++)
    {
        sum ^= bytes[i];
        sum = (sum << 5) | (sum >> 27);
        sum += 0x9E3779B9U;
    }

    return sum;
}

/** 
 * @brief 一阶低通滤波器（针对 uint16_t 输入）
 * @param prev 上一次的输出值
 * @param input 当前原始输入值
 * @param alpha 滤波系数（0.0f - 1.0f，值越大响应越快，但噪声也越大）
 * @return 处理后的输出值
 */
static uint16_t lowPassChannelU16(uint16_t prev, uint16_t input, float alpha)
{
    float y = (1.0f - alpha) * (float)prev + alpha * (float)input;
    y = constrain(y, (float)CONFIG_PPM_MIN_VALID, (float)CONFIG_PPM_MAX_VALID);
    return (uint16_t)(y + 0.5f);
}

static uint16_t invertPpmChannelU16(uint16_t input)
{
    uint16_t constrained = (uint16_t)constrain((float)input,
                                               (float)CONFIG_PPM_MIN_VALID,
                                               (float)CONFIG_PPM_MAX_VALID);
    return (uint16_t)(CONFIG_PPM_MIN_VALID + CONFIG_PPM_MAX_VALID - constrained);
}

static PidObject* getPidObjectById(FlightPidId_t pidId)
{
    switch (pidId)
    {
        case FLIGHT_PID_ROLL_ANGLE:
            return &g_controller.rollAnglePid;
        case FLIGHT_PID_PITCH_ANGLE:
            return &g_controller.pitchAnglePid;
        case FLIGHT_PID_ROLL_RATE:
            return &g_controller.rollRatePid;
        case FLIGHT_PID_PITCH_RATE:
            return &g_controller.pitchRatePid;
        case FLIGHT_PID_YAW_RATE:
            return &g_controller.yawRatePid;
        case FLIGHT_PID_ALTITUDE:
            return &g_altitudePid;
        default:
            return 0;
    }
}

static bool setPidGainInternal(FlightPidId_t pidId, FlightGainType_t gainType, float value)
{
    PidObject* pid = getPidObjectById(pidId);

    if (pid == 0)
    {
        return false;
    }

    if (value < 0.0f)
    {
        return false;
    }

    switch (gainType)
    {
        case FLIGHT_GAIN_KP:
            pidSetKp(pid, value);
            return true;
        case FLIGHT_GAIN_KI:
            pidSetKi(pid, value);
            return true;
        case FLIGHT_GAIN_KD:
            pidSetKd(pid, value);
            return true;
        default:
            return false;
    }
}

static bool savePidParamsToFlash(void)
{
    FlightPidPersist_t data;
    PidObject* pid;
    uint32_t wordAddress;
    uint32_t i;
    uint32_t sectorError = 0;
    HAL_StatusTypeDef halStatus;
    FLASH_EraseInitTypeDef erase;
    const uint32_t* words;

    memset(&data, 0, sizeof(data));
    data.magic = PID_FLASH_MAGIC;
    data.version = PID_FLASH_VERSION;

    for (i = 0; i < (uint32_t)FLIGHT_PID_COUNT; i++)
    {
        pid = getPidObjectById((FlightPidId_t)i);
        if (pid == 0)
        {
            return false;
        }
        data.gains[i][FLIGHT_GAIN_KP] = pid->kp;
        data.gains[i][FLIGHT_GAIN_KI] = pid->ki;
        data.gains[i][FLIGHT_GAIN_KD] = pid->kd;
    }

    data.checksum = calcPersistChecksum(&data);

    HAL_FLASH_Unlock();

    memset(&erase, 0, sizeof(erase));
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase.Sector = FLASH_SECTOR_7;
    erase.NbSectors = 1;

    halStatus = HAL_FLASHEx_Erase(&erase, &sectorError);
    if (halStatus != HAL_OK)
    {
        HAL_FLASH_Lock();
        return false;
    }

    words = (const uint32_t*)&data;
    wordAddress = PID_FLASH_ADDR;
    for (i = 0; i < (uint32_t)(sizeof(FlightPidPersist_t) / sizeof(uint32_t)); i++)
    {
        halStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, wordAddress, words[i]);
        if (halStatus != HAL_OK)
        {
            HAL_FLASH_Lock();
            return false;
        }
        wordAddress += 4U;
    }

    HAL_FLASH_Lock();
    return true;
}

static bool loadPidParamsFromFlash(void)
{
    const FlightPidPersist_t* data = (const FlightPidPersist_t*)PID_FLASH_ADDR;
    uint32_t i;

    if (data->magic != PID_FLASH_MAGIC || data->version != PID_FLASH_VERSION)
    {
        return false;
    }

    if (data->checksum != calcPersistChecksum(data))
    {
        return false;
    } 

    for (i = 0; i < (uint32_t)FLIGHT_PID_COUNT; i++)
    {
        if (!setPidGainInternal((FlightPidId_t)i, FLIGHT_GAIN_KP, data->gains[i][FLIGHT_GAIN_KP]))
        {
            return false;
        }
        if (!setPidGainInternal((FlightPidId_t)i, FLIGHT_GAIN_KI, data->gains[i][FLIGHT_GAIN_KI]))
        {
            return false;
        }
        if (!setPidGainInternal((FlightPidId_t)i, FLIGHT_GAIN_KD, data->gains[i][FLIGHT_GAIN_KD]))
        {
            return false;
        }
    }

    return true;
}

static float pressureToAltitudeM(float pressurePa, float refPressurePa)
{
    if (pressurePa < 1000.0f)
    {
        pressurePa = 1000.0f;
    }
    if (refPressurePa < 1000.0f)
    {
        refPressurePa = 1000.0f;
    }

    return 44330.0f * (1.0f - powf(pressurePa / refPressurePa, 0.19029495f));
}

/** 
 * @brief 将原始磁力计数据归一化为单位向量
 * @param magRaw 输入的原始磁力计数据（LSB，三轴）
 * @param magNorm 归一化后的向量
 * @return 1=归一化成功，0=输入无效（零向量）
 */
static uint8_t normalizeMag(const int16_t magRaw[3], float magNorm[3])
{
    float mx = (float)magRaw[0];
    float my = (float)magRaw[1];
    float mz = (float)magRaw[2];
    float norm = sqrtf(mx * mx + my * my + mz * mz);

    if (norm < 1.0f)
    {
        magNorm[0] = 0.0f;
        magNorm[1] = 0.0f;
        magNorm[2] = 0.0f;
        return 0;
    }

    magNorm[0] = mx / norm;
    magNorm[1] = my / norm;
    magNorm[2] = mz / norm;
    return 1;
}

/**
 * @brief   获取传感器数据是否有效
 * @param   ppm 指向PPM数据结构的指针，包含遥控器输入通道数据
 * @return  1=数据有效，0=数据无效
 * @note    该函数通过检查前4个PPM通道的值是否在预设的有效范围内来判断数据的有效性。
 */
static uint8_t isPpmFrameValid(const struct PPM_Data* ppm)
{
    uint8_t i;
    if (ppm == 0)
    {
        return 0;
    }

    for (i = 0; i < 4; i++)
    {
        if (ppm->ppmCh[i] < CONFIG_PPM_MIN_VALID || ppm->ppmCh[i] > CONFIG_PPM_MAX_VALID)
        {
            return 0;
        }
    }
    return 1;
}

static void applyMotorSafe(void)
{
    MotorOutput_t motor;
    motor.m1 = CONFIG_MOTOR_MIN_THROTTLE;
    motor.m2 = CONFIG_MOTOR_MIN_THROTTLE;
    motor.m3 = CONFIG_MOTOR_MIN_THROTTLE;
    motor.m4 = CONFIG_MOTOR_MIN_THROTTLE;
    Motor_WriteOutput(&motor);

    g_debug.m1 = motor.m1;
    g_debug.m2 = motor.m2;
    g_debug.m3 = motor.m3;
    g_debug.m4 = motor.m4;
}

/** 
 * @brief 判断IMU（惯性测量单元，包含陀螺仪和加速度计）是否处于静止状态
 * @param imu 指向GYRO_ACCEL_Data结构体的指针，包含陀螺仪和加速度计的原始数据
 * @return 1=IMU处于静止状态，0=IMU不处于静止状态
 */
static uint8_t isImuStill(const struct GYRO_ACCEL_Data* imu)
{
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
    float amag;

    gx = fabsf((float)imu->gyro[0] / 16.4f);
    gy = fabsf((float)imu->gyro[1] / 16.4f);
    gz = fabsf((float)imu->gyro[2] / 16.4f);

    if (gx > CONFIG_GYRO_STILL_THRESHOLD_DPS ||
        gy > CONFIG_GYRO_STILL_THRESHOLD_DPS ||
        gz > CONFIG_GYRO_STILL_THRESHOLD_DPS)
    {
        return 0;
    }

    ax = (float)imu->accel[0] / 16384.0f;
    ay = (float)imu->accel[1] / 16384.0f;
    az = (float)imu->accel[2] / 16384.0f;
    amag = sqrtf(ax * ax + ay * ay + az * az);

    if (fabsf(amag - 1.0f) > CONFIG_ACCEL_STILL_TOL_G)
    {
        return 0;
    }

    return 1;
}

/**
 * @brief 重置IMU零偏校准累积器
 * 该函数会将陀螺仪和加速度计的累积和清零，并重置校准样本计数器。
 * 在IMU不处于静止状态时调用，以确保后续的零偏计算基于新的静止数据。
 */
static void resetCalibrationAccumulator(void)
{
    g_gyroSum[0] = 0;
    g_gyroSum[1] = 0;
    g_gyroSum[2] = 0;
    g_accelSum[0] = 0;
    g_accelSum[1] = 0;
    g_accelSum[2] = 0;
    g_calibCount = 0;
}

/** 
 * @brief 执行一次IMU零偏置校准步骤
 * @param imu 指向GYRO_ACCEL_Data结构体的指针，包含陀螺仪和加速度计的原始数据
 */
static void runBiasCalibrationStep(const struct GYRO_ACCEL_Data* imu)
{
    if (g_biasReady)
    {
        return;
    }

    if (!isImuStill(imu))
    {
        resetCalibrationAccumulator();
        return;
    }

    g_gyroSum[0] += imu->gyro[0];
    g_gyroSum[1] += imu->gyro[1];
    g_gyroSum[2] += imu->gyro[2];
    g_accelSum[0] += imu->accel[0];
    g_accelSum[1] += imu->accel[1];
    g_accelSum[2] += imu->accel[2];
    g_calibCount++;

    if (g_calibCount >= CONFIG_SENSOR_CALIB_SAMPLES)
    {
        g_gyroBiasRaw[0] = (int32_t)(g_gyroSum[0] / g_calibCount);
        g_gyroBiasRaw[1] = (int32_t)(g_gyroSum[1] / g_calibCount);
        g_gyroBiasRaw[2] = (int32_t)(g_gyroSum[2] / g_calibCount);

        g_accelBiasRaw[0] = (int32_t)(g_accelSum[0] / g_calibCount);
        g_accelBiasRaw[1] = (int32_t)(g_accelSum[1] / g_calibCount);
        g_accelBiasRaw[2] = (int32_t)(g_accelSum[2] / g_calibCount) - 16384;

        g_biasReady = 1;
        resetCalibrationAccumulator();
    }
}

void FlightControl_Init(void)
{
    const float dt = 1.0f / (float)CONFIG_CONTROLLER_RATE_HZ;

    AttitudeEstimator_Init(&g_estimator, dt, 2.0f, 0.005f, true);
    AttitudeController_Init(&g_controller, dt);
    pidInit(&g_altitudePid, 0.0f, 1.2f, 0.25f, 0.03f, 0.0f,
            dt, 1.0f / dt, 5.0f, true);
    pidSetIntegralLimit(&g_altitudePid, 120.0f);
    g_altitudePid.outputLimit = 220.0f;

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, CONFIG_MOTOR_MIN_THROTTLE);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, CONFIG_MOTOR_MIN_THROTTLE);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, CONFIG_MOTOR_MIN_THROTTLE);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, CONFIG_MOTOR_MIN_THROTTLE);

    g_flightState = FLIGHT_STATE_DISARMED;
    g_biasReady = 0;
    g_gyroBiasRaw[0] = 0;
    g_gyroBiasRaw[1] = 0;
    g_gyroBiasRaw[2] = 0;
    g_accelBiasRaw[0] = 0;
    g_accelBiasRaw[1] = 0;
    g_accelBiasRaw[2] = 0;
    g_altitudeSp = 0.0f;
    g_altHoldActive = 0;
    g_baroRefPressurePa = SEA_LEVEL_PRESSURE;
    g_baroRefReady = 0;

    (void)loadPidParamsFromFlash();
    resetCalibrationAccumulator();
}

void FlightControl_GetDebugSnapshot(FlightDebugData_t* out)
{
    if (out == 0)
    {
        return;
    }

    *out = g_debug;
}

bool FlightControl_SetPidGain(FlightPidId_t pidId, FlightGainType_t gainType, float value)
{
    if (!setPidGainInternal(pidId, gainType, value))
    {
        return false;
    }

    if (!savePidParamsToFlash())
    {
        return false;
    }

    return true;
}

bool FlightControl_GetPidGain(FlightPidId_t pidId, FlightGainType_t gainType, float* outValue)
{
    PidObject* pid = getPidObjectById(pidId);

    if (pid == 0 || outValue == 0)
    {
        return false;
    }

    switch (gainType)
    {
        case FLIGHT_GAIN_KP:
            *outValue = pid->kp;
            return true;
        case FLIGHT_GAIN_KI:
            *outValue = pid->ki;
            return true;
        case FLIGHT_GAIN_KD:
            *outValue = pid->kd;
            return true;
        default:
            return false;
    }
}

/** 
 * @brief 飞行控制主任务
 * @param params 任务参数，当前未使用
 */
void FlightControl_Task(void* params)
{
    (void)params;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(2); /* 500Hz */
    float yawTargetDeg = 0.0f;

    /* ========== 原始传感器数据接收 ========== */
    struct GYRO_ACCEL_Data imuRaw;      /* 原始 IMU 数据（陀螺仪 + 加速度计），单位：LSB */
    struct MAG_Data magRaw;             /* 原始磁力计数据，单位：LSB */
    struct Pressure_Data pressureRaw;   /* 原始气压计数据，单位：Pa */
    struct PPM_Data ppmRaw;             /* 原始遥控器 PPM 信号数据 */
    
    /* ========== 传感器数据时间戳管理（用于超时检测）========== */
    TickType_t lastPpmTick;             /* PPM 信号最后接收时间戳 */
    TickType_t lastImuTick;             /* IMU 数据最后接收时间戳 */
    TickType_t lastMagTick;             /* 磁力计数据最后接收时间戳 */
    TickType_t lastPressureTick;        /* 气压计数据最后接收时间戳 */
    TickType_t armCmdStartTick = 0;     /* 解锁命令起始时间戳 */
    TickType_t disarmCmdStartTick = 0;  /* 上锁命令起始时间戳 */
    TickType_t nowTick;                 /* 当前系统时间戳 */

    /* ========== 控制流程核心数据结构 ========== */
    SensorData_t sensor;                /* 标准化传感器数据输入（供姿态估计器使用） */
    AttitudeSetpoint_t sp;              /* 姿态设定点（目标角度/角速度） */
    AttitudeState_t state;              /* 当前姿态状态（角度°、角速度°/s） */
    MotorOutput_t motor;                /* 电机 PWM 输出值 */
    ControlInput_t input;               /* 滤波后控制输入 */
    ControlInput_t inputRaw;            /* 原始控制输入（未滤波） */
    
    /* ========== 传感器数据处理与校正 ========== */
    int16_t gyroCorrected[3];           /* 零偏校正后的陀螺仪数据 [x,y,z]，单位：LSB */
    int16_t accelCorrected[3];          /* 零偏校正后的加速度计数据 [x,y,z]，单位：LSB */
    int16_t latestMagRaw[3] = {0, 0, 0};/* 最新磁力计原始值 [x,y,z]，单位：LSB */
    uint8_t magValid = 0;               /* 磁力计数据有效性标志（1=有效，0=无效/超时） */
    float filteredPressurePa = SEA_LEVEL_PRESSURE;  /* 滤波后气压值，单位：Pa */
    float altitudeM = 0.0f;             /* 相对起飞点的高度，单位：米（m） */
    uint8_t pressureValid = 0;          /* 气压计数据有效性标志（1=有效，0=无效/超时） */
    
    /* ========== 最终控制输出 ========== */
    uint16_t throttleCmd = CONFIG_MOTOR_MIN_THROTTLE;  /* 最终油门指令，范围：MIN_THROTTLE~MAX_THROTTLE */

    FlightControl_Init();
    lastPpmTick = xTaskGetTickCount();
    lastImuTick = lastPpmTick;
    lastMagTick = lastPpmTick;
    lastPressureTick = lastPpmTick;
    
    state.roll = 0.0f;
    state.pitch = 0.0f;
    state.yaw = 0.0f;
    state.rollRate = 0.0f;
    state.pitchRate = 0.0f;
    state.yawRate = 0.0f;
    yawTargetDeg = 0.0f;

    input.forward = 1500;
    input.lateral = 1500;
    input.yaw = 1500;
    input.lift = 1000;
    input.armed = false;

    inputRaw = input;

    for (;;)
    {
        nowTick = xTaskGetTickCount();

        if (xQueueReceive(QueueMAG, &magRaw, 0) == pdTRUE)
        {
            latestMagRaw[0] = magRaw.mag[0];
            latestMagRaw[1] = magRaw.mag[1];
            latestMagRaw[2] = magRaw.mag[2];
            magValid = 1;
            lastMagTick = nowTick;
        }

        if (xQueueReceive(QueuePressure, &pressureRaw, 0) == pdTRUE)
        {
            float pressurePa = (float)pressureRaw.pressure;

            if (pressurePa > 1000.0f)
            {
                pressureValid = 1;
                lastPressureTick = nowTick;
                filteredPressurePa = filteredPressurePa * 0.9f + pressurePa * 0.1f;

                if (!g_baroRefReady || g_flightState != FLIGHT_STATE_ARMED)
                {
                    g_baroRefPressurePa = filteredPressurePa;
                    g_baroRefReady = 1;
                }

                altitudeM = pressureToAltitudeM(filteredPressurePa, g_baroRefPressurePa);
            }
        }

        if (xQueueReceive(QueueGYROACCEL, &imuRaw, 0) == pdTRUE)
        {
            lastImuTick = nowTick;
            if (g_flightState != FLIGHT_STATE_ARMED)
            {
                runBiasCalibrationStep(&imuRaw);
            }

            gyroCorrected[0] = (int16_t)(imuRaw.gyro[0] - g_gyroBiasRaw[0]);
            gyroCorrected[1] = (int16_t)(imuRaw.gyro[1] - g_gyroBiasRaw[1]);
            gyroCorrected[2] = (int16_t)(imuRaw.gyro[2] - g_gyroBiasRaw[2]);

            accelCorrected[0] = (int16_t)(imuRaw.accel[0] - g_accelBiasRaw[0]);
            accelCorrected[1] = (int16_t)(imuRaw.accel[1] - g_accelBiasRaw[1]);
            accelCorrected[2] = (int16_t)(imuRaw.accel[2] - g_accelBiasRaw[2]);

            SensorData_ConvertFromRaw(gyroCorrected, accelCorrected, &sensor);

            if (magValid)
            {
                if (!normalizeMag(latestMagRaw, sensor.mag))
                {
                    sensor.mag[0] = 0.0f;
                    sensor.mag[1] = 0.0f;
                    sensor.mag[2] = 0.0f;
                }
            }

            AttitudeEstimator_Update(&g_estimator, &sensor);

            const EulerAngle_t* e = AttitudeEstimator_GetEuler(&g_estimator);

            state.roll = e->roll;
            state.pitch = e->pitch;
            state.yaw = e->yaw;
            state.rollRate = (float)gyroCorrected[0] / 16.4f;
            state.pitchRate = (float)gyroCorrected[1] / 16.4f;
            state.yawRate = (float)gyroCorrected[2] / 16.4f;
        }

        if (magValid && ((nowTick - lastMagTick) > pdMS_TO_TICKS(CONFIG_MAG_STALE_TIMEOUT_MS)))
        {
            magValid = 0;
        }

        if (pressureValid && ((nowTick - lastPressureTick) > pdMS_TO_TICKS(CONFIG_PRESSURE_STALE_TIMEOUT_MS)))
        {
            pressureValid = 0;
            g_altHoldActive = 0;
            pidReset(&g_altitudePid, altitudeM);
        }

        if (xQueueReceive(QueuePPM, &ppmRaw, 0) == pdTRUE)
        {
            if (isPpmFrameValid(&ppmRaw))
            {
                inputRaw.lateral = ppmRaw.ppmCh[0];
                inputRaw.forward = invertPpmChannelU16(ppmRaw.ppmCh[1]);
                inputRaw.lift = invertPpmChannelU16(ppmRaw.ppmCh[2]);
                inputRaw.yaw = ppmRaw.ppmCh[3];
                lastPpmTick = nowTick;
            }
        }

        input.lateral = lowPassChannelU16(input.lateral, inputRaw.lateral, RC_FILTER_ALPHA);
        input.forward = lowPassChannelU16(input.forward, inputRaw.forward, RC_FILTER_ALPHA);
        input.lift = lowPassChannelU16(input.lift, inputRaw.lift, RC_FILTER_ALPHA);
        input.yaw = lowPassChannelU16(input.yaw, inputRaw.yaw, RC_FILTER_ALPHA);

        if (((nowTick - lastPpmTick) > pdMS_TO_TICKS(CONFIG_PPM_FAILSAFE_TIMEOUT_MS)) ||
            ((g_flightState == FLIGHT_STATE_ARMED) &&
             ((nowTick - lastImuTick) > pdMS_TO_TICKS(CONFIG_IMU_FAILSAFE_TIMEOUT_MS))))
        {
            g_flightState = FLIGHT_STATE_FAILSAFE;
        }
        else if (g_flightState == FLIGHT_STATE_FAILSAFE)
        {
            g_flightState = FLIGHT_STATE_DISARMED;
        }

        if (inputRaw.lift <= CONFIG_THROTTLE_MIN)
        {
            if (inputRaw.yaw >= CONFIG_ARM_YAW_HIGH)
            {
                if (armCmdStartTick == 0)
                {
                    armCmdStartTick = nowTick;
                }
                else if ((g_flightState == FLIGHT_STATE_DISARMED) &&
                         g_biasReady &&
                         (nowTick - armCmdStartTick >= pdMS_TO_TICKS(CONFIG_ARM_HOLD_MS)))
                {
                    g_flightState = FLIGHT_STATE_ARMED;
                    AttitudeController_Reset(&g_controller, &state);
                }
            }
            else
            {
                armCmdStartTick = 0;
            }

            if (inputRaw.yaw <= CONFIG_DISARM_YAW_LOW)
            {
                if (disarmCmdStartTick == 0)
                {
                    disarmCmdStartTick = nowTick;
                }
                else if ((g_flightState == FLIGHT_STATE_ARMED) &&
                         (nowTick - disarmCmdStartTick >= pdMS_TO_TICKS(CONFIG_DISARM_HOLD_MS)))
                {
                    g_flightState = FLIGHT_STATE_DISARMED;
                }
            }
            else
            {
                disarmCmdStartTick = 0;
            }
        }
        else
        {
            armCmdStartTick = 0;
            disarmCmdStartTick = 0;
        }

        input.armed = (g_flightState == FLIGHT_STATE_ARMED);

        if (g_flightState != FLIGHT_STATE_ARMED)
        {
            AttitudeController_Reset(&g_controller, &state);
            g_altHoldActive = 0;
            g_altitudeSp = altitudeM;
            pidReset(&g_altitudePid, altitudeM);
            applyMotorSafe();
            yawTargetDeg = state.yaw;

            g_debug.rollSp = 0.0f;
            g_debug.pitchSp = 0.0f;
            g_debug.yawRateSp = 0.0f;
            g_debug.rollOut = 0.0f;
            g_debug.pitchOut = 0.0f;
            g_debug.yawOut = 0.0f;
        }
        else
        {
            float yawStick;
            float yawErr;
            float liftStick;
            float altitudeOut = 0.0f;

            AttitudeController_GenerateSetpoint(&input, &sp);

            yawStick = constrain(((float)input.yaw - 1500.0f) / 500.0f, -1.0f, 1.0f);
            liftStick = constrain(((float)input.lift - 1500.0f) / 500.0f, -1.0f, 1.0f);

            throttleCmd = input.lift;

            if (pressureValid && g_baroRefReady)
            {
                if (fabsf(liftStick) < 0.08f)
                {
                    if (!g_altHoldActive)
                    {
                        g_altHoldActive = 1;
                        g_altitudeSp = altitudeM;
                        pidReset(&g_altitudePid, altitudeM);
                    }

                    pidSetDesired(&g_altitudePid, g_altitudeSp);
                    altitudeOut = pidUpdate(&g_altitudePid, altitudeM, false);
                    throttleCmd = (uint16_t)constrain((float)input.lift + altitudeOut,
                                                     (float)CONFIG_MOTOR_IDLE_THROTTLE,
                                                     (float)CONFIG_MOTOR_MAX_THROTTLE);
                }
                else
                {
                    g_altHoldActive = 0;
                    g_altitudeSp = altitudeM;
                    pidReset(&g_altitudePid, altitudeM);
                }
            }

            if (fabsf(yawStick) < 0.05f)
            {
                yawErr = capAngle(yawTargetDeg - state.yaw);
                sp.yawRate = constrain(yawErr * 4.0f, -CONFIG_MAX_YAW_RATE, CONFIG_MAX_YAW_RATE);
            }
            else
            {
                yawTargetDeg = state.yaw;
            }

            AttitudeController_Update(&g_controller, &sp, &state, false);
            AttitudeController_MixToMotor(&g_controller, throttleCmd, &motor);
            Motor_WriteOutput(&motor);

            g_debug.rollSp = sp.roll;
            g_debug.pitchSp = sp.pitch;
            g_debug.yawRateSp = sp.yawRate;
            g_debug.rollOut = g_controller.rollOut;
            g_debug.pitchOut = g_controller.pitchOut;
            g_debug.yawOut = g_controller.yawOut;
            g_debug.m1 = motor.m1;
            g_debug.m2 = motor.m2;
            g_debug.m3 = motor.m3;
            g_debug.m4 = motor.m4;
        }

        g_debug.state = g_flightState;
        g_debug.linkAlive = ((nowTick - lastPpmTick) <= pdMS_TO_TICKS(CONFIG_PPM_FAILSAFE_TIMEOUT_MS)) ? 1u : 0u;
        g_debug.sensorCalibrated = g_biasReady;
        g_debug.roll = state.roll;
        g_debug.pitch = state.pitch;
        g_debug.yaw = state.yaw;
        g_debug.throttle = throttleCmd;

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}
