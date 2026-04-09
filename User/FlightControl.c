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
#define FC_OUTER_LOOP_DIV  (CONFIG_CONTROLLER_RATE_HZ / CONFIG_OUTER_LOOP_RATE_HZ)
#define PID_FLASH_ADDR       0x08060000U
#define PID_FLASH_MAGIC      0x50494431U
#define PID_FLASH_VERSION_V1 0x00010000U
#define PID_FLASH_VERSION    0x00010001U
#define RC_FILTER_ALPHA      0.20f
#define IMU_DROP_STAT_WINDOW_CYCLES ((uint16_t)CONFIG_CONTROLLER_RATE_HZ)
#define IMU_CTRL_SCALE_STALE         0.40f
#define IMU_CTRL_SCALE_PREFAILSAFE   0.15f

/**
 * @brief IMU 数据新鲜度枚举类型
 * @note 四个新鲜度等级
 */
typedef enum
{
    IMU_FRESHNESS_FRESH = 0,
    IMU_FRESHNESS_STALE,
    IMU_FRESHNESS_PREFAILSAFE,
    IMU_FRESHNESS_LOST
} ImuFreshness_t;

/**
 * @brief 飞行控制器 PID 参数持久化结构体
 */
typedef struct
{
    uint32_t magic;       // 魔数标识（0x50494431U），用于验证 Flash 中 PID 数据的有效性
    uint32_t version;     // 版本号（0x00010001U），用于管理 PID 参数格式的版本兼容性
    float gains[FLIGHT_PID_COUNT][3];  // PID 增益参数数组 [通道数][3]，存储 P/I/D 三参数（单位：比例系数）
    float iLimits[FLIGHT_PID_COUNT];    // PID 积分限幅
    float outputLimits[FLIGHT_PID_COUNT]; // PID 总输出限幅
    uint32_t checksum;    // 校验和，用于验证 Flash 中 PID 数据的完整性（防止数据损坏）
} FlightPidPersist_t;

typedef struct
{
    uint32_t magic;
    uint32_t version;
    float gains[FLIGHT_PID_COUNT][3];
    uint32_t checksum;
} FlightPidPersistV1_t;

/* ========== 飞控核心模块实例 ========== */
static AttitudeEstimator_t g_estimator;    // Mahony姿态估计器（融合 IMU 数据解算姿态）
static AttitudeController_t g_controller;  // 双环 PID 姿态控制器（角度环 + 角速度环）
static PidObject g_altitudePid;            // 高度 PID 控制器（用于气压定高模式）
static volatile FlightDebugData_t g_debug; // 调试数据（通过串口/OLED 实时输出）
static FlightState_t g_flightState = FLIGHT_STATE_DISARMED;  // 当前飞行状态（解锁/锁定等）
static volatile FlightPidSetError_t g_lastPidSetError = FLIGHT_PID_SET_OK;
static TaskHandle_t g_flightControlTaskHandle = NULL;

/* ========== 高度控制参数 ========== */
static float g_altitudeSp = 0.0f;          // 目标设定点高度（米，定高模式使用）
static uint8_t g_altHoldActive = 0;        // 定高模式激活标志（1=激活，0=关闭）
static float g_baroRefPressurePa = SEA_LEVEL_PRESSURE;  // 参考气压值（起飞点气压，用于计算相对高度）
static uint8_t g_baroRefReady = 0;         // 参考气压就绪标志（1=已校准，0=未校准）
#if CONFIG_GIMBAL_DEBUG_MODE
static float g_debugTiltAngleLimitDeg = 0.0f;
#endif

/* ========== IMU 零偏校准参数 ========== */
static int32_t g_gyroBiasRaw[3] = {0, 0, 0};  // 陀螺仪零偏校正值（LSB，三轴）
static int32_t g_accelBiasRaw[3] = {0, 0, 0}; // 加速度计零偏校正值（LSB，三轴）
static uint8_t g_biasReady = 0;            // 零偏校准就绪标志（1=校准完成，0=校准中）

/* ========== IMU 校准过程变量 ========== */
static int64_t g_gyroSum[3] = {0, 0, 0};   // 陀螺仪累加和（用于滑动平均滤波）
static int64_t g_accelSum[3] = {0, 0, 0};  // 加速度计累加和（用于滑动平均滤波）
static uint16_t g_calibCount = 0;          // 校准采样计数器

/**
 * @brief 计算 PID 参数的校验和
 * @param data PID 参数结构体指针
 * @return 校验和（32 位无符号整数）
 */
static uint32_t calcPersistChecksum(const FlightPidPersist_t* data)
{
    const uint8_t* bytes;
    uint32_t sum = 0xA5A5A5A5U;
    uint32_t i;

    if (data == 0)
    {
        return 0U;
    }

    bytes = (const uint8_t*)data;

    for (i = 0; i < (uint32_t)(sizeof(FlightPidPersist_t) - sizeof(uint32_t)); i++)
    {
        sum ^= bytes[i];
        sum = (sum << 5) | (sum >> 27);
        sum += 0x9E3779B9U;
    }

    return sum;
}

static uint32_t calcPersistChecksumV1(const FlightPidPersistV1_t* data)
{
    const uint8_t* bytes;
    uint32_t sum = 0xA5A5A5A5U;
    uint32_t i;

    if (data == 0)
    {
        return 0U;
    }

    bytes = (const uint8_t*)data;
    for (i = 0; i < (uint32_t)(sizeof(FlightPidPersistV1_t) - sizeof(uint32_t)); i++)
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

/** 
 * @brief 反转 PPM 通道值（将输入范围 [MIN, MAX] 映射到 [MAX, MIN]）
 * @param input 原始输入值
 * @return 反转后的输出值
 */
static uint16_t invertPpmChannelU16(uint16_t input)
{
    uint16_t constrained = (uint16_t)constrain((float)input,
                                               (float)CONFIG_PPM_MIN_VALID,
                                               (float)CONFIG_PPM_MAX_VALID);
    return (uint16_t)(CONFIG_PPM_MIN_VALID + CONFIG_PPM_MAX_VALID - constrained);
}

/**
 * @brief 调试模式下油门限幅
 * @param throttle 输入的油门值
 */
static uint16_t clampThrottleForDebug(uint16_t throttle)
{
#if CONFIG_GIMBAL_DEBUG_MODE
    return (uint16_t)constrain((float)throttle,
                               (float)CONFIG_MOTOR_MIN_THROTTLE,
                               (float)CONFIG_GIMBAL_DEBUG_MAX_THROTTLE);
#else
    return throttle;
#endif
}

/**
 * @brief 调试模式下电机输出限幅
 * @param motor 输入的电机输出结构体指针
 */
static void clampMotorOutputForDebug(MotorOutput_t* motor)
{
#if CONFIG_GIMBAL_DEBUG_MODE
    if (motor == 0)
    {
        return;
    }

    motor->m1 = (uint16_t)constrain((float)motor->m1,
                                    (float)CONFIG_MOTOR_MIN_THROTTLE,
                                    (float)CONFIG_GIMBAL_DEBUG_MAX_THROTTLE);
    motor->m2 = (uint16_t)constrain((float)motor->m2,
                                    (float)CONFIG_MOTOR_MIN_THROTTLE,
                                    (float)CONFIG_GIMBAL_DEBUG_MAX_THROTTLE);
    motor->m3 = (uint16_t)constrain((float)motor->m3,
                                    (float)CONFIG_MOTOR_MIN_THROTTLE,
                                    (float)CONFIG_GIMBAL_DEBUG_MAX_THROTTLE);
    motor->m4 = (uint16_t)constrain((float)motor->m4,
                                    (float)CONFIG_MOTOR_MIN_THROTTLE,
                                    (float)CONFIG_GIMBAL_DEBUG_MAX_THROTTLE);
#else
    (void)motor;
#endif
}

/** 
 * @brief 根据 PID ID 获取对应的 PID 对象
 * @param pidId PID ID
 * @return PID 对象指针，若未找到则返回 0
 */
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

/**
 * @brief 设置 PID 参数
 * @param pidId PID ID
 * @param gainType 参数类型（比例系数、积分系数、微分系数）
 * @param value 参数值
 * @return 设置成功则返回 true，否则返回 false
 */
static bool setPidGainInternal(FlightPidId_t pidId, FlightGainType_t gainType, float value)
{
    PidObject* pid = getPidObjectById(pidId);

    if (pid == 0)
    {
        return false;
    }

    if (!isfinite(value) || value < 0.0f)
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

static bool setPidLimitInternal(FlightPidId_t pidId, FlightLimitType_t limitType, float value)
{
    PidObject* pid = getPidObjectById(pidId);

    if (pid == 0)
    {
        return false;
    }

    if (!isfinite(value) || value < 0.0f)
    {
        return false;
    }

    switch (limitType)
    {
        case FLIGHT_LIMIT_INTEGRAL:
            pid->iLimit = value;
            return true;
        case FLIGHT_LIMIT_OUTPUT:
            pid->outputLimit = value;
            return true;
        default:
            return false;
    }
}

static bool fillPersistFromCurrent(FlightPidPersist_t* data)
{
    uint32_t i;
    PidObject* pid;

    if (data == 0)
    {
        return false;
    }

    memset(data, 0, sizeof(*data));
    data->magic = PID_FLASH_MAGIC;
    data->version = PID_FLASH_VERSION;

    for (i = 0; i < (uint32_t)FLIGHT_PID_COUNT; i++)
    {
        pid = getPidObjectById((FlightPidId_t)i);
        if (pid == 0)
        {
            return false;
        }

        data->gains[i][FLIGHT_GAIN_KP] = pid->kp;
        data->gains[i][FLIGHT_GAIN_KI] = pid->ki;
        data->gains[i][FLIGHT_GAIN_KD] = pid->kd;
        data->iLimits[i] = pid->iLimit;
        data->outputLimits[i] = pid->outputLimit;
    }

    return true;
}

/**
 * @brief 保存 PID 参数到 Flash
 * @return 成功保存则返回 true，否则返回 false
 * @note 函数会先擦除 Flash 中保存的 PID 参数，然后写入新的参数。
 */
static bool savePidParamsToFlash(const FlightPidPersist_t* data)
{
    uint32_t wordAddress;
    uint32_t i;
    uint32_t sectorError = 0;
    HAL_StatusTypeDef halStatus;
    FLASH_EraseInitTypeDef erase;
    const uint32_t* words;

    if (data == 0)
    {
        return false;
    }

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

    words = (const uint32_t*)data;
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

/**
 * @brief 从 Flash 中加载 PID 参数
 * @return 读取成功则返回 true，否则返回 false
 * @note 函数会先检查 Flash 中保存的 PID 参数的校验和，如果校验和正确，则读取参数。
 */
static bool loadPidParamsFromFlash(void)
{
    const FlightPidPersist_t* data = (const FlightPidPersist_t*)PID_FLASH_ADDR;
    const FlightPidPersistV1_t* dataV1 = (const FlightPidPersistV1_t*)PID_FLASH_ADDR;
    uint32_t i;

    if (data->magic != PID_FLASH_MAGIC)
    {
        return false;
    }

    if (data->version == PID_FLASH_VERSION)
    {
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
            if (!setPidLimitInternal((FlightPidId_t)i, FLIGHT_LIMIT_INTEGRAL, data->iLimits[i]))
            {
                return false;
            }
            if (!setPidLimitInternal((FlightPidId_t)i, FLIGHT_LIMIT_OUTPUT, data->outputLimits[i]))
            {
                return false;
            }
        }

        return true;
    }

    if (dataV1->version == PID_FLASH_VERSION_V1)
    {
        if (dataV1->checksum != calcPersistChecksumV1(dataV1))
        {
            return false;
        }

        for (i = 0; i < (uint32_t)FLIGHT_PID_COUNT; i++)
        {
            if (!setPidGainInternal((FlightPidId_t)i, FLIGHT_GAIN_KP, dataV1->gains[i][FLIGHT_GAIN_KP]))
            {
                return false;
            }
            if (!setPidGainInternal((FlightPidId_t)i, FLIGHT_GAIN_KI, dataV1->gains[i][FLIGHT_GAIN_KI]))
            {
                return false;
            }
            if (!setPidGainInternal((FlightPidId_t)i, FLIGHT_GAIN_KD, dataV1->gains[i][FLIGHT_GAIN_KD]))
            {
                return false;
            }
        }

        return true;
    }

    return false;
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

/**
 * @brief 将所有电机输出设置为安全的最低油门值
 * @note 该函数在飞行器解锁失败或发生严重错误时调用，以确保电机不会意外启动。
 */
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

void Motor_Init(void)
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, CONFIG_MOTOR_MAX_THROTTLE);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, CONFIG_MOTOR_MAX_THROTTLE);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, CONFIG_MOTOR_MAX_THROTTLE);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, CONFIG_MOTOR_MAX_THROTTLE);
    vTaskDelay(2500);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, CONFIG_MOTOR_MIN_THROTTLE);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, CONFIG_MOTOR_MIN_THROTTLE);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, CONFIG_MOTOR_MIN_THROTTLE);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, CONFIG_MOTOR_MIN_THROTTLE);
    vTaskDelay(2500);
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

    //Motor_Init();
        printf("motor ok\r\n");
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, CONFIG_MOTOR_MIN_THROTTLE);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, CONFIG_MOTOR_MIN_THROTTLE);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, CONFIG_MOTOR_MIN_THROTTLE);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, CONFIG_MOTOR_MIN_THROTTLE);

    g_flightState = FLIGHT_STATE_DISARMED;
    printf("[FC][STATE] -> DISARMED, reason=init\r\n");
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

#if CONFIG_GIMBAL_DEBUG_MODE
    {
        float maxSafeTilt = (CONFIG_MAX_ROLL_ANGLE < CONFIG_MAX_PITCH_ANGLE) ?
                            CONFIG_MAX_ROLL_ANGLE : CONFIG_MAX_PITCH_ANGLE;
        g_debugTiltAngleLimitDeg = CONFIG_GIMBAL_DEBUG_TILT_ANGLE_VALID(CONFIG_GIMBAL_DEBUG_MAX_TILT_ANGLE) ?
                                   CONFIG_GIMBAL_DEBUG_MAX_TILT_ANGLE : maxSafeTilt;
    }
#endif

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

void FlightControl_NotifyImuSampleReady(void)
{
    TaskHandle_t taskHandle = g_flightControlTaskHandle;

    if (taskHandle != NULL)
    {
        xTaskNotifyGive(taskHandle);
    }
}

/**
 * @brief 设置 PID 增益参数
 * @param pidId PID ID
 * @param gainType 增益类型
 * @param value 增益值
 * @return 设置是否成功
 */
bool FlightControl_SetPidGain(FlightPidId_t pidId, FlightGainType_t gainType, float value)
{
    FlightPidPersist_t data;

    g_lastPidSetError = FLIGHT_PID_SET_OK;

    /* 禁止在已解锁/失控保护状态写入 Flash，避免飞行中产生实时性抖动。 */
    if (g_flightState != FLIGHT_STATE_DISARMED)
    {
        g_lastPidSetError = FLIGHT_PID_SET_ERR_NOT_DISARMED;
        return false;
    }

    if (!isfinite(value) || value < 0.0f)
    {
        g_lastPidSetError = FLIGHT_PID_SET_ERR_INVALID_INPUT;
        return false;
    }

    if ((uint32_t)pidId >= (uint32_t)FLIGHT_PID_COUNT || (uint32_t)gainType > (uint32_t)FLIGHT_GAIN_KD)
    {
        g_lastPidSetError = FLIGHT_PID_SET_ERR_INVALID_INPUT;
        return false;
    }

    if (!fillPersistFromCurrent(&data))
    {
        g_lastPidSetError = FLIGHT_PID_SET_ERR_INTERNAL;
        return false;
    }

    data.gains[pidId][gainType] = value;
    data.checksum = calcPersistChecksum(&data);

    if (!savePidParamsToFlash(&data))
    {
        g_lastPidSetError = FLIGHT_PID_SET_ERR_FLASH_WRITE;
        return false;
    }

    if (!setPidGainInternal(pidId, gainType, value))
    {
        g_lastPidSetError = FLIGHT_PID_SET_ERR_INTERNAL;
        return false;
    }

    return true;
}

FlightPidSetError_t FlightControl_GetLastPidSetError(void)
{
    return g_lastPidSetError;
}

/**
 * @brief 获取 PID 增益参数
 * @param pidId PID ID
 * @param gainType 增益类型
 * @param outValue 储存获取的增益值的指针
 * @return 获取是否成功
 */
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

bool FlightControl_SetPidLimit(FlightPidId_t pidId, FlightLimitType_t limitType, float value)
{
    FlightPidPersist_t data;

    g_lastPidSetError = FLIGHT_PID_SET_OK;

    /* 与 PID 增益调参保持同一安全策略：仅允许在 DISARMED 状态修改。 */
    if (g_flightState != FLIGHT_STATE_DISARMED)
    {
        g_lastPidSetError = FLIGHT_PID_SET_ERR_NOT_DISARMED;
        return false;
    }

    if (!isfinite(value) || value < 0.0f)
    {
        g_lastPidSetError = FLIGHT_PID_SET_ERR_INVALID_INPUT;
        return false;
    }

    if ((uint32_t)pidId >= (uint32_t)FLIGHT_PID_COUNT || (uint32_t)limitType > (uint32_t)FLIGHT_LIMIT_OUTPUT)
    {
        g_lastPidSetError = FLIGHT_PID_SET_ERR_INVALID_INPUT;
        return false;
    }

    if (!fillPersistFromCurrent(&data))
    {
        g_lastPidSetError = FLIGHT_PID_SET_ERR_INTERNAL;
        return false;
    }

    if (limitType == FLIGHT_LIMIT_INTEGRAL)
    {
        data.iLimits[pidId] = value;
    }
    else
    {
        data.outputLimits[pidId] = value;
    }

    data.checksum = calcPersistChecksum(&data);

    if (!savePidParamsToFlash(&data))
    {
        g_lastPidSetError = FLIGHT_PID_SET_ERR_FLASH_WRITE;
        return false;
    }

    if (!setPidLimitInternal(pidId, limitType, value))
    {
        g_lastPidSetError = FLIGHT_PID_SET_ERR_INTERNAL;
        return false;
    }

    return true;
}

bool FlightControl_GetPidLimit(FlightPidId_t pidId, FlightLimitType_t limitType, float* outValue)
{
    PidObject* pid = getPidObjectById(pidId);

    if (pid == 0 || outValue == 0)
    {
        return false;
    }

    switch (limitType)
    {
        case FLIGHT_LIMIT_INTEGRAL:
            *outValue = pid->iLimit;
            return true;
        case FLIGHT_LIMIT_OUTPUT:
            *outValue = pid->outputLimit;
            return true;
        default:
            return false;
    }
}

/**
 * @brief 更新 IMU 新鲜度状态并统计丢包率
 * @param imuSeen IMU 是否已经接收到过样本
 * @param imuNewSample 本周期是否接收到新样本
 * @param nowTick 当前系统 Tick
 * @param lastImuTick 最近一次 IMU 新样本 Tick
 * @param imuFreshness IMU 新鲜度输出指针
 * @param imuStatCycles IMU 统计窗口内总周期数
 * @param imuStatDrops IMU 统计窗口内丢样本次数
 * @param imuDropRatePct IMU 丢包率输出（百分比）
 */
static void updateImuFreshnessAndDropStats(uint8_t imuSeen,
                                           uint8_t imuNewSample,
                                           TickType_t nowTick,
                                           TickType_t lastImuTick,
                                           ImuFreshness_t* imuFreshness,
                                           uint16_t* imuStatCycles,
                                           uint16_t* imuStatDrops,
                                           float* imuDropRatePct)
{
    TickType_t imuAgeTicks; /* IMU 样本年龄（tick） */

    if (imuFreshness == 0 || imuStatCycles == 0 || imuStatDrops == 0 || imuDropRatePct == 0)
    {
        return;
    }

    if (!imuSeen)
    {
        *imuFreshness = IMU_FRESHNESS_LOST;
        return;
    }

    imuAgeTicks = nowTick - lastImuTick;
    if (imuAgeTicks > pdMS_TO_TICKS(CONFIG_IMU_FAILSAFE_TIMEOUT_MS))
    {
        *imuFreshness = IMU_FRESHNESS_LOST;
    }
    else if (imuAgeTicks > pdMS_TO_TICKS(CONFIG_IMU_PREFAILSAFE_TIMEOUT_MS))
    {
        *imuFreshness = IMU_FRESHNESS_PREFAILSAFE;
    }
    else if (imuAgeTicks > pdMS_TO_TICKS(CONFIG_IMU_STALE_TIMEOUT_MS))
    {
        *imuFreshness = IMU_FRESHNESS_STALE;
    }
    else
    {
        *imuFreshness = IMU_FRESHNESS_FRESH;
    }

    (*imuStatCycles)++;
    if (!imuNewSample)
    {
        (*imuStatDrops)++;
    }

    if (*imuStatCycles >= IMU_DROP_STAT_WINDOW_CYCLES)
    {
        *imuDropRatePct = ((float)(*imuStatDrops) * 100.0f) / (float)(*imuStatCycles);
        *imuStatCycles = 0;
        *imuStatDrops = 0;
    }
}

/**
 * @brief 使用 IMU 数据更新姿态状态（含零偏校正与磁力计融合）
 * @param imuSeen IMU 是否已经接收到过样本
 * @param imuRaw 当前 IMU 原始样本
 * @param gyroCorrected 零偏校正后的陀螺仪数据输出
 * @param accelCorrected 零偏校正后的加速度计数据输出
 * @param sensor 供估计器使用的标准化传感器输入
 * @param magValid 磁力计样本是否有效
 * @param latestMagRaw 最新磁力计原始值
 * @param imuFreshness IMU 新鲜度状态
 * @param state 当前姿态状态输出
 */
static void updateAttitudeStateFromImu(uint8_t imuSeen,
                                       const struct GYRO_ACCEL_Data* imuRaw,
                                       int16_t gyroCorrected[3],
                                       int16_t accelCorrected[3],
                                       SensorData_t* sensor,
                                       uint8_t magValid,
                                       const int16_t latestMagRaw[3],
                                       ImuFreshness_t imuFreshness,
                                       AttitudeState_t* state)
{
    const EulerAngle_t* e; /* 姿态估计器输出的欧拉角 */

    if (!imuSeen || imuRaw == 0 || sensor == 0 || latestMagRaw == 0 || state == 0)
    {
        return;
    }

    gyroCorrected[0] = (int16_t)(imuRaw->gyro[0] - g_gyroBiasRaw[0]);
    gyroCorrected[1] = (int16_t)(imuRaw->gyro[1] - g_gyroBiasRaw[1]);
    gyroCorrected[2] = (int16_t)(imuRaw->gyro[2] - g_gyroBiasRaw[2]);

    accelCorrected[0] = (int16_t)(imuRaw->accel[0] - g_accelBiasRaw[0]);
    accelCorrected[1] = (int16_t)(imuRaw->accel[1] - g_accelBiasRaw[1]);
    accelCorrected[2] = (int16_t)(imuRaw->accel[2] - g_accelBiasRaw[2]);

    SensorData_ConvertFromRaw(gyroCorrected, accelCorrected, sensor);

    if (magValid)
    {
        if (!normalizeMag(latestMagRaw, sensor->mag))
        {
            sensor->mag[0] = 0.0f;
            sensor->mag[1] = 0.0f;
            sensor->mag[2] = 0.0f;
        }
    }

    if (imuFreshness == IMU_FRESHNESS_FRESH)
    {
        AttitudeEstimator_Update(&g_estimator, sensor);
        e = AttitudeEstimator_GetEuler(&g_estimator);

        state->roll = e->roll;
        state->pitch = e->pitch;
        state->yaw = e->yaw;
        state->rollRate = (float)gyroCorrected[0] / 16.4f;
        state->pitchRate = (float)gyroCorrected[1] / 16.4f;
        state->yawRate = (float)gyroCorrected[2] / 16.4f;
    }
    else
    {
        /* IMU 数据陈旧时冻结姿态并清零角速度反馈，避免重复积分旧样本。 */
        state->rollRate = 0.0f;
        state->pitchRate = 0.0f;
        state->yawRate = 0.0f;
    }
}

/**
 * @brief 根据 PPM/IMU 超时状态更新飞控状态机（失控保护）
 * @param nowTick 当前系统 Tick
 * @param lastPpmTick 最近一次有效 PPM Tick
 * @param lastImuTick 最近一次有效 IMU Tick
 * @param state 当前姿态状态
 */
static void updateFailsafeState(TickType_t nowTick, TickType_t lastPpmTick, TickType_t lastImuTick, const AttitudeState_t* state)
{
    uint8_t ppmTimedOut; /* PPM 是否超时 */
    uint8_t imuTimedOut; /* IMU 是否超时（仅解锁后生效） */

    ppmTimedOut = ((nowTick - lastPpmTick) > pdMS_TO_TICKS(CONFIG_PPM_FAILSAFE_TIMEOUT_MS)) ? 1u : 0u;
    imuTimedOut = ((g_flightState == FLIGHT_STATE_ARMED) &&
                   ((nowTick - lastImuTick) > pdMS_TO_TICKS(CONFIG_IMU_FAILSAFE_TIMEOUT_MS))) ? 1u : 0u;

    if (ppmTimedOut || imuTimedOut)
    {
        applyMotorSafe();
        AttitudeController_Reset(&g_controller, state);
        if (g_flightState != FLIGHT_STATE_FAILSAFE)
        {
            printf("[FC][STATE] -> FAILSAFE, reason=%s%s, ppmAge=%lu, imuAge=%lu\r\n",
                   ppmTimedOut ? "PPM_TIMEOUT" : "",
                   imuTimedOut ? (ppmTimedOut ? "+IMU_TIMEOUT" : "IMU_TIMEOUT") : "",
                   (unsigned long)(nowTick - lastPpmTick),
                   (unsigned long)(nowTick - lastImuTick));
        }
        g_flightState = FLIGHT_STATE_FAILSAFE;
    }
    else if (g_flightState == FLIGHT_STATE_FAILSAFE)
    {
        g_flightState = FLIGHT_STATE_DISARMED;
        printf("[FC][STATE] -> DISARMED, reason=RECOVER_FROM_FAILSAFE\r\n");
    }
}

/**
 * @brief 处理油门最低位下的解锁/上锁摇杆命令
 * @param inputRaw 原始遥控输入
 * @param nowTick 当前系统 Tick
 * @param armCmdStartTick 解锁命令计时起点
 * @param disarmCmdStartTick 上锁命令计时起点
 * @param state 当前姿态状态（解锁时用于控制器复位）
 */
static void updateArmDisarmCommand(const ControlInput_t* inputRaw,
                                   TickType_t nowTick,
                                   TickType_t* armCmdStartTick,
                                   TickType_t* disarmCmdStartTick,
                                   const AttitudeState_t* state)
{
    if (inputRaw == 0 || armCmdStartTick == 0 || disarmCmdStartTick == 0 || state == 0)
    {
        return;
    }

    if (inputRaw->lift <= CONFIG_THROTTLE_MIN)
    {
        if (inputRaw->yaw <= CONFIG_DISARM_YAW_LOW)
        {
            //printf("%d\r\n", nowTick - *armCmdStartTick);
            if (*armCmdStartTick == 0)
            {
                *armCmdStartTick = nowTick;
            }
            else if ((g_flightState == FLIGHT_STATE_DISARMED) &&
                     g_biasReady &&
                     (nowTick - *armCmdStartTick >= pdMS_TO_TICKS(CONFIG_ARM_HOLD_MS)))
            {
                g_flightState = FLIGHT_STATE_ARMED;
                printf("unlock\r\n");
                AttitudeController_Reset(&g_controller, state);
            }
        }
        else
        {
            *armCmdStartTick = 0;
        }

        if (inputRaw->yaw >= CONFIG_ARM_YAW_HIGH)
        {
            if (*disarmCmdStartTick == 0)
            {
                *disarmCmdStartTick = nowTick;
            }
            else if ((g_flightState == FLIGHT_STATE_ARMED) &&
                     (nowTick - *disarmCmdStartTick >= pdMS_TO_TICKS(CONFIG_DISARM_HOLD_MS)))
            {
                g_flightState = FLIGHT_STATE_DISARMED;
                printf("[FC][STATE] -> DISARMED, reason=RC_DISARM_CMD, lift=%u, yaw=%u, hold=%lu\r\n",
                       inputRaw->lift,
                       inputRaw->yaw,
                       (unsigned long)(nowTick - *disarmCmdStartTick));
            }
        }
        else
        {
            *disarmCmdStartTick = 0;
        }
    }
    else
    {
        *armCmdStartTick = 0;
        *disarmCmdStartTick = 0;
    }
}

/**
 * @brief 应用锁定状态下的控制器复位与安全输出逻辑
 * @param state 当前姿态状态
 * @param altitudeM 当前高度
 * @param yawTargetDeg 当前偏航目标角指针
 */
static void applyDisarmedBehavior(const AttitudeState_t* state, float altitudeM, float* yawTargetDeg)
{
    if (state == 0 || yawTargetDeg == 0)
    {
        return;
    }

    AttitudeController_Reset(&g_controller, state);
    g_altHoldActive = 0;
    g_altitudeSp = altitudeM;
    pidReset(&g_altitudePid, altitudeM);
    *yawTargetDeg = state->yaw;

    g_debug.rollSp = 0.0f;
    g_debug.pitchSp = 0.0f;
    g_debug.yawRateSp = 0.0f;
    g_debug.rollRateSp = 0.0f;
    g_debug.pitchRateSp = 0.0f;
    g_debug.rollOut = 0.0f;
    g_debug.pitchOut = 0.0f;
    g_debug.yawOut = 0.0f;
    g_debug.rollRate = 0.0f;
    g_debug.pitchRate = 0.0f;
    g_debug.yawRate = 0.0f;
}

/**
 * @brief 更新定高模式并生成油门指令
 * @param input 当前滤波后的遥控输入
 * @param altitudeM 当前高度
 * @param pressureValid 气压数据是否有效
 * @param throttleCmd 油门输出指令指针
 */
static void updateAltitudeHoldThrottle(const ControlInput_t* input,
                                       float altitudeM,
                                       uint8_t pressureValid,
                                       uint16_t* throttleCmd)
{
    float liftStick;   /* 升力输入归一化值 */
    float altitudeOut; /* 高度环输出量 */

    if (input == 0 || throttleCmd == 0)
    {
        return;
    }

    liftStick = constrain(((float)input->lift - 1500.0f) / 500.0f, -1.0f, 1.0f);
    altitudeOut = 0.0f;
    *throttleCmd = input->lift;

    if (!CONFIG_GIMBAL_DEBUG_MODE && pressureValid && g_baroRefReady)
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
            *throttleCmd = (uint16_t)constrain((float)input->lift + altitudeOut,
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
    else
    {
        g_altHoldActive = 0;
        g_altitudeSp = altitudeM;
        pidReset(&g_altitudePid, altitudeM);
    }

    *throttleCmd = clampThrottleForDebug(*throttleCmd);
}

/**
 * @brief 根据偏航摇杆更新偏航控制目标（摇杆回中时启用航向保持）
 * @param input 当前滤波后的遥控输入
 * @param state 当前姿态状态
 * @param yawTargetDeg 偏航目标角指针
 * @param sp 姿态设定点
 */
static void updateYawSetpoint(const ControlInput_t* input,
                              const AttitudeState_t* state,
                              float* yawTargetDeg,
                              AttitudeSetpoint_t* sp)
{
    float yawStick; /* 偏航输入归一化值 */
    float yawErr;   /* 偏航角误差 */

    if (input == 0 || state == 0 || yawTargetDeg == 0 || sp == 0)
    {
        return;
    }

    yawStick = constrain(((float)input->yaw - 1500.0f) / 500.0f, -1.0f, 1.0f);
    if (fabsf(yawStick) < 0.05f)
    {
        yawErr = capAngle(*yawTargetDeg - state->yaw);
        sp->yawRate = constrain(yawErr * 4.0f, -CONFIG_MAX_YAW_RATE, CONFIG_MAX_YAW_RATE);
    }
    else
    {
        *yawTargetDeg = state->yaw;
    }
}

/**
 * @brief 根据 IMU 新鲜度施加控制降级与油门限制
 * @param imuFreshness 当前 IMU 新鲜度
 * @param imuFreshnessPrev 上一周期 IMU 新鲜度
 * @param state 当前姿态状态
 * @param throttleCmd 油门输出指令
 * @return 控制量缩放系数
 */
static float applyImuFreshnessControl(ImuFreshness_t imuFreshness,
                                      ImuFreshness_t imuFreshnessPrev,
                                      const AttitudeState_t* state,
                                      uint16_t* throttleCmd)
{
    float imuCtrlScale = 1.0f; /* IMU 降级控制缩放系数 */

    if (state == 0 || throttleCmd == 0)
    {
        return imuCtrlScale;
    }

    if (imuFreshness == IMU_FRESHNESS_STALE)
    {
        imuCtrlScale = IMU_CTRL_SCALE_STALE;
    }
    else if (imuFreshness == IMU_FRESHNESS_PREFAILSAFE)
    {
        imuCtrlScale = IMU_CTRL_SCALE_PREFAILSAFE;
        *throttleCmd = (uint16_t)constrain((float)(*throttleCmd),
                                           (float)CONFIG_MOTOR_IDLE_THROTTLE,
                                           (float)(CONFIG_MOTOR_IDLE_THROTTLE + 80));
    }

    if ((imuFreshness != IMU_FRESHNESS_FRESH) && (imuFreshnessPrev == IMU_FRESHNESS_FRESH))
    {
        /* 进入 IMU 降级状态时重置控制器积分，避免旧误差累计导致突变。 */
        AttitudeController_Reset(&g_controller, state);
    }

    return imuCtrlScale;
}

/** 
 * @brief 飞行控制主任务
 * @param params 任务参数，当前未使用
 */
void FlightControl_TaskImpl(void* params)
{
    (void)params;

    const bool isAcro = (CONFIG_DEFAULT_FLIGHT_MODE == CONFIG_FLIGHT_MODE_RATE);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(FC_DT_MS); /* 内环 500Hz */
    float yawTargetDeg = 0.0f;          /* 当前目标偏航角，单位：度 */
    uint16_t outerLoopCounter = 0;      /* 外环分频计数器 */
    uint8_t updateOuterLoop = 1;        /* 本周期是否更新外环 */

    /* ========== 原始传感器数据接收 ========== */
    struct GYRO_ACCEL_Data imuRaw;      /* 原始 IMU 数据（陀螺仪 + 加速度计），单位：LSB */
    struct MAG_Data magRaw;             /* 原始磁力计数据，单位：LSB */
    struct Pressure_Data pressureRaw;   /* 原始气压计数据，单位：Pa */
    struct PPM_Data ppmRaw;             /* 原始遥控器 PPM 信号数据 */
    struct GYRO_ACCEL_Data imuLast = {{0, 0, 0}, {0, 0, 0}};
    struct MAG_Data magLast = {{0, 0, 0}};
    struct Pressure_Data pressureLast = {0};
    struct PPM_Data ppmLast = {{0}};
    uint8_t imuSeen = 0;                /* IMU 数据是否已接收过标志（0=未接收，1=已接收） */
    uint8_t imuNewSample = 0;           /* IMU 新数据采样标志（每次接收到新数据时置 1） */
    uint8_t magSeen = 0;                /* 磁力计数据是否已接收过标志（0=未接收，1=已接收） */
    uint8_t pressureSeen = 0;           /* 气压计数据是否已接收过标志（0=未接收，1=已接收） */
    uint8_t ppmSeen = 0;                /* PPM 遥控器信号是否已接收过标志（0=未接收，1=已接收） */
    ImuFreshness_t imuFreshness = IMU_FRESHNESS_LOST;  /* IMU 数据新鲜度状态（用于判断数据是否超时/丢失） */
    ImuFreshness_t imuFreshnessPrev = IMU_FRESHNESS_LOST;  /* 上一周期的 IMU 新鲜度状态（用于状态变化检测） */
    uint16_t imuStatCycles = 0;         /* IMU 数据总周期数统计（用于计算丢包率） */
    uint16_t imuStatDrops = 0;          /* IMU 数据丢失次数统计（用于计算丢包率） */
    float imuDropRatePct = 0.0f;        /* IMU 数据丢包率百分比（0.0~100.0%） */
    
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

    g_flightControlTaskHandle = xTaskGetCurrentTaskHandle();
	
    for (;;)
    {
        nowTick = xTaskGetTickCount();
        updateOuterLoop = (outerLoopCounter == 0u) ? 1u : 0u;
        outerLoopCounter++;
        if (outerLoopCounter >= FC_OUTER_LOOP_DIV)
        {
            outerLoopCounter = 0u;
        }

        if (xQueueReceive(QueueMAG, &magRaw, 0) == pdTRUE)
        {            
            magLast = magRaw;
            magSeen = 1;
            latestMagRaw[0] = magRaw.mag[0];
            latestMagRaw[1] = magRaw.mag[1];
            latestMagRaw[2] = magRaw.mag[2];
            magValid = 1;
            lastMagTick = nowTick;
        }
        else if (magSeen)
        {
            /* 队列暂无新样本时沿用上一帧磁力计数据。 */
            magRaw = magLast;
            latestMagRaw[0] = magRaw.mag[0];
            latestMagRaw[1] = magRaw.mag[1];
            latestMagRaw[2] = magRaw.mag[2];
        }
        
        if (xQueueReceive(QueuePressure, &pressureRaw, 0) == pdTRUE)
        {
            pressureLast = pressureRaw;
            pressureSeen = 1;
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
        else if (pressureSeen)
        {
            /* 没有新气压样本时保留上次高度输入，不刷新超时计时。 */
            pressureRaw = pressureLast;
        }

        imuNewSample = 0;
        if (xQueueReceive(QueueGYROACCEL, &imuRaw, 0) == pdTRUE)
        {
            imuLast = imuRaw;
            imuSeen = 1;
            imuNewSample = 1;
            lastImuTick = nowTick;
            if (g_flightState != FLIGHT_STATE_ARMED)
            {
                runBiasCalibrationStep(&imuRaw);
            }
        }
        else if (imuSeen)
        {
            imuRaw = imuLast;
        }

        updateImuFreshnessAndDropStats(imuSeen, imuNewSample, nowTick, lastImuTick,
            &imuFreshness, &imuStatCycles, &imuStatDrops, &imuDropRatePct);

        updateAttitudeStateFromImu(imuSeen, &imuRaw, gyroCorrected, accelCorrected,
             &sensor, magValid, latestMagRaw, imuFreshness, &state);

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
                ppmLast = ppmRaw;
                ppmSeen = 1;
                inputRaw.lateral = invertPpmChannelU16(ppmRaw.ppmCh[0]);
                inputRaw.forward = invertPpmChannelU16(ppmRaw.ppmCh[1]);
                inputRaw.lift = invertPpmChannelU16(ppmRaw.ppmCh[2]);
                inputRaw.yaw = invertPpmChannelU16(ppmRaw.ppmCh[3]);
                lastPpmTick = nowTick;
            }
        }
        else if (ppmSeen)
        {
            /* 遥控队列无新数据时沿用上一帧通道值。 */
            ppmRaw = ppmLast;
            inputRaw.lateral = invertPpmChannelU16(ppmRaw.ppmCh[0]);
            inputRaw.forward = invertPpmChannelU16(ppmRaw.ppmCh[1]);
            inputRaw.lift = invertPpmChannelU16(ppmRaw.ppmCh[2]);
            inputRaw.yaw = invertPpmChannelU16(ppmRaw.ppmCh[3]);
        }

        input.lateral = lowPassChannelU16(input.lateral, inputRaw.lateral, RC_FILTER_ALPHA);
        input.forward = lowPassChannelU16(input.forward, inputRaw.forward, RC_FILTER_ALPHA);
        input.lift = lowPassChannelU16(input.lift, inputRaw.lift, RC_FILTER_ALPHA);
        input.yaw = lowPassChannelU16(input.yaw, inputRaw.yaw, RC_FILTER_ALPHA);

        updateFailsafeState(nowTick, lastPpmTick, lastImuTick, &state);

        updateArmDisarmCommand(&inputRaw, nowTick, &armCmdStartTick, &disarmCmdStartTick, &state);

        input.armed = (g_flightState == FLIGHT_STATE_ARMED);

        if (g_flightState != FLIGHT_STATE_ARMED)
        {
            applyDisarmedBehavior(&state, altitudeM, &yawTargetDeg);
        }
        else
        {
            float imuCtrlScale = 1.0f;

            AttitudeController_GenerateSetpoint(&input, &sp, isAcro);

#if CONFIG_GIMBAL_DEBUG_MODE
            /* 调试保护的倾角限幅仅适用于角度模式。角速度模式的限幅由 CONFIG_MAX_*_RATE 控制。 */
            if (!isAcro)
            {
                sp.roll = constrain(sp.roll,
                                    -g_debugTiltAngleLimitDeg,
                                    g_debugTiltAngleLimitDeg);
                sp.pitch = constrain(sp.pitch,
                                     -g_debugTiltAngleLimitDeg,
                                     g_debugTiltAngleLimitDeg);
            }
#endif

            updateAltitudeHoldThrottle(&input, altitudeM, pressureValid, &throttleCmd);
            updateYawSetpoint(&input, &state, &yawTargetDeg, &sp);
            imuCtrlScale = applyImuFreshnessControl(imuFreshness, imuFreshnessPrev,
                                &state, &throttleCmd);

            sp.roll *= imuCtrlScale;
            sp.pitch *= imuCtrlScale;
            sp.yawRate *= imuCtrlScale;

            AttitudeController_Update(&g_controller, &sp, &state, isAcro, (updateOuterLoop != 0));
            AttitudeController_MixToMotor(&g_controller, throttleCmd, &motor);
            clampMotorOutputForDebug(&motor);
            Motor_WriteOutput(&motor);

            g_debug.rollSp = sp.roll;
            g_debug.pitchSp = sp.pitch;
            g_debug.yawRateSp = sp.yawRate;
            g_debug.rollRateSp = g_controller.rollRateSp;
            g_debug.pitchRateSp = g_controller.pitchRateSp;
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
        g_debug.imuFreshness = (uint8_t)imuFreshness;
        g_debug.imuDropRatePct = imuDropRatePct;
        g_debug.roll = state.roll;
        g_debug.pitch = state.pitch;
        g_debug.yaw = state.yaw;
        g_debug.rollRate = state.rollRate;
        g_debug.pitchRate = state.pitchRate;
        g_debug.yawRate = state.yawRate;
        g_debug.throttle = throttleCmd;

        imuFreshnessPrev = imuFreshness;

        {
            TickType_t elapsedTicks = xTaskGetTickCount() - xLastWakeTime;

            if (elapsedTicks < xPeriod)
            {
                TickType_t waitTicks = xPeriod - elapsedTicks;

                /* 等待窗口内优先响应 IMU 通知，随后仍由 vTaskDelayUntil 保证严格定频。 */
                (void)ulTaskNotifyTake(pdTRUE, waitTicks);
            }

            vTaskDelayUntil(&xLastWakeTime, xPeriod);
        }
    }
}
