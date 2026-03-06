#include "AttitudeEstimator.h"
#include "physicalConstants.h"
#include "num.h"
#include <math.h>
#include <string.h>

/**
 * @brief 单位化四元数
 * @param q 四元数指针
 */
static void Quaternion_Normalize(Quaternion_t* q)
{
    float norm = sqrtf(q->q0 * q->q0 + q->q1 * q->q1 + q->q2 * q->q2 + q->q3 * q->q3);
    if (norm > 0.0f)
    {
        float invNorm = 1.0f / norm;
        q->q0 *= invNorm;
        q->q1 *= invNorm;
        q->q2 *= invNorm;
        q->q3 *= invNorm;
    }
    else
    {
        q->q0 = 1.0f;
        q->q1 = 0.0f;
        q->q2 = 0.0f;
        q->q3 = 0.0f;
    }
}

static void Quaternion_ToEuler(const Quaternion_t* q, EulerAngle_t* euler)
{
    float sinr_cosp = 2.0f * (q->q0 * q->q1 + q->q2 * q->q3);
    float cosr_cosp = 1.0f - 2.0f * (q->q1 * q->q1 + q->q2 * q->q2);
    euler->roll = atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;

    float sinp = 2.0f * (q->q0 * q->q2 - q->q3 * q->q1);
    if (fabsf(sinp) >= 1.0f)
    {
        euler->pitch = copysignf(90.0f, sinp);
    }
    else
    {
        euler->pitch = asinf(sinp) * RAD_TO_DEG;
    }

    float siny_cosp = 2.0f * (q->q0 * q->q3 + q->q1 * q->q2);
    float cosy_cosp = 1.0f - 2.0f * (q->q2 * q->q2 + q->q3 * q->q3);
    euler->yaw = atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;

    euler->yaw = capAngle(euler->yaw);
    euler->roll = capAngle(euler->roll);
}

static float AngleWrapDeg(float angleDeg)
{
    while (angleDeg > 180.0f)
    {
        angleDeg -= 360.0f;
    }
    while (angleDeg < -180.0f)
    {
        angleDeg += 360.0f;
    }
    return angleDeg;
}

/*
函数名称：ApplyMagYawCorrection
函数功能：根据磁力计数据修正姿态估计器的偏航角
输入参数：
  - estimator: 指向AttitudeEstimator_t结构体的指针，表示姿态估计器
  - sensor: 指向SensorData_t结构体的指针，包含原始传感器数据
  
返回值：无
*/
static void ApplyMagYawCorrection(AttitudeEstimator_t* estimator, const SensorData_t* sensor)
{
    const float mx = sensor->mag[0];
    const float my = sensor->mag[1];
    const float mz = sensor->mag[2];
    const float magNorm = sqrtf(mx * mx + my * my + mz * mz);
    const float alpha = 0.02f;

    if (magNorm < 1e-6f)
    {
        return;
    }

    const float rollRad = estimator->euler.roll * DEG_TO_RAD;
    const float pitchRad = estimator->euler.pitch * DEG_TO_RAD;
    const float cosRoll = cosf(rollRad);
    const float sinRoll = sinf(rollRad);
    const float cosPitch = cosf(pitchRad);
    const float sinPitch = sinf(pitchRad);

    const float mxComp = mx * cosPitch + mz * sinPitch;
    const float myComp = mx * sinRoll * sinPitch + my * cosRoll - mz * sinRoll * cosPitch;

    if ((fabsf(mxComp) + fabsf(myComp)) < 1e-6f)
    {
        return;
    }

    const float magYawDeg = atan2f(-myComp, mxComp) * RAD_TO_DEG;
/**
 * @brief 姿态估计器初始化
 * @param estimator 姿态估计器指针
 * @param dt 控制周期（秒）
 * @param kp 比例增益
 * @param ki 积分增益
 * @param useMag 是否使用磁力计
 */
    const float yawErrDeg = AngleWrapDeg(magYawDeg - estimator->euler.yaw);

    estimator->euler.yaw = AngleWrapDeg(estimator->euler.yaw + alpha * yawErrDeg);
}

void AttitudeEstimator_Init(AttitudeEstimator_t* estimator, float dt,
                            float kp, float ki, bool useMag)
{
    if (estimator == NULL)
    {
        return;
    }

    memset(estimator, 0, sizeof(AttitudeEstimator_t));

    estimator->q.q0 = 1.0f;
    estimator->q.q1 = 0.0f;
    estimator->q.q2 = 0.0f;
    estimator->q.q3 = 0.0f;

    estimator->euler.roll = 0.0f;
    estimator->euler.pitch = 0.0f;
    estimator->euler.yaw = 0.0f;

    estimator->dt = dt;
    estimator->twoKp = 2.0f * kp;
/**
 * @brief 用IMU数据更新姿态估计器
 * @param estimator 姿态估计器指针
 * @param gx, gy, gz 陀螺仪角速度（rad/s）
 * @param ax, ay, az 加速度计（m/s^2）
 */
    estimator->twoKi = 2.0f * ki;
    estimator->useMag = useMag;
    estimator->initialized = true;
}

void AttitudeEstimator_UpdateIMU(AttitudeEstimator_t* estimator,
                                  float gx, float gy, float gz,
                                  float ax, float ay, float az)
{
    if (estimator == NULL || !estimator->initialized)
    {
        return;
    }

    float recipNorm;
    // halfvx/halfvy/halfvz: 当前四元数推算出的重力方向（机体坐标系下，单位向量）
    float halfvx, halfvy, halfvz;
    // halfex/halfey/halfez: 加速度计测量与理论重力方向的误差（叉乘结果，反馈修正用）
    float halfex, halfey, halfez;
    
    float qa, qb, qc;

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        halfvx = estimator->q.q1 * estimator->q.q3 - estimator->q.q0 * estimator->q.q2;
        halfvy = estimator->q.q0 * estimator->q.q1 + estimator->q.q2 * estimator->q.q3;
        halfvz = estimator->q.q0 * estimator->q.q0 - 0.5f + estimator->q.q3 * estimator->q.q3;

        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        if (estimator->twoKi > 0.0f)
        {
            estimator->integralFBx += estimator->twoKi * halfex * estimator->dt;
            estimator->integralFBy += estimator->twoKi * halfey * estimator->dt;
            estimator->integralFBz += estimator->twoKi * halfez * estimator->dt;

            gx += estimator->integralFBx;
            gy += estimator->integralFBy;
            gz += estimator->integralFBz;
        }
        else
        {
            estimator->integralFBx = 0.0f;
            estimator->integralFBy = 0.0f;
            estimator->integralFBz = 0.0f;
        }

        gx += estimator->twoKp * halfex;
        gy += estimator->twoKp * halfey;
        gz += estimator->twoKp * halfez;
    }

    gx *= 0.5f * estimator->dt;
    gy *= 0.5f * estimator->dt;
    gz *= 0.5f * estimator->dt;

    qa = estimator->q.q0;
    qb = estimator->q.q1;
    qc = estimator->q.q2;

    estimator->q.q0 += (-qb * gx - qc * gy - estimator->q.q3 * gz);
    estimator->q.q1 += (qa * gx + qc * gz - estimator->q.q3 * gy);
    estimator->q.q2 += (qa * gy - qb * gz + estimator->q.q3 * gx);
    estimator->q.q3 += (qa * gz + qb * gy - qc * gx);
/**
 * @brief 用传感器数据更新姿态估计器
 * @param estimator 姿态估计器指针
 * @param sensor 传感器数据指针
 */

    Quaternion_Normalize(&estimator->q);
    Quaternion_ToEuler(&estimator->q, &estimator->euler);
}

void AttitudeEstimator_Update(AttitudeEstimator_t* estimator,
                              const SensorData_t* sensor)
{
    if (estimator == NULL || sensor == NULL)
    {
        return;
    }

    AttitudeEstimator_UpdateIMU(estimator,
                                sensor->gyro[0], sensor->gyro[1], sensor->gyro[2],
                                sensor->accel[0], sensor->accel[1], sensor->accel[2]);

    if (estimator->useMag)
/**
 * @brief 获取当前欧拉角
 * @param estimator 姿态估计器指针
 * @return 欧拉角结构体指针
 */
    {
        ApplyMagYawCorrection(estimator, sensor);
    }
}

const EulerAngle_t* AttitudeEstimator_GetEuler(AttitudeEstimator_t* estimator)
{
    if (estimator == NULL)
    {
/**
 * @brief 获取当前四元数
 * @param estimator 姿态估计器指针
 * @return 四元数结构体指针
 */
        return NULL;
    }
    return &estimator->euler;
}

const Quaternion_t* AttitudeEstimator_GetQuaternion(AttitudeEstimator_t* estimator)
{
    if (estimator == NULL)
    {
/**
 * @brief 重置姿态估计器
 * @param estimator 姿态估计器指针
 */
        return NULL;
    }
    return &estimator->q;
}

void AttitudeEstimator_Reset(AttitudeEstimator_t* estimator)
{
    if (estimator == NULL)
    {
        return;
    }

    estimator->q.q0 = 1.0f;
    estimator->q.q1 = 0.0f;
    estimator->q.q2 = 0.0f;
    estimator->q.q3 = 0.0f;

    estimator->euler.roll = 0.0f;
    estimator->euler.pitch = 0.0f;
    estimator->euler.yaw = 0.0f;

/**
 * @brief 设置姿态估计器增益
 * @param estimator 姿态估计器指针
 * @param kp 比例增益
 * @param ki 积分增益
 */
    estimator->integralFBx = 0.0f;
    estimator->integralFBy = 0.0f;
    estimator->integralFBz = 0.0f;
}

void AttitudeEstimator_SetGains(AttitudeEstimator_t* estimator, float kp, float ki)
{
    if (estimator == NULL)
    {
        return;
    }

    estimator->twoKp = 2.0f * kp;
    estimator->twoKi = 2.0f * ki;
}

/*
函数名称：SensorData_ConvertFromRaw
函数功能：将原始的陀螺仪和加速度计数据转换为弧度制和米每二次方秒，并将磁力计数据初始化为0
输入参数：
  - rawGyro: 包含原始陀螺仪数据的数组，单位为LSB
  - rawAccel: 包含原始加速度计数据的数组，单位为LSB
  - sensor: 指向SensorData_t结构体的指针，用于存储转换后的传感器数据

返回值：无

*/
void SensorData_ConvertFromRaw(const int16_t rawGyro[3],
                               const int16_t rawAccel[3],
                               SensorData_t* sensor)
{
    if (rawGyro == NULL || rawAccel == NULL || sensor == NULL)
    {
        return;
    }

    sensor->gyro[0] = ((float)rawGyro[0] / GYRO_SENSITIVITY_2000) * DEG_TO_RAD;
    sensor->gyro[1] = ((float)rawGyro[1] / GYRO_SENSITIVITY_2000) * DEG_TO_RAD;
    sensor->gyro[2] = ((float)rawGyro[2] / GYRO_SENSITIVITY_2000) * DEG_TO_RAD;

    sensor->accel[0] = ((float)rawAccel[0] / ACCEL_SENSITIVITY_2G) * GRAVITY_MAGNITUDE;
    sensor->accel[1] = ((float)rawAccel[1] / ACCEL_SENSITIVITY_2G) * GRAVITY_MAGNITUDE;
    sensor->accel[2] = ((float)rawAccel[2] / ACCEL_SENSITIVITY_2G) * GRAVITY_MAGNITUDE;

    sensor->mag[0] = 0.0f;
    sensor->mag[1] = 0.0f;
    sensor->mag[2] = 0.0f;
}
