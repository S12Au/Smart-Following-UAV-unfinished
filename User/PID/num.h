/**
 * num.h - 数值处理工具函数
 * 用于PID控制中的数学运算
 */
#ifndef NUM_H_
#define NUM_H_

#include <math.h>
#include <stdint.h>
#include "physicalConstants.h"

/**
 * @brief 约束函数 - 将值限制在指定范围内
 * @param val 输入值
 * @param min 最小值
 * @param max 最大值
 * @return 约束后的值
 */
static inline float constrain(float val, float min, float max)
{
    if (val < min)
    {
        return min;
    }
    else if (val > max)
    {
        return max;
    }
    else
    {
        return val;
    }
}

/**
 * @brief 整型约束函数
 */
static inline int32_t constrainInt(int32_t val, int32_t min, int32_t max)
{
    if (val < min)
    {
        return min;
    }
    else if (val > max)
    {
        return max;
    }
    else
    {
        return val;
    }
}

/**
 * @brief 死区函数
 * @param val 输入值
 * @param deadband 死区大小
 * @return 处理后的值
 */
static inline float deadband(float val, float deadband)
{
    if (fabsf(val) < deadband)
    {
        return 0.0f;
    }
    else if (val > 0)
    {
        return val - deadband;
    }
    else
    {
        return val + deadband;
    }
}

/**
 * @brief 限制角度到 [-180, 180] 范围
 */
static inline float capAngle(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

/**
 * @brief 角度转弧度
 */
static inline float radians(float degrees)
{
    return degrees * (float)M_PI / 180.0f;
}

/**
 * @brief 弧度转角度
 */
static inline float degrees(float radians)
{
    return radians * 180.0f / (float)M_PI;
}

/**
 * @brief 线性插值
 */
static inline float lerp(float a, float b, float t)
{
    return a + t * (b - a);
}

#endif /* NUM_H_ */
