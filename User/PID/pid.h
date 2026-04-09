/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * pid.h - implementation of the PID regulator
 */
#ifndef PID_H_
#define PID_H_

#include <stdbool.h>
#include "filter.h"

#define DEFAULT_PID_INTEGRATION_LIMIT 5000.0
#define DEFAULT_PID_OUTPUT_LIMIT      0.0

/** 
 * @brief PID对象结构体
 * 包含PID算法所需的所有参数和状态变量
 */
typedef struct
{
  float desired;      // 期望值（设定值）
  float error;        // 误差（期望值 - 测量值）
  float prevMeasured; // 上次测量值
  float integ;        // 积分项累加值
  float deriv;        // 微分项值
  float kp;           // 比例增益系数
  float ki;           // 积分增益系数
  float kd;           // 微分增益系数
  float kff;          // 前馈增益系数
  float outP;         // 比例项输出（调试用）
  float outI;         // 积分项输出（调试用）
  float outD;         // 微分项输出（调试用）
  float outFF;        // 前馈项输出（调试用）
  float iLimit;       // 积分限幅值（绝对值），0 表示无限幅
  float outputLimit;  // PID 总输出限幅值（绝对值），0 表示无限幅
  float dt;           // 采样时间间隔（秒）
  float errorDeadzone;// 误差死区阈值（绝对值），0 表示禁用
  lpf2pData dFilter;  // 微分项二阶低通滤波器
  bool enableDFilter; // 微分项滤波器使能标志
  bool enableErrorDeadzone; // 误差死区使能
} PidObject;

/**
 * PID object initialization.
 *
 * @param[out] pid   A pointer to the pid object to initialize.
 * @param[in] desired  The initial set point.
 * @param[in] kp        The proportional gain
 * @param[in] ki        The integral gain
 * @param[in] kd        The derivative gain
 * @param[in] kff       The feedforward gain
 * @param[in] dt        Delta time since the last call
 * @param[in] samplingRate Frequency the update will be called
 * @param[in] cutoffFreq   Frequency to set the low pass filter cutoff at
 * @param[in] enableDFilter Enable setting for the D lowpass filter
 */
 void pidInit(PidObject* pid, const float desired, const float kp,
              const float ki, const float kd, const float kff, const float dt,
              const float samplingRate, const float cutoffFreq,
              bool enableDFilter);

/**
 * Set the integral limit for this PID in deg.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] limit Pid integral swing limit.
 */
void pidSetIntegralLimit(PidObject* pid, const float limit);

/**
 * Reset the PID error values
 *
 * @param[in] pid    A pointer to the pid object.
 * @param[in] initial Initial value of the controlled state.
 */
void pidReset(PidObject* pid, float initial);

/**
 * Update the PID parameters.
 *
 * @param[in] pid         A pointer to the pid object.
 * @param[in] measured    The measured value
 * @param[in] isYawAngle  Set to TRUE if it is a PID on yaw angle, set to false otherwise
 * @return PID algorithm output
 */
float pidUpdate(PidObject* pid, const float measured, const bool isYawAngle);

/**
 * @brief 设置PID调试输出目标对象；仅当pidUpdate处理该对象时输出调试日志。
 * @param pid 目标PID对象指针，传NULL表示不过滤（输出全部）。
 */
void pidSetDebugTarget(const PidObject* pid);

/**
 * Set a new set point for the PID to track.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] desired The new set point
 */
void pidSetDesired(PidObject* pid, const float desired);

/**
 * Set a new set point for the PID to track.
 * @return The set point
 */
float pidGetDesired(PidObject* pid);

/**
 * Find out if PID is active
 * @return TRUE if active, FALSE otherwise
 */
bool pidIsActive(PidObject* pid);

/**
 * Set a new proportional gain for the PID.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] kp    The new proportional gain
 */
void pidSetKp(PidObject* pid, const float kp);

/**
 * Set a new integral gain for the PID.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] ki    The new integral gain
 */
void pidSetKi(PidObject* pid, const float ki);

/**
 * Set a new derivative gain for the PID.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] kd    The derivative gain
 */
void pidSetKd(PidObject* pid, const float kd);

/**
 * Set a new feed-froward gain for the PID.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] kff    The new proportional gain
 */
void pidSetKff(PidObject* pid, const float kff);

/**
 * Set a new dt gain for the PID. Defaults to IMU_UPDATE_DT upon construction
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] dt    Delta time
 */
void pidSetDt(PidObject* pid, const float dt);

/**
 * Reset the Dfilter and set cutoff frequency
 *
 * @param[out] pid   A pointer to the pid object to initialize.
 * @param[in] samplingRate Frequency the update will be called
 * @param[in] cutoffFreq   Frequency to set the low pass filter cutoff at
 * @param[in] enableDFilter Enable setting for the D lowpass filter
*/
void filterReset(PidObject* pid, const float samplingRate, const float cutoffFreq, bool enableDFilter);

#endif /* PID_H_ */
