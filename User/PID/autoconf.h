/**
 * autoconf.h - 自动配置文件
 * 定义编译时配置选项
 */
#ifndef AUTOCONF_H_
#define AUTOCONF_H_

/**
 * 控制器配置选项
 */

/* 是否对整个PID输出进行滤波（而非仅D项滤波）
 * 0: 仅对D项滤波
 * 1: 对整个输出滤波
 */
#define CONFIG_CONTROLLER_PID_FILTER_ALL    0

/* 控制器更新频率 (Hz) */
#define CONFIG_CONTROLLER_RATE_HZ           500

/* 姿态外环（角度环）更新频率 (Hz)
 * 设计为低于内环：典型 50-200Hz。当前设置为 100Hz。 */
#define CONFIG_OUTER_LOOP_RATE_HZ           100

#if (CONFIG_OUTER_LOOP_RATE_HZ < 50) || (CONFIG_OUTER_LOOP_RATE_HZ > 200)
#error "CONFIG_OUTER_LOOP_RATE_HZ must be in [50, 200] Hz"
#endif

#if (CONFIG_OUTER_LOOP_RATE_HZ >= CONFIG_CONTROLLER_RATE_HZ)
#error "CONFIG_OUTER_LOOP_RATE_HZ must be lower than CONFIG_CONTROLLER_RATE_HZ"
#endif

#if ((CONFIG_CONTROLLER_RATE_HZ % CONFIG_OUTER_LOOP_RATE_HZ) != 0)
#error "CONFIG_CONTROLLER_RATE_HZ must be an integer multiple of CONFIG_OUTER_LOOP_RATE_HZ"
#endif

/* 姿态解算更新频率 (Hz) */
#define CONFIG_ATTITUDE_RATE_HZ             500

/* 是否使用四元数姿态解算 */
#define CONFIG_USE_QUATERNION_ATTITUDE      1

/* 是否启用抗积分饱和 */
#define CONFIG_ANTI_WINDUP                  1

/* 电机控制参数 */
/* 电机 PWM 输出范围（单位：us）*/
#define CONFIG_MOTOR_MIN_THROTTLE           1000  // 电机最小油门 PWM 脉宽
#define CONFIG_MOTOR_MAX_THROTTLE           2000  // 电机最大油门 PWM 脉宽
#define CONFIG_MOTOR_IDLE_THROTTLE          1100  // 电机怠速 PWM 脉宽（启动后维持转速）

/* 云台调试模式
 * 1: 启用调试保护，限制总油门和最终电机输出，并关闭定高模式
 * 0: 恢复正常飞行逻辑 */
#define CONFIG_GIMBAL_DEBUG_MODE            1
#define CONFIG_GIMBAL_DEBUG_MAX_THROTTLE    1380
#define CONFIG_GIMBAL_DEBUG_MAX_TILT_ANGLE  15.0f

/* 遥控器油门死区（单位：us）*/
#define CONFIG_THROTTLE_MIN                 1050  // 遥控器油门杆最小有效值
#define CONFIG_THROTTLE_MAX                 1950  // 遥控器油门杆最大有效值

/* 飞行模式 */
#define CONFIG_FLIGHT_MODE_ANGLE            0     // 自稳模式标识（角度控制）
#define CONFIG_FLIGHT_MODE_RATE             1     // 手动模式标识（角速度控制）
#define CONFIG_DEFAULT_FLIGHT_MODE          CONFIG_FLIGHT_MODE_ANGLE  // 默认飞行模式

/* PID 参数档位：0=SAFE（安全）, 1=NORMAL（普通）, 2=AGILE（敏捷）*/
#define CONFIG_PID_PROFILE_DEFAULT          1     // 默认 PID 参数配置档位

/* 角度限制（单位：度）*/
#define CONFIG_MAX_ROLL_ANGLE               45.0f // 最大横滚角限制
#define CONFIG_MAX_PITCH_ANGLE              45.0f // 最大俯仰角限制

/* 角速度限制（单位：度/秒）*/
#define CONFIG_MAX_ROLL_RATE                360.0f // 最大横滚角速度
#define CONFIG_MAX_PITCH_RATE               360.0f // 最大俯仰角速度
#define CONFIG_MAX_YAW_RATE                 180.0f // 最大偏航角速度

/* 遥控输入有效范围（单位：us）*/
#define CONFIG_PPM_MIN_VALID                900   // PPM 信号最小有效脉宽
#define CONFIG_PPM_MAX_VALID                2100  // PPM 信号最大有效脉宽

/* 安全逻辑超时时间（单位：毫秒）*/
#define CONFIG_PPM_FAILSAFE_TIMEOUT_MS      300   // 遥控器信号丢失保护超时
#define CONFIG_IMU_FAILSAFE_TIMEOUT_MS      20    // IMU 数据失效超时
#define CONFIG_IMU_STALE_TIMEOUT_MS         4     // IMU 轻度陈旧阈值（进入降级控制）
#define CONFIG_IMU_PREFAILSAFE_TIMEOUT_MS   10    // IMU 重度陈旧阈值（进入预保护）
#define CONFIG_MAG_STALE_TIMEOUT_MS         500   // 磁力计数据过期超时
#define CONFIG_PRESSURE_STALE_TIMEOUT_MS    500   // 气压计数据过期超时
#define CONFIG_ARM_HOLD_MS                  2000  // 解锁命令保持时间
#define CONFIG_DISARM_HOLD_MS               1500   // 上锁命令保持时间
#define CONFIG_ARM_YAW_HIGH                 1900  // 解锁时偏航杆高位阈值
#define CONFIG_DISARM_YAW_LOW               1100  // 上锁时偏航杆低位阈值

/* 串口调试输出频率（单位：Hz）*/
#define CONFIG_UART_DEBUG_RATE_HZ           20    // 调试数据串口发送频率

/* 传感器零偏校准参数 */
#define CONFIG_SENSOR_CALIB_SAMPLES         1000  // 校准采样点数
#define CONFIG_GYRO_STILL_THRESHOLD_DPS     12.0f  // 陀螺仪静止判断阈值（度/秒）
#define CONFIG_ACCEL_STILL_TOL_G            0.15f // 加速度计静止容差（g）

#if (CONFIG_IMU_STALE_TIMEOUT_MS >= CONFIG_IMU_PREFAILSAFE_TIMEOUT_MS)
#error "CONFIG_IMU_STALE_TIMEOUT_MS must be lower than CONFIG_IMU_PREFAILSAFE_TIMEOUT_MS"
#endif

#if (CONFIG_IMU_PREFAILSAFE_TIMEOUT_MS >= CONFIG_IMU_FAILSAFE_TIMEOUT_MS)
#error "CONFIG_IMU_PREFAILSAFE_TIMEOUT_MS must be lower than CONFIG_IMU_FAILSAFE_TIMEOUT_MS"
#endif

#if (CONFIG_GIMBAL_DEBUG_MAX_THROTTLE < CONFIG_MOTOR_IDLE_THROTTLE) || \
	(CONFIG_GIMBAL_DEBUG_MAX_THROTTLE > CONFIG_MOTOR_MAX_THROTTLE)
#error "CONFIG_GIMBAL_DEBUG_MAX_THROTTLE must be within [CONFIG_MOTOR_IDLE_THROTTLE, CONFIG_MOTOR_MAX_THROTTLE]"
#endif

/* 注意：预处理器 #if 仅支持整型常量表达式，浮点范围检查需在 C 代码中执行。 */
#define CONFIG_GIMBAL_DEBUG_TILT_ANGLE_VALID(tilt_deg) \
    (((tilt_deg) > 0.0f) && \
     ((tilt_deg) <= CONFIG_MAX_ROLL_ANGLE) && \
     ((tilt_deg) <= CONFIG_MAX_PITCH_ANGLE))

#endif /* AUTOCONF_H_ */
