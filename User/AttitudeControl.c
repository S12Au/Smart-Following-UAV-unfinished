#include "AttitudeControl.h"
#include "autoconf.h"
#include "num.h"
#include "physicalConstants.h"
#include "tim.h"

#define RC_MID              1500.0f
#define RC_SCALE            500.0f
#define MOTOR_MIN           CONFIG_MOTOR_MIN_THROTTLE
#define MOTOR_MAX           CONFIG_MOTOR_MAX_THROTTLE
#define MOTOR_IDLE          CONFIG_MOTOR_IDLE_THROTTLE

typedef struct
{
    float angleKp;
    float angleKi;
    float angleKd;

    float rateKp;
    float rateKi;
    float rateKd;

    float yawRateKp;
    float yawRateKi;
    float yawRateKd;
} PidProfileParam_t;

static const PidProfileParam_t g_pidProfiles[] = {
    /* PID_PROFILE_SAFE */
    {4.0f, 0.03f, 0.15f, 0.14f, 0.08f, 0.0015f, 0.20f, 0.10f, 0.0f},
    /* PID_PROFILE_NORMAL */
    {5.5f, 0.05f, 0.20f, 0.18f, 0.10f, 0.0020f, 0.25f, 0.15f, 0.0f},
    /* PID_PROFILE_AGILE */
    {6.5f, 0.06f, 0.25f, 0.24f, 0.12f, 0.0025f, 0.30f, 0.18f, 0.0f},
};

static inline float rcNormalize(uint16_t rc)
{
    return constrain(((float)rc - RC_MID) / RC_SCALE, -1.0f, 1.0f);
}

void AttitudeController_Init(AttitudeController_t* ctrl, float dt)
{
    const float outerDt = 1.0f / (float)CONFIG_OUTER_LOOP_RATE_HZ;

    if (ctrl == NULL)
    {
        return;
    }

    ctrl->dt = dt;

        /* 角度外环按较低频率运行，避免将高频IMU噪声直接放大到角速度设定。 */
        pidInit(&ctrl->rollAnglePid, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            outerDt, (float)CONFIG_OUTER_LOOP_RATE_HZ, 30.0f, true);
    pidSetIntegralLimit(&ctrl->rollAnglePid, 20.0f);

        pidInit(&ctrl->pitchAnglePid, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            outerDt, (float)CONFIG_OUTER_LOOP_RATE_HZ, 30.0f, true);
    pidSetIntegralLimit(&ctrl->pitchAnglePid, 20.0f);

        pidInit(&ctrl->rollRatePid, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            dt, 1.0f / dt, 60.0f, true);
    pidSetIntegralLimit(&ctrl->rollRatePid, 200.0f);
    ctrl->rollRatePid.outputLimit = 350.0f;

        pidInit(&ctrl->pitchRatePid, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            dt, 1.0f / dt, 60.0f, true);
    pidSetIntegralLimit(&ctrl->pitchRatePid, 200.0f);
    ctrl->pitchRatePid.outputLimit = 350.0f;

        pidInit(&ctrl->yawRatePid, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
            dt, 1.0f / dt, 30.0f, true);
    pidSetIntegralLimit(&ctrl->yawRatePid, 100.0f);
    ctrl->yawRatePid.outputLimit = 200.0f;

        AttitudeController_LoadProfile(ctrl, (PidProfile_t)CONFIG_PID_PROFILE_DEFAULT);

    ctrl->rollRateSp = 0.0f;
    ctrl->pitchRateSp = 0.0f;
    ctrl->yawRateSp = 0.0f;

    ctrl->rollOut = 0.0f;
    ctrl->pitchOut = 0.0f;
    ctrl->yawOut = 0.0f;
}

void AttitudeController_LoadProfile(AttitudeController_t* ctrl, PidProfile_t profile)
{
    const PidProfileParam_t* p;

    if (ctrl == NULL)
    {
        return;
    }

    if ((int)profile < 0 || profile > PID_PROFILE_AGILE)
    {
        profile = PID_PROFILE_NORMAL;
    }

    p = &g_pidProfiles[(int)profile];

    pidSetKp(&ctrl->rollAnglePid, p->angleKp);
    pidSetKi(&ctrl->rollAnglePid, p->angleKi);
    pidSetKd(&ctrl->rollAnglePid, p->angleKd);

    pidSetKp(&ctrl->pitchAnglePid, p->angleKp);
    pidSetKi(&ctrl->pitchAnglePid, p->angleKi);
    pidSetKd(&ctrl->pitchAnglePid, p->angleKd);

    pidSetKp(&ctrl->rollRatePid, p->rateKp);
    pidSetKi(&ctrl->rollRatePid, p->rateKi);
    pidSetKd(&ctrl->rollRatePid, p->rateKd);

    pidSetKp(&ctrl->pitchRatePid, p->rateKp);
    pidSetKi(&ctrl->pitchRatePid, p->rateKi);
    pidSetKd(&ctrl->pitchRatePid, p->rateKd);

    pidSetKp(&ctrl->yawRatePid, p->yawRateKp);
    pidSetKi(&ctrl->yawRatePid, p->yawRateKi);
    pidSetKd(&ctrl->yawRatePid, p->yawRateKd);
}

void AttitudeController_Reset(AttitudeController_t* ctrl, const AttitudeState_t* state)
{
    if (ctrl == NULL || state == NULL)
    {
        return;
    }

    pidReset(&ctrl->rollAnglePid, state->roll);
    pidReset(&ctrl->pitchAnglePid, state->pitch);
    pidReset(&ctrl->rollRatePid, state->rollRate);
    pidReset(&ctrl->pitchRatePid, state->pitchRate);
    pidReset(&ctrl->yawRatePid, state->yawRate);

    ctrl->rollOut = 0.0f;
    ctrl->pitchOut = 0.0f;
    ctrl->yawOut = 0.0f;
}

void AttitudeController_GenerateSetpoint(const ControlInput_t* input, AttitudeSetpoint_t* sp)
{
    if (input == NULL || sp == NULL)
    {
        return;
    }

    float rollIn = rcNormalize(input->lateral);
    float pitchIn = rcNormalize(input->forward);
    float yawIn = rcNormalize(input->yaw);

    sp->roll = rollIn * CONFIG_MAX_ROLL_ANGLE;
    sp->pitch = -pitchIn * CONFIG_MAX_PITCH_ANGLE;
    sp->yawRate = yawIn * CONFIG_MAX_YAW_RATE;
}

void AttitudeController_Update(AttitudeController_t* ctrl,
                               const AttitudeSetpoint_t* sp,
                               const AttitudeState_t* state,
                               bool isAcro,
                               bool updateOuterLoop)
{
    if (ctrl == NULL || sp == NULL || state == NULL)
    {
        return;
    }

    if (isAcro)
    {
        ctrl->rollRateSp = constrain(sp->roll, -CONFIG_MAX_ROLL_RATE, CONFIG_MAX_ROLL_RATE);
        ctrl->pitchRateSp = constrain(sp->pitch, -CONFIG_MAX_PITCH_RATE, CONFIG_MAX_PITCH_RATE);
    }
    else
    {
        if (updateOuterLoop)
        {
            pidSetDesired(&ctrl->rollAnglePid, sp->roll);
            pidSetDesired(&ctrl->pitchAnglePid, sp->pitch);

            ctrl->rollRateSp = pidUpdate(&ctrl->rollAnglePid, state->roll, false);
            ctrl->pitchRateSp = pidUpdate(&ctrl->pitchAnglePid, state->pitch, false);

            ctrl->rollRateSp = constrain(ctrl->rollRateSp, -CONFIG_MAX_ROLL_RATE, CONFIG_MAX_ROLL_RATE);
            ctrl->pitchRateSp = constrain(ctrl->pitchRateSp, -CONFIG_MAX_PITCH_RATE, CONFIG_MAX_PITCH_RATE);
        }
    }

    ctrl->yawRateSp = constrain(sp->yawRate, -CONFIG_MAX_YAW_RATE, CONFIG_MAX_YAW_RATE);

    pidSetDesired(&ctrl->rollRatePid, ctrl->rollRateSp);
    pidSetDesired(&ctrl->pitchRatePid, ctrl->pitchRateSp);
    pidSetDesired(&ctrl->yawRatePid, ctrl->yawRateSp);

    ctrl->rollOut = pidUpdate(&ctrl->rollRatePid, state->rollRate, false);
    ctrl->pitchOut = pidUpdate(&ctrl->pitchRatePid, state->pitchRate, false);
    ctrl->yawOut = pidUpdate(&ctrl->yawRatePid, state->yawRate, false);
}

void AttitudeController_MixToMotor(const AttitudeController_t* ctrl,
                                   uint16_t throttle,
                                   MotorOutput_t* motor)
{
    if (ctrl == NULL || motor == NULL)
    {
        return;
    }

    float t = (float)throttle;

    float m1 = t + ctrl->rollOut + ctrl->pitchOut - ctrl->yawOut;
    float m2 = t - ctrl->rollOut + ctrl->pitchOut + ctrl->yawOut;
    float m3 = t - ctrl->rollOut - ctrl->pitchOut - ctrl->yawOut;
    float m4 = t + ctrl->rollOut - ctrl->pitchOut + ctrl->yawOut;

    if (throttle < CONFIG_THROTTLE_MIN)
    {
        m1 = MOTOR_MIN;
        m2 = MOTOR_MIN;
        m3 = MOTOR_MIN;
        m4 = MOTOR_MIN;
    }
    else
    {
        m1 = constrain(m1, MOTOR_IDLE, MOTOR_MAX);
        m2 = constrain(m2, MOTOR_IDLE, MOTOR_MAX);
        m3 = constrain(m3, MOTOR_IDLE, MOTOR_MAX);
        m4 = constrain(m4, MOTOR_IDLE, MOTOR_MAX);
    }

    motor->m1 = (uint16_t)m1;
    motor->m2 = (uint16_t)m2;
    motor->m3 = (uint16_t)m3;
    motor->m4 = (uint16_t)m4;
}

void Motor_WriteOutput(const MotorOutput_t* motor)
{
    if (motor == NULL)
    {
        return;
    }

    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, motor->m1);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, motor->m2);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, motor->m3);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, motor->m4);
}
