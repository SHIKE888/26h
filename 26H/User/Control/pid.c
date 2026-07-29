/**
 * @file    pid.c
 * @brief   增量式 PID 控制器实现 — 移植 F411
 */

#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_max = out_max;
    pid->integral = 0.0f;
    pid->last_err = 0.0f;
    pid->last_output = 0.0f;
    pid->alpha = 0.0f; /* 默认不平滑 */
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->last_err = 0.0f;
    pid->last_output = 0.0f;
}

void PID_SetAlpha(PID_t *pid, float alpha)
{
    if (alpha < 0.0f)
        alpha = 0.0f;
    if (alpha > 1.0f)
        alpha = 1.0f;
    pid->alpha = alpha;
}

float PID_Calc(PID_t *pid, float target, float current)
{
    float err = target - current;
    float output;

    /* 位置式 PID */
    output = pid->kp * err + pid->ki * pid->integral + pid->kd * (err - pid->last_err);

    /* 输出限幅 */
    if (output > pid->out_max)
        output = pid->out_max;
    else if (output < -pid->out_max)
        output = -pid->out_max;

    /* 积分抗饱和: 仅当输出未饱和时累积积分 */
    if (pid->ki > 0.0f)
    {
        if ((output > -pid->out_max && output < pid->out_max) || (err > 0 && output <= -pid->out_max) || (err < 0 && output >= pid->out_max))
        {
            /* 输出在范围内, 或者误差有利于退出饱和 → 正常积分 */
            pid->integral += err;
        }
        /* 否则: 输出已饱和且误差同向 → 冻结积分, 不累积 */

        /* 积分值限幅 */
        float limit = pid->out_max / pid->ki;
        if (pid->integral > limit)
            pid->integral = limit;
        if (pid->integral < -limit)
            pid->integral = -limit;
    }
    else
    {
        pid->integral = 0.0f;
    }

    /* 输出平滑 */
    if (pid->alpha > 0.0f)
    {
        output = pid->alpha * output + (1.0f - pid->alpha) * pid->last_output;
    }
    pid->last_output = output;

    pid->last_err = err;
    return output;
}
