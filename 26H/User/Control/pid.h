/**
 * @file    pid.h
 * @brief   增量式/位置式 PID 控制器 — 移植 F411
 */

#ifndef CONTROL_PID_H_
#define CONTROL_PID_H_

#include "main.h"

/* ========================== PID 参数 ================================= */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float out_max;
    float integral;
    float last_err;
    float last_output; /* 上一次输出, 用于增量平滑 */
    float alpha;       /* 输出平滑系数 (0~1, 0=不滤波) */
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_max);
void PID_SetAlpha(PID_t *pid, float alpha);
float PID_Calc(PID_t *pid, float target, float current);
void PID_Reset(PID_t *pid);

#endif
