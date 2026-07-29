/**
 * @file    track.c
 * @brief   8路循迹小车控制实现 — 移植 F411
 */

#include "track.h"
#include "motor.h"
#include "trace.h"

#include <string.h>

TrackCtrl g_track;

static int32_t clamp_i32(int32_t value, int32_t min, int32_t max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

static int16_t trace_calc_error(uint8_t trace_status, uint8_t *black_count)
{
    /* 对称权重, 中心在 trace3/trace4 之间 */
    static const int16_t weights[8] = {
        3000, 1500, 1000, 250,
        -250, -1000, -1500, -3000};

    int32_t sum = 0;
    uint8_t count = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        if (trace_status & (1U << i))
        {
            sum += weights[i];
            count++;
        }
    }

    if (black_count != NULL)
        *black_count = count;
    if (count == 0)
        return -32768;
    if (count >= 7)
        return 0;

    int16_t raw = (int16_t)(sum / (int32_t)count);
    return raw;
}

void Track_Init(void)
{
    memset(&g_track, 0, sizeof(g_track));
    g_track.state = TRACK_IDLE;
    g_track.base_speed = TRACK_BASE_SPEED_DEFAULT;
    g_track.search_pwm = TRACK_TURN_LIMIT_DEFAULT;
    g_track.turn_limit = TRACK_TURN_LIMIT_DEFAULT;

    PID_Init(&g_track.pid_line,
             TRACK_LINE_KP_DEFAULT, TRACK_LINE_KI_DEFAULT, TRACK_LINE_KD_DEFAULT,
             (float)g_track.turn_limit);
    PID_SetAlpha(&g_track.pid_line, TRACK_LINE_ALPHA_DEFAULT);
}

void Track_Start(void)
{
    g_track.state = TRACK_RUN;
    g_track.last_error = 0;
    PID_Reset(&g_track.pid_line);
}

void Track_Stop(void)
{
    g_track.state = TRACK_IDLE;
    Motor_Stop();
}

void Track_SetBaseSpeed(int32_t base_speed)
{
    g_track.base_speed = clamp_i32(base_speed, 0, PWM_MAX_DUTY);
}

void Track_Process(uint8_t trace_status)
{
    uint8_t black_count = 0;
    int16_t error = trace_calc_error(trace_status, &black_count);
    int32_t turn;

    if (g_track.state == TRACK_IDLE)
    {
        Motor_Stop();
        return;
    }

    if (error == -32768)
    {
        g_track.state = TRACK_LOST;
        PID_Reset(&g_track.pid_line); /* 丢线: 清零积分, 防止重寻时突跳 */
    }
    else
    {
        g_track.state = TRACK_RUN;
    }

    if (g_track.state == TRACK_LOST)
    {
        /* 丢线: 若标志置位则不停止, 交给上层控制 */
        if (!g_track.no_stop_on_lost)
        {
            Motor_Stop();
        }
        return;
    }

    if (g_track.state == TRACK_RUN)
    {
        /* ---- PID 计算基本转向量 ---- */
        g_track.last_error = error;
        turn = (int32_t)PID_Calc(&g_track.pid_line, 0.0f, (float)error);

        /* 差速: turn>0 左慢右快(右转), turn<0 左快右慢(左转) */
        int32_t speed_a = g_track.base_speed - turn;
        int32_t speed_b = g_track.base_speed + turn;

        Motor_SetTargetSpeed(
            clamp_i32(speed_a, -PWM_MAX_DUTY, PWM_MAX_DUTY),
            clamp_i32(speed_b, -PWM_MAX_DUTY, PWM_MAX_DUTY));
        return;
    }
}
