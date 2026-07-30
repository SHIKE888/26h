/**
 * @file    track.h
 * @brief   8路循迹小车控制接口 — 移植 F411
 */

#ifndef CONTROL_TRACK_H_
#define CONTROL_TRACK_H_

#include "main.h"
#include "pid.h"

/* ======================== 循迹调参宏 (快速调整) ======================== */
#define TRACK_BASE_SPEED_DEFAULT 1000  /* 默认基础速度 (编码器原始值) */
#define TRACK_TURN_LIMIT_DEFAULT 1500 /* 转弯差速上限 */
#define TRACK_LINE_KP_DEFAULT 0.08f
#define TRACK_LINE_KI_DEFAULT 0.0006f
#define TRACK_LINE_KD_DEFAULT 1.0f
#define TRACK_LINE_ALPHA_DEFAULT 0.0f

typedef enum
{
    TRACK_IDLE = 0,
    TRACK_RUN = 1,
    TRACK_LOST = 2,
} TrackState;

typedef struct
{
    TrackState state;
    PID_t pid_line;
    int32_t base_speed;
    int32_t search_pwm;
    int32_t turn_limit;
    int16_t last_error;
    uint8_t no_stop_on_lost;
} TrackCtrl;

extern TrackCtrl g_track;

void Track_Init(void);
void Track_Start(void);
void Track_Stop(void);
void Track_SetBaseSpeed(int32_t base_speed);
void Track_Process(uint8_t trace_status);

#endif
