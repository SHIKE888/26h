/**
 * @file    track.h
 * @brief   8路循迹小车控制接口 — 移植 F411
 */

#ifndef CONTROL_TRACK_H_
#define CONTROL_TRACK_H_

#include "main.h"
#include "pid.h"

/* ======================== 循迹调参宏 (快速调整) ======================== */
/* F1 纯循迹参数 */
#define TRACK_BASE_SPEED_DEFAULT 1000 /* 默认基础速度 (编码器原始值) */
#define TRACK_TURN_LIMIT_DEFAULT 1500 /* 转弯差速上限 */
#define TRACK_LINE_KP_DEFAULT 0.08f   // 速度600用0.05，速度1000用0.08
#define TRACK_LINE_KI_DEFAULT 0.0006f
#define TRACK_LINE_KD_DEFAULT 1.0f

/* F2~F5 双控模式共用参数 (速度更低以减少对平衡球的冲击) */
#define TRACK_BASE_SPEED_DUAL 600  /* 双控基础速度 */
#define TRACK_TURN_LIMIT_DUAL 1200 /* 转弯差速上限 */
#define TRACK_LINE_KP_DUAL 0.06f   /* 循迹 Kp */
#define TRACK_LINE_KI_DUAL 0.0005f
#define TRACK_LINE_KD_DUAL 0.8f

/* 缓启动参数 (双控模式) */
#define TRACK_RAMP_STEP 3  /* 每 10ms 步长 (600/2000*10=3) */
#define TRACK_RAMP_MS 2000 /* 缓启动时间 (ms), 缓减速亦同 */

/* 双控时限宏 */
#define DUAL_TIMEOUT_8S 800   /* 8秒 (单位: 10ms) */
#define DUAL_TIMEOUT_30S 3000 /* 30秒 (单位: 10ms) */

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
