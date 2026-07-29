/**
 * @file    encoder.h
 * @brief   500线 GMR 编码器测速 (带防溢出处理) — 移植 F411
 * @note    电机A(左): TIM3_CH1/CH2 (PB4/PB5)
 *          电机B(右): TIM4_CH1/CH2 (PB6/PB7)
 *          编码器模式: TIM_ENCODERMODE_TI12 (4倍频, 500线=2000脉冲/转)
 */

#ifndef ENCODER_ENCODER_H_
#define ENCODER_ENCODER_H_

#include "main.h"

/* ========================== 编码器参数 =============================== */
#define ENCODER_LINES 500       /* 500线 GMR 编码器 */
#define ENCODER_RESOLUTION 2000 /* TI12模式: 4倍频, 500线×4=2000脉冲/转 */

/* ====================== 编码器极性反转宏 ============================ */
/* 若电机正转时编码器读数为负, 将对应宏设为 1                          */
#define ENCODER_A_INVERT 0
#define ENCODER_B_INVERT 0

/* ====================== 速度低通滤波 ================================ */
/* 滤波系数: 0=无滤波, 越接近1滤波越强 (建议 0.0~0.3)                 */
#define ENCODER_SPEED_ALPHA 0.2f

/* ========================== 数据结构 ================================= */

typedef struct
{
    int32_t pulse;     /* 累计脉冲数 (32位) */
    int32_t speed_raw; /* 本次原始脉冲 (未滤波) */
    int32_t speed;     /* 滤波后速度值 (供PID使用) */
    float speed_f;     /* 滤波中间值 */
} Encoder_TypeDef;

/* ========================== 全局变量 ================================= */
extern Encoder_TypeDef g_encoder_A;
extern Encoder_TypeDef g_encoder_B;

/* ========================== 函数声明 ================================= */

void Encoder_Init(void);
void Encoder_Update(uint32_t dt_ms);
void Encoder_Reset(void);

/**
 * @brief 获取电机A编码器累计脉冲数
 */
static inline int32_t Encoder_GetPulseA(void) { return g_encoder_A.pulse; }

/**
 * @brief 获取电机B编码器累计脉冲数
 */
static inline int32_t Encoder_GetPulseB(void) { return g_encoder_B.pulse; }

/**
 * @brief 获取电机A瞬时速度 (原始脉冲值)
 */
static inline int32_t Encoder_GetSpeedA(void) { return g_encoder_A.speed; }

/**
 * @brief 获取电机B瞬时速度 (原始脉冲值)
 */
static inline int32_t Encoder_GetSpeedB(void) { return g_encoder_B.speed; }

#endif /* ENCODER_ENCODER_H_ */
