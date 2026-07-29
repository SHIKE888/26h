/**
 * @file    encoder.c
 * @brief   500线 GMR 编码器测速实现 (防溢出处理) — 移植 F411
 * @note    电机A: TIM3_CH1/CH2 (PB4/PB5), 电机B: TIM4_CH1/CH2 (PB6/PB7)
 *          CubeMX 已将 TIM3/TIM4 配置为编码器模式
 */

#include "encoder.h"
#include "tim.h"

/* ========================== 全局变量 ================================= */
Encoder_TypeDef g_encoder_A = {0};
Encoder_TypeDef g_encoder_B = {0};

/* ========================== 接口实现 ================================= */

void Encoder_Init(void)
{
    /* TIM3/TIM4 已在 MX_TIM3/4_Init() 中配置为 TI12 编码器模式 */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    g_encoder_A.pulse = 0;
    g_encoder_A.speed = 0;
    g_encoder_A.speed_f = 0.0f;
    g_encoder_A.speed_raw = 0;
    g_encoder_B.pulse = 0;
    g_encoder_B.speed = 0;
    g_encoder_B.speed_f = 0.0f;
    g_encoder_B.speed_raw = 0;
}

void Encoder_Update(uint32_t dt_ms)
{
    (void)dt_ms;

    /* --- 电机A: TIM3 --- */
    int cnt_a = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

#if ENCODER_A_INVERT
    cnt_a = -cnt_a;
#endif
    g_encoder_A.pulse += cnt_a;
    g_encoder_A.speed_raw = cnt_a;

    /* 低通滤波 */
    if (ENCODER_SPEED_ALPHA > 0.0f)
    {
        g_encoder_A.speed_f = ENCODER_SPEED_ALPHA * (float)cnt_a + (1.0f - ENCODER_SPEED_ALPHA) * g_encoder_A.speed_f;
        g_encoder_A.speed = (int32_t)g_encoder_A.speed_f;
    }
    else
    {
        g_encoder_A.speed = cnt_a;
    }

    /* --- 电机B: TIM4 --- */
    int cnt_b = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);

#if ENCODER_B_INVERT
    cnt_b = -cnt_b;
#endif
    g_encoder_B.pulse += cnt_b;
    g_encoder_B.speed_raw = cnt_b;

    if (ENCODER_SPEED_ALPHA > 0.0f)
    {
        g_encoder_B.speed_f = ENCODER_SPEED_ALPHA * (float)cnt_b + (1.0f - ENCODER_SPEED_ALPHA) * g_encoder_B.speed_f;
        g_encoder_B.speed = (int32_t)g_encoder_B.speed_f;
    }
    else
    {
        g_encoder_B.speed = cnt_b;
    }
}

void Encoder_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    g_encoder_A.pulse = 0;
    g_encoder_A.speed = 0;
    g_encoder_A.speed_f = 0.0f;
    g_encoder_A.speed_raw = 0;
    g_encoder_B.pulse = 0;
    g_encoder_B.speed = 0;
    g_encoder_B.speed_f = 0.0f;
    g_encoder_B.speed_raw = 0;
}
