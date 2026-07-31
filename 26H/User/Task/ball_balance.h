/**
 * @file    ball_balance.h
 * @brief   钢球平衡控制系统 (PID + 步进电机)
 *
 * 控制链路:
 *   K230 x坐标 → EMA滤波 → PD控制器 → 步进电机角度 → 管道倾斜 → 钢球滚动
 *
 * 方向约定:
 *   CCW (负角度) → 管道右倾 → 球右滚
 *    CW (正角度) → 管道左倾 → 球左滚
 *
 * 限位: ±15° (防止机械碰撞)
 */

#ifndef BALL_BALANCE_H_
#define BALL_BALANCE_H_

#include "stm32f4xx_hal.h"

/* ---- PID 参数 (可在线调整) ---- */
#define BAL_KP 0.018f /* 比例增益 (deg/pixel) */
#define BAL_KD 2.0f   /* 微分增益 */
#define BAL_KI 0.04f  /* 积分增益 (小值, 消除稳态误差) */

/* ---- 滤波器 ---- */
#define BAL_EMA_ALPHA 0.30f /* EMA 平滑系数 (0~1, 越小越平滑) */

/* ---- 限位 ---- */
#define BAL_ANGLE_MAX 20.0f  /* 最大倾斜角 (°) */
#define BAL_ANGLE_MIN -15.0f /* 最小倾斜角 (°) */

/* ---- 控制周期 (ms) ---- */
#define BAL_CTRL_PERIOD_MS 10

/* ---- 目标丢失超时 (ms), 超时后缓慢回零 ---- */
#define BAL_TIMEOUT_MS 500

/* ---- 电机参数 ---- */
#define BAL_MOTOR_SPEED 300 /* 转速 (RPM) */
#define BAL_MOTOR_ACC 0x60  /* 加速度 */

/* ---- 初始化 ---- */
void BallBalance_Init(float target_x);

/* ---- 主控制循环 (在 TIM1 10ms 回调中调用) ---- */
void BallBalance_Tick(void);

/* ---- 暂停/恢复平衡控制 (失能步进电机时使用) ---- */
void BallBalance_Pause(void);
void BallBalance_Resume(void);

/* ---- 获取当前状态 (供 OLED 显示) ---- */
float BallBalance_GetAngle(void);
float BallBalance_GetFilteredX(void);
int32_t BallBalance_GetError(void);

#endif /* BALL_BALANCE_H_ */
