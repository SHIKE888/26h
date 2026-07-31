#ifndef __BSP_ROLLBALL_STEPPER_H
#define __BSP_ROLLBALL_STEPPER_H

/* ==========================================================================
 * bsp_rollball_stepper.h —— 球杆系统步进电机驱动接口
 * --------------------------------------------------------------------------
 * 【步进电机是怎么被控制的？】
 *   步进驱动器（如 TB6600 / D36A 这类）只认三根信号线：
 *     STEP(PUL)：每来一个脉冲，电机转一个“步距角”。脉冲越快 → 转得越快。
 *     DIR      ：高/低电平决定顺时针还是逆时针。
 *     EN       ：使能。失能时电机线圈断电，可以用手随便转（也就“软了”）。
 *
 *   所以“控制电机速度” = “控制 STEP 脉冲的频率”。
 *   本文件对外暴露的核心函数就是：
 *       RollBallStepper_SetFrequency(频率Hz, 方向电平)
 *
 * 【脉冲是怎么产生的？】
 *   用定时器 TIM8 的“输出比较 + 翻转(Toggle)模式”：
 *     TIM8 的计数器以 100 kHz 一直往上数；
 *     每次数到 CCR1 寄存器的值，硬件就把 PC6 引脚电平翻转一次，
 *     同时进一个中断，中断里把 CCR1 再加上一个固定间隔，安排下一次翻转。
 *   注意：翻转两次（高→低→高）才算一个完整脉冲，
 *   所以“间隔 tick 数 = 100000 / (频率 × 2)”，代码里就是这么算的。
 *
 * 【为什么不用普通 PWM？】
 *   PWM 改频率要动 ARR/PSC，改的瞬间容易产生半个畸形脉冲，
 *   而步进电机对“丢脉冲/多脉冲”很敏感。用输出比较+软件累加 CCR，
 *   频率可以在任意时刻平滑切换，不会破坏脉冲完整性。
 * ========================================================================== */

#include "main.h"

/*
 * TIM8 runs at 100 kHz.  PC6 toggles once per compare event, therefore one
 * complete STEP pulse needs two compare events.
 *
 * 中文说明：
 *   TIM8 时基 100 kHz（168MHz 主频 ÷ (1679+1) = 100kHz，见 tim.c）。
 *   即计数器每 10 微秒 +1。PC6 每次比较事件翻转一次电平，
 *   两次翻转 = 一个完整 STEP 脉冲。
 */
#define ROLLBALL_STEPPER_TIMER_TICK_HZ       100000U   //定时器每秒计数 10 万次
#define ROLLBALL_STEPPER_MIN_FREQUENCY_HZ         1U   //允许的最低脉冲频率
#define ROLLBALL_STEPPER_MAX_FREQUENCY_HZ      5000U   //允许的最高脉冲频率（硬上限）

/* 1.8-degree motor with the driver configured for 16 microsteps.
 * 中文：1.8° 步距角电机（转一圈 360/1.8 = 200 个整步），
 *       驱动器拨码设为 16 细分，于是转一圈需要 200×16 = 3200 个脉冲。
 * 换算：脉冲频率 f (Hz) 对应转速 = f / 3200 转/秒。
 *       例如 1600Hz ≈ 0.5 转/秒 = 30 转/分。
 * 【重要】如果你改了驱动器的细分拨码，一定要同步改下面这个宏！ */
#define ROLLBALL_STEPPER_FULL_STEPS_PER_REV      200U   //整步数/圈（1.8°电机）
#define ROLLBALL_STEPPER_MICROSTEP_DIVISION       16U   //驱动器细分数（拨码决定）
#define ROLLBALL_STEPPER_PULSES_PER_REV         \
	(ROLLBALL_STEPPER_FULL_STEPS_PER_REV *       \
	 ROLLBALL_STEPPER_MICROSTEP_DIVISION)              //= 3200 脉冲/圈

/*
 * The drawing example enables its D36A driver with a high EN level.
 * Change this one macro if the installed driver uses an active-low EN input.
 *
 * 中文：EN 引脚“哪个电平算使能”。参考的 D36A 驱动器是高电平使能。
 *       如果你换的驱动器是低电平使能（很常见！比如很多 TB6600 模块），
 *       只需要把这里改成 GPIO_PIN_RESET，其它代码一行都不用动。
 *       现象判断：上电后电机一直发烫锁死 / 或者始终能用手轻松转动，
 *                 多半就是这个宏设反了。
 */
#define ROLLBALL_STEPPER_ENABLE_ACTIVE_LEVEL GPIO_PIN_SET

//初始化：配置 DIR/EN 引脚为输出，把 TIM8 摆到“停止且不输出”的安全状态
void RollBallStepper_Init(void);
//使能/失能驱动器。1=通电锁住，0=断电放松（失能前会自动停脉冲）
HAL_StatusTypeDef RollBallStepper_SetEnabled(uint8_t enabled);
//查询当前是否处于使能状态
uint8_t RollBallStepper_IsEnabled(void);

/*
 * Start or update a continuous STEP stream.
 * frequency_hz == 0 stops STEP output but keeps EN unchanged.
 *
 * 中文：启动或更新连续脉冲输出。这是控制电机的主入口。
 * @param frequency_hz 脉冲频率(Hz)，决定转速；传 0 表示停止发脉冲（EN 不变）
 * @param direction    DIR 引脚电平，决定转向
 * @retval HAL_OK 成功；HAL_ERROR 表示参数越界或驱动器还没使能
 */
HAL_StatusTypeDef RollBallStepper_SetFrequency(uint32_t frequency_hz,
	                                           GPIO_PinState direction);
//立即停止脉冲输出（电机停转，但仍处于使能锁住状态）
void RollBallStepper_Stop(void);
//读回当前正在输出的频率（0 表示已停）
uint32_t RollBallStepper_GetFrequency(void);
//读回当前 DIR 电平
GPIO_PinState RollBallStepper_GetDirection(void);

/* Called directly by TIM8_CC_IRQHandler.
 * 中文：由 stm32f4xx_it.c 里的 TIM8_CC_IRQHandler() 直接调用，
 *       负责安排下一次电平翻转。用户代码不要手动调用它。 */
void RollBallStepper_TIM8_CC_IRQHandler(void);

#endif
