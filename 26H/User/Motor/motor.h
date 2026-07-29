/**
 * @file    motor.h
 * @brief   TB6612 双路电机驱动 (PWM + 方向控制) — 移植 F411
 * @note    26H CubeMX 配置:
 *          TIM2_CH1(PA15)=MB_PWM → 电机B
 *          TIM2_CH2(PB3)=MA_PWM  → 电机A
 *          方向: MA1(PB12)/MA2(PB13), MB1(PB14)/MB2(PB15)
 */

#ifndef MOTOR_MOTOR_H_
#define MOTOR_MOTOR_H_

#include "main.h"

/* ======================== PWM 调参宏 ================================ */
/* TIM2: PSC=0, ARR=7199 → 占空比 0~7199                            */
#define PWM_MAX_DUTY 7199
#define PWM_MIN_DUTY 0

/* ====================== 电机换向宏 ================================== */
/* 设为 1 反转对应电机方向 (根据实际接线调整)                          */
#define MOTOR_A_DIR_INVERT 0
#define MOTOR_B_DIR_INVERT 0

/* ====================== PWM 通道映射 ================================== */
/* 26H CubeMX: TIM2_CH1(PA15)=MB, TIM2_CH2(PB3)=MA                    */
#define MOTOR_A_PWM_CHANNEL TIM_CHANNEL_2 /* MA_PWM → PB3  → TIM2_CH2 */
#define MOTOR_B_PWM_CHANNEL TIM_CHANNEL_1 /* MB_PWM → PA15 → TIM2_CH1 */

/* ====================== 速度闭环 PID 调参宏 =========================== */
#define SPEED_PID_KP 0.6f
#define SPEED_PID_KI 0.1f
#define SPEED_PID_KD 0.0f
#define SPEED_PID_OUT_MAX 7199.0f
#define SPEED_PID_ALPHA 0.0f

/* ============================= 方向宏 ============================== */
/* TB6612 真值表:
 *   IN1=1, IN2=0 → CW  (正转)
 *   IN1=0, IN2=1 → CCW (反转)
 *   IN1=0, IN2=0 → STOP (停止)
 *   IN1=1, IN2=1 → BRAKE (刹车)
 *
 * 注: CW/CCW 方向取决于电机接线, 可通过 MOTOR_x_DIR_INVERT 调整
 * ====================================================================== */

/* --- 电机 A (MA1=PB12, MA2=PB13) --- */
#if MOTOR_A_DIR_INVERT
#define MotorA_CW()                                                \
    do                                                             \
    {                                                              \
        HAL_GPIO_WritePin(MA1_GPIO_Port, MA1_Pin, GPIO_PIN_RESET); \
        HAL_GPIO_WritePin(MA2_GPIO_Port, MA2_Pin, GPIO_PIN_SET);   \
    } while (0)
#define MotorA_CCW()                                               \
    do                                                             \
    {                                                              \
        HAL_GPIO_WritePin(MA1_GPIO_Port, MA1_Pin, GPIO_PIN_SET);   \
        HAL_GPIO_WritePin(MA2_GPIO_Port, MA2_Pin, GPIO_PIN_RESET); \
    } while (0)
#else
#define MotorA_CW()                                                \
    do                                                             \
    {                                                              \
        HAL_GPIO_WritePin(MA1_GPIO_Port, MA1_Pin, GPIO_PIN_SET);   \
        HAL_GPIO_WritePin(MA2_GPIO_Port, MA2_Pin, GPIO_PIN_RESET); \
    } while (0)
#define MotorA_CCW()                                               \
    do                                                             \
    {                                                              \
        HAL_GPIO_WritePin(MA1_GPIO_Port, MA1_Pin, GPIO_PIN_RESET); \
        HAL_GPIO_WritePin(MA2_GPIO_Port, MA2_Pin, GPIO_PIN_SET);   \
    } while (0)
#endif

#define MotorA_Stop()                                              \
    do                                                             \
    {                                                              \
        HAL_GPIO_WritePin(MA1_GPIO_Port, MA1_Pin, GPIO_PIN_RESET); \
        HAL_GPIO_WritePin(MA2_GPIO_Port, MA2_Pin, GPIO_PIN_RESET); \
    } while (0)
#define MotorA_Brake()                                           \
    do                                                           \
    {                                                            \
        HAL_GPIO_WritePin(MA1_GPIO_Port, MA1_Pin, GPIO_PIN_SET); \
        HAL_GPIO_WritePin(MA2_GPIO_Port, MA2_Pin, GPIO_PIN_SET); \
    } while (0)

/* --- 电机 B (MB1=PB14, MB2=PB15) --- */
#if MOTOR_B_DIR_INVERT
#define MotorB_CW()                                                \
    do                                                             \
    {                                                              \
        HAL_GPIO_WritePin(MB1_GPIO_Port, MB1_Pin, GPIO_PIN_RESET); \
        HAL_GPIO_WritePin(MB2_GPIO_Port, MB2_Pin, GPIO_PIN_SET);   \
    } while (0)
#define MotorB_CCW()                                               \
    do                                                             \
    {                                                              \
        HAL_GPIO_WritePin(MB1_GPIO_Port, MB1_Pin, GPIO_PIN_SET);   \
        HAL_GPIO_WritePin(MB2_GPIO_Port, MB2_Pin, GPIO_PIN_RESET); \
    } while (0)
#else
#define MotorB_CW()                                                \
    do                                                             \
    {                                                              \
        HAL_GPIO_WritePin(MB1_GPIO_Port, MB1_Pin, GPIO_PIN_SET);   \
        HAL_GPIO_WritePin(MB2_GPIO_Port, MB2_Pin, GPIO_PIN_RESET); \
    } while (0)
#define MotorB_CCW()                                               \
    do                                                             \
    {                                                              \
        HAL_GPIO_WritePin(MB1_GPIO_Port, MB1_Pin, GPIO_PIN_RESET); \
        HAL_GPIO_WritePin(MB2_GPIO_Port, MB2_Pin, GPIO_PIN_SET);   \
    } while (0)
#endif

#define MotorB_Stop()                                              \
    do                                                             \
    {                                                              \
        HAL_GPIO_WritePin(MB1_GPIO_Port, MB1_Pin, GPIO_PIN_RESET); \
        HAL_GPIO_WritePin(MB2_GPIO_Port, MB2_Pin, GPIO_PIN_RESET); \
    } while (0)
#define MotorB_Brake()                                           \
    do                                                           \
    {                                                            \
        HAL_GPIO_WritePin(MB1_GPIO_Port, MB1_Pin, GPIO_PIN_SET); \
        HAL_GPIO_WritePin(MB2_GPIO_Port, MB2_Pin, GPIO_PIN_SET); \
    } while (0)

/* =========================== 函数声明 ================================ */

/**
 * @brief 初始化电机 GPIO 和 PWM
 * @note  TIM2 已在 CubeMX 中配置为 PWM 输出, 此处仅启动 PWM
 */
void Motor_Init(void);

/**
 * @brief 设置电机 A 速度和方向
 * @param speed  速度值
 *   >0  -> 正转, duty = speed (限制 <= PWM_MAX_DUTY)
 *   <0  -> 反转, duty = |speed| (限制 <= PWM_MAX_DUTY)
 *   =0  -> 停止
 */
void MotorA_SetSpeed(int32_t speed);

/**
 * @brief 设置电机 B 速度和方向
 * @param speed  同 MotorA_SetSpeed
 */
void MotorB_SetSpeed(int32_t speed);

/**
 * @brief 同时设置两个电机速度
 * @param speedA  电机A速度
 * @param speedB  电机B速度
 */
void Motor_SetSpeed(int32_t speedA, int32_t speedB);

/**
 * @brief 停止两个电机
 */
void Motor_Stop(void);

/**
 * @brief 急停刹车: 短接制动电机, hold_ms 后自动停止
 * @note  在中断中调用启动刹车，在主循环中调用 Motor_BrakeUpdate() 驱动
 */
void Motor_EmergencyBrake(uint32_t hold_ms);
uint8_t Motor_BrakeUpdate(void);

/* ======================== 速度闭环控制 ============================ */

#include "pid.h"

extern PID_t g_speed_pid_a; /**< 电机A速度PID */
extern PID_t g_speed_pid_b; /**< 电机B速度PID */
extern int32_t g_target_speed_a;
extern int32_t g_target_speed_b;
extern int32_t g_pwm_actual_a;     /**< 电机A实际PWM输出值 (CCR) */
extern int32_t g_pwm_actual_b;     /**< 电机B实际PWM输出值 (CCR) */
extern int32_t g_speed_snapshot_a; /**< 编码器A快照 (与PID计算同步) */
extern int32_t g_speed_snapshot_b; /**< 编码器B快照 (与PID计算同步) */

/**
 * @brief  初始化速度闭环 PID 控制器
 * @note   调用 PID_Reset 清除积分累积和历史误差, 每次启动前应调用
 */
void Motor_SpeedControl_Init(void);

/**
 * @brief 设置电机A的目标编码器速度
 */
void Motor_SetTargetSpeedA(int32_t target);

/**
 * @brief 设置电机B的目标编码器速度
 */
void Motor_SetTargetSpeedB(int32_t target);

/**
 * @brief 同时设置两个电机的目标编码器速度
 */
void Motor_SetTargetSpeed(int32_t targetA, int32_t targetB);

/**
 * @brief 设置目标速度并重置 PID 积分 (启动时调用)
 */
void Motor_SetTargetSpeed_Reset(int32_t targetA, int32_t targetB);

/**
 * @brief 速度闭环更新函数 (应在 10ms 周期中调用)
 * @note  读取实际编码器速度, PID 计算后输出 PWM 给电机
 */
void Motor_SpeedControl_Update(void);

#endif /* MOTOR_MOTOR_H_ */
