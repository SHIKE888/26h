/**
 * @file    motor.c
 * @brief   TB6612 双路电机驱动实现 — 移植 F411
 * @note    TIM2_CH1(PA15)=MB_PWM=电机B, TIM2_CH2(PB3)=MA_PWM=电机A
 *          方向: MA1(PB12)/MA2(PB13), MB1(PB14)/MB2(PB15)
 */

#include "motor.h"
#include "tim.h"
#include "encoder.h"

/* ==================== 速度闭环 PID 全局变量 ===================== */
PID_t g_speed_pid_a = {0};
PID_t g_speed_pid_b = {0};
int32_t g_target_speed_a = 0;
int32_t g_target_speed_b = 0;
int32_t g_pwm_actual_a = 0;
int32_t g_pwm_actual_b = 0;
int32_t g_speed_snapshot_a = 0;
int32_t g_speed_snapshot_b = 0;

/* ========================== 限幅函数 ================================= */
static int32_t clamp(int32_t val, int32_t min, int32_t max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

/* ========================== 初始化 =================================== */
void Motor_Init(void)
{
    /* TIM2 PWM 已在 CubeMX MX_TIM2_Init 中配置完毕, 仅需启动通道 */
    HAL_TIM_PWM_Start(&htim2, MOTOR_A_PWM_CHANNEL); /* PB3  -> 电机A PWM */
    HAL_TIM_PWM_Start(&htim2, MOTOR_B_PWM_CHANNEL); /* PA15 -> 电机B PWM */

    Motor_Stop();
}

/* ========================== 电机 A =================================== */
void MotorA_SetSpeed(int32_t speed)
{
    if (speed > 0)
    {
        MotorA_CW();
        g_pwm_actual_a = clamp(speed, PWM_MIN_DUTY, PWM_MAX_DUTY);
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR_A_PWM_CHANNEL, g_pwm_actual_a);
    }
    else if (speed < 0)
    {
        MotorA_CCW();
        g_pwm_actual_a = clamp(-speed, PWM_MIN_DUTY, PWM_MAX_DUTY);
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR_A_PWM_CHANNEL, g_pwm_actual_a);
    }
    else
    {
        MotorA_Stop();
        g_pwm_actual_a = 0;
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR_A_PWM_CHANNEL, 0);
    }
}

/* ========================== 电机 B =================================== */
void MotorB_SetSpeed(int32_t speed)
{
    if (speed > 0)
    {
        MotorB_CW();
        g_pwm_actual_b = clamp(speed, PWM_MIN_DUTY, PWM_MAX_DUTY);
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR_B_PWM_CHANNEL, g_pwm_actual_b);
    }
    else if (speed < 0)
    {
        MotorB_CCW();
        g_pwm_actual_b = clamp(-speed, PWM_MIN_DUTY, PWM_MAX_DUTY);
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR_B_PWM_CHANNEL, g_pwm_actual_b);
    }
    else
    {
        MotorB_Stop();
        g_pwm_actual_b = 0;
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR_B_PWM_CHANNEL, 0);
    }
}

/* ========================== 双电机 =================================== */
void Motor_SetSpeed(int32_t speedA, int32_t speedB)
{
    MotorA_SetSpeed(speedA);
    MotorB_SetSpeed(speedB);
}

void Motor_Stop(void)
{
    MotorA_Stop();
    MotorB_Stop();
    __HAL_TIM_SET_COMPARE(&htim2, MOTOR_A_PWM_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&htim2, MOTOR_B_PWM_CHANNEL, 0);
    g_target_speed_a = 0;
    g_target_speed_b = 0;
}

/* ======================= 速度闭环 PID 控制实现 ======================= */

/**
 * @brief 初始化速度闭环 PID 控制器
 * @note  每次启动/模式切换前应调用, 清除积分累积和历史误差
 */
void Motor_SpeedControl_Init(void)
{
    PID_Init(&g_speed_pid_a, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_OUT_MAX);
    PID_Init(&g_speed_pid_b, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, SPEED_PID_OUT_MAX);
    PID_SetAlpha(&g_speed_pid_a, SPEED_PID_ALPHA);
    PID_SetAlpha(&g_speed_pid_b, SPEED_PID_ALPHA);
}

/* ---- 目标速度设置 ---- */
void Motor_SetTargetSpeedA(int32_t target) { g_target_speed_a = target; }
void Motor_SetTargetSpeedB(int32_t target) { g_target_speed_b = target; }

void Motor_SetTargetSpeed(int32_t a, int32_t b)
{
    g_target_speed_a = a;
    g_target_speed_b = b;
}

void Motor_SetTargetSpeed_Reset(int32_t a, int32_t b)
{
    g_target_speed_a = a;
    g_target_speed_b = b;
    PID_Reset(&g_speed_pid_a);
    PID_Reset(&g_speed_pid_b);
}

/**
 * @brief 速度闭环更新函数 (应在定时中断中以固定周期调用, 如 10ms)
 * @note  使用增量式 PID 计算 PWM 输出, 积分项可消除稳态误差
 */
void Motor_SpeedControl_Update(void)
{
    /* 快照编码器值, 确保 PID 计算与 OLED 显示使用同一采样值 */
    int32_t spd_a = Encoder_GetSpeedA();
    int32_t spd_b = Encoder_GetSpeedB();
    g_speed_snapshot_a = spd_a;
    g_speed_snapshot_b = spd_b;

    float pid_out_a = PID_Calc(&g_speed_pid_a,
                               (float)g_target_speed_a,
                               (float)spd_a);
    float pid_out_b = PID_Calc(&g_speed_pid_b,
                               (float)g_target_speed_b,
                               (float)spd_b);

    MotorA_SetSpeed((int32_t)pid_out_a);
    MotorB_SetSpeed((int32_t)pid_out_b);
}

/* ======================== 急停刹车 ================================ */

static uint8_t g_brake_active = 0;
static uint32_t g_brake_end = 0;

void Motor_EmergencyBrake(uint32_t hold_ms)
{
    MotorA_Brake();
    MotorB_Brake();
    __HAL_TIM_SET_COMPARE(&htim2, MOTOR_A_PWM_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&htim2, MOTOR_B_PWM_CHANNEL, 0);
    g_target_speed_a = 0;
    g_target_speed_b = 0;
    g_brake_active = 1;
    g_brake_end = HAL_GetTick() + hold_ms;
}

uint8_t Motor_BrakeUpdate(void)
{
    if (!g_brake_active)
        return 0;

    if (HAL_GetTick() >= g_brake_end)
    {
        MotorA_Stop();
        MotorB_Stop();
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR_A_PWM_CHANNEL, 0);
        __HAL_TIM_SET_COMPARE(&htim2, MOTOR_B_PWM_CHANNEL, 0);
        g_brake_active = 0;
        return 0;
    }
    return 1;
}
