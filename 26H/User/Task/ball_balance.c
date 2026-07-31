/**
 * @file    ball_balance.c
 * @brief   钢球平衡控制实现 (PD + EMA滤波 + 步进电机限位)
 *
 * 控制公式:
 *   error    = target_x - filtered_x
 *   d_error  = error - prev_error
 *   i_error += error * dt (积分, 带限幅)
 *   output   = Kp*error + Kd*d_error + Ki*i_error
 *   angle    = CLAMP(output, -15°, +15°)
 *   → StepMotor_SetAngle(angle)
 *
 * 预处理:
 *   filtered_x = α*raw_x + (1-α)*filtered_x   (EMA一阶低通)
 *   检测丢失时: 不更新滤波器, 超时后缓慢回零
 */

#include "ball_balance.h"
#include "k230_uart.h"
#include "step_motor.h"

/* 简易绝对值 (避免链接 math.h) */
#define ABSF(x) ((x) < 0.0f ? -(x) : (x))

/* ---- 外部引用 ---- */
extern StepMotor g_step_motor;

/* ---- 内部状态 ---- */
static float g_target_x = 400.0f;
static float g_filtered_x = 0.0f;
static float g_prev_error = 0.0f;
static float g_i_error = 0.0f;
static float g_cur_angle = 0.0f;
static uint32_t g_last_valid_tick = 0;
static volatile float g_pending_angle = 0.0f; /* 待发送的目标角度 */
static volatile uint8_t g_angle_dirty = 0;    /* 角度已更新标志 */
static volatile uint8_t g_paused = 0;         /* 暂停标志: 1=暂停平衡控制 */

/*  API
 * ========================================================================== */

/**
 * @brief 初始化平衡控制器
 * @param target_x 目标 X 坐标 (K230 像素坐标)
 */
void BallBalance_Init(float target_x)
{
    g_target_x = target_x;
    g_filtered_x = target_x; /* 初始假设球在目标位置 */
    g_prev_error = 0.0f;
    g_i_error = 0.0f;
    g_cur_angle = 0.0f;

    /* 设置步进电机为平衡模式参数 (中速, 平滑加减速) */
    g_step_motor.speed = BAL_MOTOR_SPEED;
    g_step_motor.acc = BAL_MOTOR_ACC;

    /* 记录时间戳 */
    g_last_valid_tick = HAL_GetTick();
}

/**
 * @brief 主控制循环, 每 10ms 由 TIM1 回调调用
 */
void BallBalance_Tick(void)
{
    uint32_t now = HAL_GetTick();

    /* ---- 暂停模式: 不计算, 不发送, 仅维持当前位置 ---- */
    if (g_paused)
        return;

    /* ---- 1. 读取 K230 数据并预处理 ---- */
    uint16_t raw_x = g_k230_data.x;
    float x_val;

    if (raw_x != 65535) /* 有效目标 */
    {
        x_val = (float)raw_x;

        /* 异常值剔除: 坐标应在合理范围 (0~800) */
        if (x_val >= 0.0f && x_val <= 800.0f)
        {
            /* EMA 一阶低通滤波 */
            g_filtered_x = BAL_EMA_ALPHA * x_val + (1.0f - BAL_EMA_ALPHA) * g_filtered_x;

            g_last_valid_tick = now;
        }
    }

    /* ---- 2. 目标丢失检测: 超时后缓慢回零 ---- */
    if (now - g_last_valid_tick > BAL_TIMEOUT_MS)
    {
        /* 缓慢回零 (每 10ms 回 0.1°) */
        if (g_cur_angle > 0.1f)
            g_cur_angle -= 0.1f;
        else if (g_cur_angle < -0.1f)
            g_cur_angle += 0.1f;
        else
            g_cur_angle = 0.0f;

        /* 重置积分, 避免回零后积分残留导致跳变 */
        g_i_error = 0.0f;

        g_step_motor.speed = BAL_MOTOR_SPEED;
        g_step_motor.acc = BAL_MOTOR_ACC;
        g_angle_dirty = 1;
        g_pending_angle = g_cur_angle;
        return;
    }

    /* ---- 3. PD+PI 控制器计算 ----
     * error = x - target (正=球在目标右侧, 需左倾回滚)
     * 方向: 负角度 → CCW → 右倾; 正角度 → CW → 左倾
     * angle = -PID(error)
     */
    float error = g_filtered_x - g_target_x; /* 像素误差 */
    float d_error = error - g_prev_error;    /* 误差微分 */

    /* 积分项 (带抗饱和限幅) */
    g_i_error += error * (BAL_CTRL_PERIOD_MS / 1000.0f);
    if (g_i_error > 50.0f)
        g_i_error = 50.0f;
    if (g_i_error < -50.0f)
        g_i_error = -50.0f;

    /* PD+PI 输出, 取负 (正误差→右倾→负角度) */
    float output = -(BAL_KP * error + BAL_KD * d_error + BAL_KI * g_i_error);

    /* 限幅到 ±15° */
    if (output > BAL_ANGLE_MAX)
        output = BAL_ANGLE_MAX;
    if (output < BAL_ANGLE_MIN)
        output = BAL_ANGLE_MIN;

    /* ---- 4. 死区: 误差很小时不动作, 避免抖动 ---- */
    if (ABSF(error) < 2.0f && ABSF(output) < 0.5f)
    {
        output = 0.0f;
        g_i_error = 0.0f;
    }

    /* ---- 5. 限速: 限制每周期角度变化量, 实现平滑加速 ---- */
    float delta = output - g_cur_angle;
    float max_step = 0.8f; /* 每 10ms 最大 0.8° → 80°/s */
    if (delta > max_step)
        delta = max_step;
    if (delta < -max_step)
        delta = -max_step;
    g_cur_angle += delta;

    /* 再次限幅 */
    if (g_cur_angle > BAL_ANGLE_MAX)
        g_cur_angle = BAL_ANGLE_MAX;
    if (g_cur_angle < BAL_ANGLE_MIN)
        g_cur_angle = BAL_ANGLE_MIN;

    /* ---- 6. 动态调速: 大偏差时提高转速 ---- */
    if (ABSF(error) > 50.0f)
        g_step_motor.speed = BAL_MOTOR_SPEED;
    else if (ABSF(error) > 20.0f)
        g_step_motor.speed = 200;
    else
        g_step_motor.speed = 120;

    g_step_motor.acc = BAL_MOTOR_ACC;

    /* ---- 7. 设置待发送角度 (主循环中发送, 不在中断中调用阻塞函数) ---- */
    g_pending_angle = g_cur_angle;
    g_angle_dirty = 1; /* 先写值, 后置标志, 避免主循环读到旧值 */

    /* 保存状态用于下次迭代 */
    g_prev_error = error;
}

/* ========================================================================== *
 *  状态获取 (供 OLED 显示)
 * ========================================================================== */

float BallBalance_GetAngle(void)
{
    return g_cur_angle;
}

float BallBalance_GetFilteredX(void)
{
    return g_filtered_x;
}

int32_t BallBalance_GetError(void)
{
    return (int32_t)(g_target_x - g_filtered_x);
}

/**
 * @brief 发送电机指令 (在主循环中调用, 不能在中断中调用)
 */
void BallBalance_SendMotorCmd(void)
{
    if (!g_paused && g_angle_dirty)
    {
        g_angle_dirty = 0;
        StepMotor_SetAngle(&g_step_motor, g_pending_angle);
    }
}

/* ========================================================================== *
 *  暂停/恢复
 * ========================================================================== */

/**
 * @brief 暂停平衡控制 (失能步进电机时调用)
 */
void BallBalance_Pause(void)
{
    g_paused = 1;
    g_angle_dirty = 0;
    g_i_error = 0.0f;
    g_prev_error = 0.0f;
}

/**
 * @brief 恢复平衡控制 (使能步进电机时调用)
 */
void BallBalance_Resume(void)
{
    g_filtered_x = g_target_x; /* 重置滤波值为目标值, 避免恢复后突变 */
    g_cur_angle = 0.0f;
    g_prev_error = 0.0f;
    g_i_error = 0.0f;
    g_angle_dirty = 0;
    g_paused = 0;
}
