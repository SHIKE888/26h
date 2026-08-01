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

/* 轨道水平补偿: 电机原点≠物理水平时的修正角 (°) */
#define BAL_LEVEL_BIAS -0.8f

/* ---- 外部引用 ---- */
extern StepMotor g_step_motor;

/* ---- 内部状态 ---- */
static float g_target_x = 400.0f;
static float g_filtered_x = 0.0f;
static float g_prev_error = 0.0f;
static float g_i_error = 0.0f;
static float g_cur_angle = 0.0f;
static float g_ff_accel = 0.0f; /* 前馈加速度 (cm/s²) */
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
            /* 所有模式均使用 K230 实时 X 坐标作为 PID 反馈 */
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
     * 方向: 负角度 -> CCW -> 右倾; 正角度 -> CW -> 左倾
     * angle = -PID(error)
     */
    float error = g_filtered_x - g_target_x; /* 像素误差 */
    float d_error = error - g_prev_error;    /* 误差微分 */

    /* 积分项 (带抗饱和限幅 + 容差内抑制 + F2 保守) */
    if (ABSF(error) > 6.0f) /* 仅大偏差时累积积分, 防止稳态蓄力 */
    {
        g_i_error += error * (BAL_CTRL_PERIOD_MS / 1000.0f);
    }
    if (g_i_error > 100.0f)
        g_i_error = 100.0f;
    if (g_i_error < -100.0f)
        g_i_error = -100.0f;

    /* ---- 启动补偿: F2严格防超调时关闭, 其他模式可用 ---- */
    static uint32_t kick_cooldown_tick = 0;
    float kick = 0.0f;
#if 1 /* 设为 0 可完全禁用 kick */
    if (ABSF(error) > 15.0f && ABSF(d_error) < 1.0f)
    {
        if (HAL_GetTick() - kick_cooldown_tick > 500)
        {
            kick = (error > 0) ? 2.5f : -2.5f;
            kick_cooldown_tick = HAL_GetTick();
        }
    }
#endif

    /* F2模式检测: 若速度上限非零, 切换为纯开环控制 */
    extern uint16_t g_ball_speed_cap;
    uint8_t is_demo_mode = (g_ball_speed_cap > 0);

    if (is_demo_mode)
    {
        /* F2 纯开环: 完全不使用 PID, 用预设角度+时间序列控制 */
        extern uint8_t g_ball_demo_phase;
        float demo_angle = 0.0f;
        switch (g_ball_demo_phase)
        {
        case 1:
            demo_angle = BAL_DEMO_ANGLE_LEFT;
            break;
        case 2:
            demo_angle = 0.0f;
            break;
        case 3:
            demo_angle = BAL_DEMO_ANGLE_RIGHT;
            break;
        case 4:
            demo_angle = -2.0f;
            break;
        default:
            break;
        }
        float output = demo_angle;
        if (output > BAL_ANGLE_MAX)
            output = BAL_ANGLE_MAX;
        if (output < BAL_ANGLE_MIN)
            output = BAL_ANGLE_MIN;
        g_cur_angle = output;
        g_step_motor.speed = BAL_DEMO_SPEED_CAP;
        g_step_motor.acc = BAL_MOTOR_ACC;
        goto set_pending;
    }

    /* ===== 以下为通用 PID 闭环控制 (F1/F3/F4/F5 共用) ===== */
    {
        float output = -(BAL_KP * error + BAL_KD * d_error + BAL_KI * g_i_error) + kick;

        /* ---- 前馈补偿 ---- */
        if (g_ff_accel > BAL_FF_DEADBAND || g_ff_accel < -BAL_FF_DEADBAND)
            output += BAL_FF_K * g_ff_accel;

        /* 限幅 */
        if (output > BAL_ANGLE_MAX)
            output = BAL_ANGLE_MAX;
        if (output < BAL_ANGLE_MIN)
            output = BAL_ANGLE_MIN;

        output += BAL_LEVEL_BIAS;

        /* ---- 死区 + 分级抑制 ---- */
        float abs_err = ABSF(error);
        if (abs_err < 3.0f)
            output = 0.0f;
        else if (abs_err < 10.0f && ABSF(output) < 3.0f)
            output *= 0.3f;

        /* ---- 接近减速 ---- */
        float delta = output - g_cur_angle;
        float max_step;
        if (abs_err < 5.0f)
            max_step = 0.25f;
        else if (abs_err < 15.0f)
            max_step = 0.8f;
        else if (abs_err < 40.0f)
            max_step = 1.5f;
        else
            max_step = 3.0f;
        if (delta > max_step)
            delta = max_step;
        else if (delta < -max_step)
            delta = -max_step;
        g_cur_angle += delta;

        if (g_cur_angle > BAL_ANGLE_MAX)
            g_cur_angle = BAL_ANGLE_MAX;
        if (g_cur_angle < BAL_ANGLE_MIN)
            g_cur_angle = BAL_ANGLE_MIN;

        /* ---- 动态调速 ---- */
        {
            float ratio = abs_err / 150.0f;
            if (ratio > 1.0f)
                ratio = 1.0f;
            if (ratio < 0.05f)
                ratio = 0.05f;
            uint16_t target_speed = (uint16_t)(20.0f + ratio * (float)(BAL_MOTOR_SPEED - 20));
            if (g_ball_speed_cap > 0 && target_speed > g_ball_speed_cap)
                target_speed = g_ball_speed_cap;
            g_step_motor.speed = target_speed;
        }
        g_step_motor.acc = BAL_MOTOR_ACC;
    } /* end PID block */

    /* ---- 7. 设置待发送角度 (主循环中发送, 不在中断中调用阻塞函数) ---- */
set_pending:
    g_pending_angle = g_cur_angle;
    /* 首次启动时, 叠加轨道水平补偿到初始角度 */
    static uint8_t first_run = 1;
    if (first_run)
    {
        g_pending_angle = BAL_LEVEL_BIAS; /* 上电即置水平补偿角 */
        first_run = 0;
    }
    g_angle_dirty = 1;

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
 * @note  若 DMA 忙则不发送, 保留 dirty 标志供下次重试
 */
void BallBalance_SendMotorCmd(void)
{
    if (!g_paused && g_angle_dirty)
    {
        /* StepMotor_SetAngle 内部已处理 DMA 忙的情况 */
        StepMotor_SetAngle(&g_step_motor, g_pending_angle);
        g_angle_dirty = 0; /* 清除标志 (SetAngle 内部已确保发送成功) */
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

/**
 * @brief 查询是否暂停
 */
uint8_t BallBalance_IsPaused(void)
{
    return g_paused;
}

/**
 * @brief 动态设置目标 X 坐标
 */
void BallBalance_SetTarget(float target_x)
{
    /* 目标改变时重置积分和微分历史, 但保留当前角度 */
    g_target_x = target_x;
    g_i_error = 0.0f;
    g_prev_error = 0.0f;
    /* g_cur_angle 保留 — PID从当前角度平稳过渡到新目标 */
    /* g_filtered_x 不重置 — PID 用实际位置驱动球到新目标 */
}

/**
 * @brief 设置操作模式 (K230 反馈 / 自主内部目标)
 */
/**
 * @brief 设置前馈加速度 (底盘加减速补偿)
 * @param accel 底盘加速度 (cm/s²), 正值=加速, 负值=减速
 */
void BallBalance_SetFeedforward(float accel)
{
    g_ff_accel = accel;
}
