/**
 * @file    step_motor.c
 * @brief   张大头 Emm_V4.2 步进电机驱动 — 角度模式实现 (USART6)
 * @author  shike888
 * @date    2026-07-31
 *
 * @note    参考 CSDN 文章: STM32步进闭环控制、速度控制（张大头Emm_V4.2驱动器）
 *          协议依据 ZDT_X42S Emm 固件 TTL 串口协议与命令格式
 */

#include "step_motor.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>

/* ========================================================================== *
 *  全局电机实例
 * ========================================================================== */

StepMotor g_step_motor;

/* ========================================================================== *
 *  静态辅助函数
 * ========================================================================== */

/**
 * @brief 判断发送缓冲区是否发生变化
 * @param motor 电机句柄指针
 * @return 1=有变化, 0=无变化
 */
static uint8_t StepMotor_HasChanged(StepMotor *motor)
{
    for (uint8_t i = 0; i < 9; i++)
    {
        if (motor->tx_buf[i] != motor->last_buf[i])
            return 1;
    }
    return 0;
}

/**
 * @brief 更新"上一次发送"缓冲区
 * @param motor 电机句柄指针
 */
static void StepMotor_UpdateLast(StepMotor *motor)
{
    for (uint8_t i = 0; i < 9; i++)
        motor->last_buf[i] = motor->tx_buf[i];
}

/**
 * @brief 通过 USART6 发送位置模式指令帧 (9 字节)
 * @param motor 电机句柄指针
 * @note  仅在数据有变化时才发送，避免重复指令
 */
static void StepMotor_SendPositionCmd(StepMotor *motor)
{
    if (!StepMotor_HasChanged(motor))
        return;

    HAL_UART_Transmit(&huart6, motor->tx_buf, 9, 100);
    StepMotor_UpdateLast(motor);
}

/**
 * @brief 通过 USART6 发送通用短指令
 * @param data  数据缓冲区
 * @param len   数据长度
 */
static void StepMotor_SendCmd(uint8_t *data, uint8_t len)
{
    HAL_UART_Transmit(&huart6, data, len, 100);
}

/* ========================================================================== *
 *  API 实现
 * ========================================================================== */

/**
 * @brief 步进电机初始化
 */
void StepMotor_Init(StepMotor *motor, uint8_t addr, uint16_t speed, uint8_t acc)
{
    /* 清零句柄 */
    memset(motor, 0, sizeof(StepMotor));

    /* 设置参数 */
    motor->addr = addr;
    motor->speed = speed;
    motor->acc = acc;

    /* 构建默认 FD 位置模式帧 (停止状态, 脉冲=0)
     * 帧格式: [Addr] [FD] [Dir|SpeedH] [SpeedL] [Acc] [PulseH] [PulseM] [PulseL] [Chk] */
    motor->tx_buf[0] = addr;
    motor->tx_buf[1] = 0xFD;
    motor->tx_buf[2] = 0x00 | ((speed >> 8) & 0x0F); /* Dir=CW, Speed高4位 */
    motor->tx_buf[3] = speed & 0xFF;                 /* Speed低8位 */
    motor->tx_buf[4] = acc;                          /* 加速度 */
    motor->tx_buf[5] = 0x00;                         /* 脉冲[23:16] */
    motor->tx_buf[6] = 0x00;                         /* 脉冲[15:8] */
    motor->tx_buf[7] = 0x00;                         /* 脉冲[7:0] */
    motor->tx_buf[8] = STEP_MOTOR_CHK_DEFAULT;

    /* 同步 last_buf */
    memcpy(motor->last_buf, motor->tx_buf, 9);

    /* 初始角度归零 */
    motor->now_angle = 0.0f;
}

/**
 * @brief 设置目标绝对角度
 * @note  内部自动计算与当前角度的差值，转换为脉冲后发送
 */
void StepMotor_SetAngle(StepMotor *motor, float angle)
{
    /* 计算角度差值 */
    float error_angle = angle - motor->now_angle;

    if (error_angle == 0.0f)
        return;

    /* 角度 → 脉冲数 */
    int32_t pulse = (int32_t)(error_angle / 360.0f * STEP_MOTOR_PULSE_PER_CIRCLE);

    if (pulse == 0)
        return;

    /* 确定方向 + 取脉冲绝对值 */
    uint8_t dir_nibble;
    uint32_t abs_pulse;

    if (pulse > 0)
    {
        dir_nibble = 0x10; /* CW */
        abs_pulse = (uint32_t)pulse;
    }
    else
    {
        dir_nibble = 0x00; /* CCW */
        abs_pulse = (uint32_t)(-pulse);
    }

    /* 填充 FD 位置模式帧
     * [0]=Addr [1]=FD [2]=Dir|SpeedH [3]=SpeedL [4]=Acc
     * [5]=Pulse[23:16] [6]=Pulse[15:8] [7]=Pulse[7:0] [8]=Chk */
    motor->tx_buf[0] = motor->addr;
    motor->tx_buf[1] = 0xFD;
    motor->tx_buf[2] = dir_nibble | ((motor->speed >> 8) & 0x0F);
    motor->tx_buf[3] = motor->speed & 0xFF;
    motor->tx_buf[4] = motor->acc;
    motor->tx_buf[5] = (abs_pulse >> 16) & 0xFF;
    motor->tx_buf[6] = (abs_pulse >> 8) & 0xFF;
    motor->tx_buf[7] = abs_pulse & 0xFF;
    motor->tx_buf[8] = STEP_MOTOR_CHK_DEFAULT;

    StepMotor_SendPositionCmd(motor);

    /* 更新当前角度 */
    motor->now_angle = angle;
}

/**
 * @brief 电机使能/失能
 * @note  帧格式: [Addr] [F3] [AB] [Enable] [Sync] [Chk]
 */
void StepMotor_Enable(StepMotor *motor, uint8_t enable)
{
    uint8_t cmd[6];
    cmd[0] = motor->addr;
    cmd[1] = 0xF3;
    cmd[2] = 0xAB;
    cmd[3] = enable ? 0x01 : 0x00;
    cmd[4] = 0x00; /* 立即执行 */
    cmd[5] = STEP_MOTOR_CHK_DEFAULT;

    StepMotor_SendCmd(cmd, 6);
}

/**
 * @brief 立即停止电机
 * @note  帧格式: [Addr] [FE] [98] [Sync] [Chk]
 */
void StepMotor_Stop(StepMotor *motor)
{
    uint8_t cmd[5];
    cmd[0] = motor->addr;
    cmd[1] = 0xFE;
    cmd[2] = 0x98;
    cmd[3] = 0x00; /* 立即执行 */
    cmd[4] = STEP_MOTOR_CHK_DEFAULT;

    StepMotor_SendCmd(cmd, 5);
}

/**
 * @brief 设置电机转速
 */
void StepMotor_SetSpeed(StepMotor *motor, uint16_t speed)
{
    motor->speed = speed;
}

/**
 * @brief 设置电机加速度
 */
void StepMotor_SetAcc(StepMotor *motor, uint8_t acc)
{
    motor->acc = acc;
}

/**
 * @brief 当前位置清零 (硬件归零)
 * @note  帧格式: [Addr] [0A] [6D] [Chk]
 */
void StepMotor_ZeroPosition(StepMotor *motor)
{
    uint8_t cmd[4];
    cmd[0] = motor->addr;
    cmd[1] = 0x0A;
    cmd[2] = 0x6D;
    cmd[3] = STEP_MOTOR_CHK_DEFAULT;

    StepMotor_SendCmd(cmd, 4);

    /* 软件角度同步归零 */
    motor->now_angle = 0.0f;
}

/**
 * @brief 触发回零 — 回到绝对坐标零点
 * @note  帧格式: [Addr] [9A] [HomeMode=04] [Sync=00] [Chk]
 *        HomeMode=04 表示回到绝对坐标零点
 *        回零过程中电机会旋转, 完成后自动停止在零点
 */
void StepMotor_Home(StepMotor *motor)
{
    uint8_t cmd[5];
    cmd[0] = motor->addr;
    cmd[1] = 0x9A;
    cmd[2] = 0x04; /* 回到绝对坐标零点 */
    cmd[3] = 0x00; /* 立即执行 */
    cmd[4] = STEP_MOTOR_CHK_DEFAULT;

    StepMotor_SendCmd(cmd, 5);

    /* 软件角度同步归零 */
    motor->now_angle = 0.0f;
}
