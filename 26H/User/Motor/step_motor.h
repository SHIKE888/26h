/**
 * @file    step_motor.h
 * @brief   张大头 Emm_V4.2 步进电机驱动 — 角度模式 (USART6)
 * @author  shike888
 * @date    2026-07-31
 *
 * @note    协议参考 ZDT_X42S Emm 固件 TTL 串口协议
 *          角度 → 脉冲转换: 360° = 3200 pulse (1.8°电机, 16细分)
 *          硬件连接: PA11=USART6_TX → 驱动器 R/A/H
 *                    PA12=USART6_RX ← 驱动器 T/B/L
 */

#ifndef STEP_MOTOR_H_
#define STEP_MOTOR_H_

#include "stm32f4xx_hal.h"

/* ========================================================================== *
 *  常量定义
 * ========================================================================== */

/** @brief 一圈对应的脉冲数 (1.8° 步进电机, 16 细分) */
#define STEP_MOTOR_PULSE_PER_CIRCLE 3200.0f

/** @brief 默认校验字节 (固定 6B 模式) */
#define STEP_MOTOR_CHK_DEFAULT 0x6B

/**
 * @brief 校验模式选择
 *        0 = 固定 0x6B (ChecksumMode=00, 驱动器默认)
 *        1 = 动态 XOR  (ChecksumMode=01, 标准模式, 推荐)
 */
#define STEP_MOTOR_CHK_XOR 1

/** @brief 角度模式默认转速 (RPM) */
#define STEP_MOTOR_DEFAULT_SPEED 500

/** @brief 角度模式默认加速度 (0x00~0xFF, 越大加减速越快) */
#define STEP_MOTOR_DEFAULT_ACC 0x70

/** @brief 默认电机地址 */
#define STEP_MOTOR_DEFAULT_ADDR 0x01

/* ========================================================================== *
 *  类型定义
 * ========================================================================== */

/** @brief 电机转动方向 */
typedef enum
{
    STEP_MOTOR_DIR_CW = 0x00,  /**< 顺时针 (正转) */
    STEP_MOTOR_DIR_CCW = 0x01, /**< 逆时针 (反转) */
} StepMotor_Dir;

/** @brief 步进电机控制句柄 */
typedef struct
{
    uint8_t addr;       /**< 电机地址 (0x01~0xFF) */
    uint16_t speed;     /**< 当前转速 (RPM) */
    uint8_t acc;        /**< 当前加速度 */
    float now_angle;    /**< 当前累计角度 (°) */
    uint8_t tx_buf[13]; /**< 发送缓冲区 (FD帧=13字节) */
} StepMotor;

/* ========================================================================== *
 *  全局电机实例声明
 * ========================================================================== */

extern StepMotor g_step_motor;
extern volatile uint8_t g_step_motor_tx_done;

/* ========================================================================== *
 *  API 函数声明
 * ========================================================================== */

/**
 * @brief 步进电机初始化
 * @param motor  电机句柄指针
 * @param addr   电机地址 (0x01~0xFF, 需与驱动器设置一致)
 * @param speed  默认转速 (RPM, 范围 0~3000)
 * @param acc    默认加速度 (0x00~0xFF)
 */
void StepMotor_Init(StepMotor *motor, uint8_t addr, uint16_t speed, uint8_t acc);

/**
 * @brief 设置目标绝对角度
 * @param motor  电机句柄指针
 * @param angle  目标角度 (°), 正负均可, 自动计算相对脉冲
 * @note         内部自动计算与当前角度的差值并发送位置指令
 */
void StepMotor_SetAngle(StepMotor *motor, float angle);

/**
 * @brief 电机使能/失能
 * @param motor  电机句柄指针
 * @param enable 0=失能, 1=使能
 */
void StepMotor_Enable(StepMotor *motor, uint8_t enable);

/**
 * @brief 立即停止电机
 * @param motor  电机句柄指针
 */
void StepMotor_Stop(StepMotor *motor);

/**
 * @brief 设置电机转速 (仅影响后续角度指令)
 * @param motor  电机句柄指针
 * @param speed  转速 (RPM, 0~3000)
 */
void StepMotor_SetSpeed(StepMotor *motor, uint16_t speed);

/**
 * @brief 设置电机加速度 (仅影响后续角度指令)
 * @param motor  电机句柄指针
 * @param acc    加速度 (0x00~0xFF)
 */
void StepMotor_SetAcc(StepMotor *motor, uint8_t acc);

/**
 * @brief 当前位置清零 (硬件归零)
 * @param motor  电机句柄指针
 */
void StepMotor_ZeroPosition(StepMotor *motor);

/**
 * @brief 触发回零 (回到绝对坐标零点)
 * @param motor  电机句柄指针
 * @note  帧格式: [Addr] [9A] [HomeMode=04] [Sync=00] [Chk]
 *        回零完成需等待数秒, 调用后建议延时 3~10s 再操作
 */
void StepMotor_Home(StepMotor *motor);

#endif /* STEP_MOTOR_H_ */
