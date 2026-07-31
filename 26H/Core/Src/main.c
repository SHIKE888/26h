/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "OLED.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "track.h"
#include "trace.h"
#include "key.h"
#include "step_motor.h"
#include "k230_uart.h"
#include "ball_balance.h"
#include "calibration.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ============================== 功能模式 ================================= */
typedef enum
{
    MODE_TRACK_PURE = 0, /* F1: 纯循迹 (开机默认) */
    MODE_BALL_DEMO,      /* F2: 平衡球自主演示 */
    MODE_DUAL_8S,        /* F3: 双控 8s  */
    MODE_DUAL_30S,       /* F4: 双控 30s */
    MODE_DUAL_CUSTOM,    /* F5: 自定义平衡目标 */
    MODE_COUNT           /* 模式总数 */
} FuncMode;

static char *g_mode_names[MODE_COUNT] = {
    "TRACK", "BALL", "DUAL8", "DUAL30", "CUSTOM"};

/* 简易绝对值 */
#define ABSF(x) ((x) < 0.0f ? -(x) : (x))

/* ============================== 运行状态 ================================= */
typedef enum
{
    STATE_IDLE = 0,    /* 空闲 (显示当前模式和提示) */
    STATE_RUNNING,     /* 运行中 (循迹和/或平衡) */
    STATE_ESTOP,       /* 急停 */
    STATE_CALIBRATION, /* 视觉标定 */
    STATE_ZERO_SET,    /* 步进电机零点设置 */
    STATE_CUSTOM_SAMP, /* F5: 采样目标坐标 */
} RunState;

/* ============================== 全局状态变量 ============================= */
static volatile RunState g_state = STATE_IDLE;
static volatile FuncMode g_mode = MODE_TRACK_PURE; /* 开机默认纯循迹 */
static volatile uint8_t g_trace_status = 0;
static volatile uint32_t g_run_ticks = 0; /* 10ms 运行计时 */
static volatile uint32_t g_run_limit = 0; /* 运行时限 (10ms 单位, 0=无限制) */
static uint32_t g_oled_tick = 0;
static uint8_t g_step_motor_enabled = 1; /* 步进电机使能标志 */
static uint32_t g_step_rearm_tick = 0;
static uint8_t g_step_rearm_pending = 0;

/* 缓启动 */
static int32_t g_ramp_speed = 0;        /* 当前缓启动速度 */
static int32_t g_prev_target_speed = 0; /* 上周期底盘目标速度 (用于前馈) */

/* F5 自定义目标 */
static float g_custom_target_x = 400.0f; /* F5 采样目标 */
static uint8_t g_custom_sampling = 0;    /* 采样进行中 */
static uint8_t g_custom_sample_cnt = 0;  /* 采样计数 */
static float g_custom_sample_sum = 0.0f;
static uint32_t g_custom_sample_tick = 0;

/* F2 自主演示 */
static uint8_t g_ball_demo_active = 0; /* F2 已启动 */
static BallDemoPhase g_ball_demo_phase = BAL_DEMO_IDLE;

/* 全局电机速度上限 (F2 用, 0=不限制) */
uint16_t g_ball_speed_cap = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief TIM1 周期中断回调 (10ms)
 * @note  PSC=99, ARR=9999 → 100MHz/100/10000 = 10ms
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM1)
        return;

    /* 编码器更新 */
    Encoder_Update(10);

    /* 运行计时 (10ms分辨率) */
    if (g_state == STATE_RUNNING && g_mode != MODE_BALL_DEMO)
        g_run_ticks++;

    /* 时限检测 (F3/F4) */
    if (g_state == STATE_RUNNING && g_run_limit > 0 && g_run_ticks >= g_run_limit)
    {
        Track_Stop();
        Motor_SpeedControl_Init();
        BallBalance_Pause();
        g_state = STATE_IDLE;
        OLED_Clear();
        return;
    }

    /* 循迹读取 */
    g_trace_status = Trace_ReadAll();

    /* 循迹 + 速度闭环 (纯循迹 / 双控模式) */
    if (g_state == STATE_RUNNING && g_mode != MODE_BALL_DEMO)
    {
        /* 十字/岔路保护 */
        uint8_t bits = g_trace_status;
        uint8_t cnt = 0;
        for (uint8_t i = 0; i < 8; i++)
            if (bits & (1U << i))
                cnt++;
        if (cnt >= 3 && cnt <= 5 && g_run_ticks > 100)
        {
            if (g_mode == MODE_DUAL_8S || g_mode == MODE_DUAL_30S || g_mode == MODE_DUAL_CUSTOM)
            {
                /* F3/F4/F5: 标记缓减速, 主循环中执行 */
                g_state = STATE_ESTOP;
                g_step_rearm_pending = 2; /* 特殊标记: 缓减速中 */
                g_step_rearm_tick = HAL_GetTick();
                Track_SetBaseSpeed(200);
                return;
            }
            else
            {
                Motor_EmergencyBrake(500);
                g_state = STATE_ESTOP;
                return;
            }
        }

        /* 缓启动 ramp (F3/F4/F5) */
        if (g_mode >= MODE_DUAL_8S && g_ramp_speed < (int32_t)TRACK_BASE_SPEED_DUAL)
        {
            g_ramp_speed += TRACK_RAMP_STEP;
            if (g_ramp_speed > (int32_t)TRACK_BASE_SPEED_DUAL)
                g_ramp_speed = TRACK_BASE_SPEED_DUAL;
            Track_SetBaseSpeed(g_ramp_speed);
        }

        Track_Process(g_trace_status);

        /* 前馈: 计算底盘加速度 → 传给平衡球系统 (平滑版, 避免转向抖动) */
        if (g_mode >= MODE_DUAL_8S)
        {
            int32_t avg_target = (g_target_speed_a + g_target_speed_b) / 2;
            /* 对 avg_target 做 EMA 平滑, 滤除转向差速的高频分量 */
            static float smooth_speed = 0.0f;
            smooth_speed = 0.15f * (float)avg_target + 0.85f * smooth_speed;
            float accel = (smooth_speed - (float)g_prev_target_speed) / 10.0f;
            BallBalance_SetFeedforward(accel);
            g_prev_target_speed = (int32_t)smooth_speed;
        }

        Motor_SpeedControl_Update();
    }

    /* 滚球平衡控制 */
    BallBalance_Tick();
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2C1_Init();
    MX_RTC_Init();
    MX_TIM1_Init();
    MX_USART1_UART_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_USART2_UART_Init();
    MX_USART6_UART_Init();
    /* USER CODE BEGIN 2 */
    /* ---- 用户模块初始化 ---- */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, "26H Boot...", 12, 0);

    Trace_Init();
    Motor_Init();
    Encoder_Init();
    Key_Init();
    Track_Init();
    Motor_SpeedControl_Init();

    /* ---- 张大头步进电机初始化 (USART6, 地址=0x01) ---- */
    StepMotor_Init(&g_step_motor,
                   STEP_MOTOR_DEFAULT_ADDR,
                   STEP_MOTOR_DEFAULT_SPEED,
                   STEP_MOTOR_DEFAULT_ACC);
    StepMotor_Enable(&g_step_motor, 1); /* 使能电机 */

    OLED_Clear();
    OLED_ShowString(0, 0, "Press KEY", 12, 0);
    OLED_ShowString(0, 1, "to START", 12, 0);

    /* ---- K230 视觉模块 UART 接收 ---- */
    K230_UART_Init();

    /* ---- 钢球平衡控制器初始化 (目标 X=400) ---- */
    BallBalance_Init(400.0f);

    /* ---- 标定数据加载 (开机从 Flash 读取) ---- */
    Calibration_Init();

    /* 启动 TIM1 周期中断 (10ms, 需 CubeMX 中 TIM1 ARR=9999) */
    HAL_TIM_Base_Start_IT(&htim1);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        /* ---- 滚球平衡: 发送电机指令 (角度在 TIM1 中断中已计算) ---- */
        BallBalance_SendMotorCmd();

        /* ---- 非阻塞回零 / 缓减速 ---- */
        if (g_step_rearm_pending)
        {
            if (g_step_rearm_pending == 2)
            {
                /* 缓减速: 持续循迹 + 逐帧降速到0, 2秒完成 */
                g_trace_status = Trace_ReadAll();
                Track_Process(g_trace_status);
                Motor_SpeedControl_Update();
                BallBalance_SetFeedforward(0.0f);
                /* 每10ms降3 → 2000/10=200步, 200*3=600, 从600降到0 */
                int32_t cur_speed = (int32_t)g_track.base_speed - TRACK_RAMP_STEP;
                if (cur_speed < 0)
                    cur_speed = 0;
                Track_SetBaseSpeed(cur_speed);
                if (cur_speed == 0)
                {
                    Track_Stop();
                    Motor_SpeedControl_Init();
                    BallBalance_Pause();
                    g_step_rearm_pending = 0;
                    g_state = STATE_IDLE;
                }
            }
            else if (HAL_GetTick() - g_step_rearm_tick >= 200)
            {
                g_step_rearm_pending = 0;
                StepMotor_Home(&g_step_motor);
                BallBalance_Resume();
            }
        }

        /* ---- 按键扫描 ---- */
        uint8_t key = Key_Scan();
        uint8_t key_l = KeyL_Scan();
        uint8_t key_r = KeyR_Scan();

        /* ================================================================ *
         *  全局: 长按 KEY 急停 (所有运行态 + 采样态)
         * ================================================================ */
        if (key == KEY_EVENT_LONG &&
            (g_state == STATE_RUNNING || g_state == STATE_CUSTOM_SAMP))
        {
            if (g_mode == MODE_DUAL_30S || g_mode == MODE_DUAL_CUSTOM)
            {
                /* F4/F5: 缓减速停车 — 降低基础速度让车自然滑停 */
                Track_SetBaseSpeed(200);
                /* 等 1.5s 后彻底停车 */
                for (volatile uint32_t i = 0; i < 150; i++)
                {
                    HAL_Delay(10);
                    BallBalance_SendMotorCmd(); /* 保持平衡球运行 */
                }
            }
            Track_Stop();
            Motor_SpeedControl_Init();
            BallBalance_Pause();
            g_ball_demo_active = 0;
            g_ball_demo_phase = BAL_DEMO_IDLE;
            g_custom_sampling = 0;
            g_state = STATE_IDLE;
            g_oled_tick = 0;
        }

        /* ================================================================ *
         *  状态机
         * ================================================================ */
        switch (g_state)
        {
        /* ---- IDLE: 模式切换 + 启动 ---- */
        case STATE_IDLE:
            if (key == KEY_EVENT_SHORT)
            {
                /* 根据当前模式启动 */
                g_run_ticks = 0;
                switch (g_mode)
                {
                case MODE_TRACK_PURE:
                    /* F1: 纯循迹 */
                    g_track.base_speed = TRACK_BASE_SPEED_DEFAULT;
                    g_track.turn_limit = TRACK_TURN_LIMIT_DEFAULT;
                    PID_Init(&g_track.pid_line,
                             TRACK_LINE_KP_DEFAULT, TRACK_LINE_KI_DEFAULT,
                             TRACK_LINE_KD_DEFAULT, 999999.0f);
                    g_track.pid_line.alpha = TRACK_LINE_ALPHA_DEFAULT;
                    Track_Start();
                    Motor_SpeedControl_Init();
                    BallBalance_Pause();
                    g_run_limit = 0;
                    break;

                case MODE_BALL_DEMO:
                    /* F2: 启动自主演示, 目标从 0cm → +5cm, 限速 */
                    g_ball_speed_cap = 120;                                      /* F2 独立限速 120 RPM */
                    BallBalance_Resume();                                        /* 先用目标400初始化: g_filtered_x=400(真实0cm) */
                    BallBalance_SetTarget(400.0f + BAL_DEMO_TARGET_POS * 40.0f); /* 再改目标到600 */
                    g_ball_demo_active = 1;
                    g_ball_demo_phase = BAL_DEMO_TO_POS;
                    g_run_limit = 0;
                    break;

                case MODE_DUAL_8S:
                    /* F3: 双控 8s — 平衡已在模式切换时启动, 这里启动循迹 */
                    g_ball_speed_cap = 0; /* 恢复全速 */
                    g_track.base_speed = TRACK_BASE_SPEED_DUAL;
                    g_track.turn_limit = TRACK_TURN_LIMIT_DUAL;
                    PID_Init(&g_track.pid_line,
                             TRACK_LINE_KP_DUAL, TRACK_LINE_KI_DUAL,
                             TRACK_LINE_KD_DUAL, 999999.0f);
                    g_track.pid_line.alpha = TRACK_LINE_ALPHA_DEFAULT;
                    g_ramp_speed = 0;
                    g_prev_target_speed = 0;
                    Track_Start();
                    Motor_SpeedControl_Init();
                    g_run_limit = DUAL_TIMEOUT_8S;
                    break;

                case MODE_DUAL_30S:
                    /* F4: 双控 30s */
                    g_ball_speed_cap = 0; /* 恢复全速 */
                    g_track.base_speed = TRACK_BASE_SPEED_DUAL;
                    g_track.turn_limit = TRACK_TURN_LIMIT_DUAL;
                    PID_Init(&g_track.pid_line,
                             TRACK_LINE_KP_DUAL, TRACK_LINE_KI_DUAL,
                             TRACK_LINE_KD_DUAL, 999999.0f);
                    g_track.pid_line.alpha = TRACK_LINE_ALPHA_DEFAULT;
                    g_ramp_speed = 0;
                    g_prev_target_speed = 0;
                    Track_Start();
                    Motor_SpeedControl_Init();
                    g_run_limit = DUAL_TIMEOUT_30S;
                    break;

                case MODE_DUAL_CUSTOM:
                    /* F5: KEY第一次 → 提示放球 */
                    if (!g_custom_sampling)
                    {
                        g_custom_sampling = 1;
                        g_custom_sample_cnt = 0;
                        g_custom_sample_sum = 0.0f;
                        g_custom_sample_tick = 0;
                        g_state = STATE_CUSTOM_SAMP;
                        g_oled_tick = 0;
                        goto skip_run_set;
                    }
                    break;

                default:
                    break;
                }
                g_state = STATE_RUNNING;
            skip_run_set:
                g_oled_tick = 0;
            }
            else if (key_l == KEY_EVENT_SHORT)
            {
                /* KEY_L 短按: 切换到上一个模式 */
                g_mode = (FuncMode)((g_mode == 0) ? (MODE_COUNT - 1) : ((int)g_mode - 1));
                g_oled_tick = 0;
                /* F2/F3/F4: 选中即自动启动零点平衡 (保持在 IDLE, 等待 KEY 启动功能) */
                if (g_mode == MODE_BALL_DEMO || g_mode == MODE_DUAL_8S || g_mode == MODE_DUAL_30S)
                {
                    BallBalance_SetTarget(400.0f);
                    BallBalance_Resume();
                    /* 不改变 g_state, 保持在 IDLE */
                }
                /* F5: 选中即执行回零 */
                if (g_mode == MODE_DUAL_CUSTOM)
                {
                    BallBalance_Pause();
                    StepMotor_Home(&g_step_motor);
                }
            }
            else if (key_r == KEY_EVENT_SHORT)
            {
                /* KEY_R 短按: 切换到下一个模式 */
                g_mode = (FuncMode)(((int)g_mode + 1) % (int)MODE_COUNT);
                g_oled_tick = 0;
                /* F2/F3/F4: 选中即自动启动零点平衡 (保持在 IDLE) */
                if (g_mode == MODE_BALL_DEMO || g_mode == MODE_DUAL_8S || g_mode == MODE_DUAL_30S)
                {
                    BallBalance_SetTarget(400.0f);
                    BallBalance_Resume();
                    /* 不改变 g_state */
                }
                /* F5: 选中即执行回零 */
                if (g_mode == MODE_DUAL_CUSTOM)
                {
                    BallBalance_Pause();
                    StepMotor_Home(&g_step_motor);
                }
            }
            else if (key_l == KEY_EVENT_LONG)
            {
                g_state = STATE_CALIBRATION;
                BallBalance_Pause();
                g_step_rearm_pending = 0;
                StepMotor_Home(&g_step_motor);
                Calibration_Enter();
                g_oled_tick = 0;
            }
            else if (key_r == KEY_EVENT_LONG)
            {
                BallBalance_Pause();
                g_state = STATE_ZERO_SET;
                g_step_rearm_pending = 0;
                StepMotor_Stop(&g_step_motor);
                HAL_Delay(100);
                StepMotor_Enable(&g_step_motor, 0);
                g_step_motor_enabled = 0;
                g_oled_tick = 0;
            }
            break;

        /* ---- RUNNING: 循迹 | 平衡演示 | 双控 ---- */
        case STATE_RUNNING:
        {
            /* F2 自主演示阶段机 */
            if (g_mode == MODE_BALL_DEMO && g_ball_demo_active)
            {
                switch (g_ball_demo_phase)
                {
                case BAL_DEMO_TO_POS:
                    BallBalance_SetTarget(400.0f + BAL_DEMO_TARGET_POS * 40.0f);
                    /* 只要靠近就立即折返, 不需等待稳定 */
                    if (ABSF(BallBalance_GetFilteredX() - (400.0f + BAL_DEMO_TARGET_POS * 40.0f)) < BAL_DEMO_TOLERANCE * 40.0f)
                    {
                        BallBalance_SetTarget(400.0f + BAL_DEMO_TARGET_NEG * 40.0f);
                        g_ball_demo_phase = BAL_DEMO_TO_NEG;
                    }
                    break;
                case BAL_DEMO_TO_NEG:
                    if (ABSF(BallBalance_GetFilteredX() - (400.0f + BAL_DEMO_TARGET_NEG * 40.0f)) < BAL_DEMO_TOLERANCE * 40.0f)
                    {
                        g_ball_demo_phase = BAL_DEMO_BAL_NEG;
                    }
                    break;
                case BAL_DEMO_BAL_NEG:
                    BallBalance_SetTarget(400.0f + BAL_DEMO_TARGET_NEG * 40.0f);
                    break;
                default:
                    break;
                }
            }
            break;
        }

        /* ---- ESTOP: 刹车状态机 ---- */
        case STATE_ESTOP:
            Motor_BrakeUpdate();
            if (key == KEY_EVENT_SHORT)
            {
                g_state = STATE_IDLE;
            }
            break;

        /* ---- 视觉标定 ---- */
        case STATE_CALIBRATION:
        {
            uint8_t cal_ret = Calibration_Process(key, key_l);
            if (cal_ret != 0)
            {
                BallBalance_Resume();
                if (!g_step_motor_enabled)
                {
                    g_step_motor_enabled = 1;
                    StepMotor_Enable(&g_step_motor, 1);
                }
                g_state = STATE_IDLE;
                g_oled_tick = 0;
                if (cal_ret == 1)
                {
                    OLED_ShowString(0, 0, "Cal Saved!", 12, 0);
                    HAL_Delay(800);
                }
                g_oled_tick = 0;
            }
            break;
        }

        /* ---- 零点设置 ---- */
        case STATE_ZERO_SET:
        {
            if (key == KEY_EVENT_SHORT)
            {
                StepMotor_ZeroPosition(&g_step_motor);
                StepMotor_Enable(&g_step_motor, 1);
                g_step_motor_enabled = 1;
                g_state = STATE_IDLE;
                BallBalance_Resume();
                StepMotor_Home(&g_step_motor);
                g_oled_tick = 0;
            }
            else if (key_r == KEY_EVENT_LONG)
            {
                StepMotor_Enable(&g_step_motor, 1);
                g_step_motor_enabled = 1;
                g_state = STATE_IDLE;
                BallBalance_Resume();
                g_oled_tick = 0;
            }
            break;
        }

        /* ---- F5: 自定义目标采样 ---- */
        case STATE_CUSTOM_SAMP:
        {
            if (key == KEY_EVENT_SHORT && !g_custom_sampling)
            {
                /* 开始采样 */
                g_custom_sampling = 1;
                g_custom_sample_cnt = 0;
                g_custom_sample_sum = 0.0f;
                g_custom_sample_tick = 0;
            }
            if (g_custom_sampling)
            {
                uint32_t now = HAL_GetTick();
                if (g_custom_sample_cnt < BAL_CUSTOM_SAMPLE_COUNT)
                {
                    if (now - g_custom_sample_tick >= BAL_CUSTOM_SAMPLE_MS)
                    {
                        g_custom_sample_tick = now;
                        if (g_k230_data.x != 65535)
                        {
                            g_custom_sample_sum += (float)g_k230_data.x;
                            g_custom_sample_cnt++;
                        }
                    }
                }
                else
                {
                    /* 采样完成 → 启动双控 */
                    g_custom_target_x = g_custom_sample_sum / (float)BAL_CUSTOM_SAMPLE_COUNT;
                    BallBalance_SetTarget(g_custom_target_x);
                    BallBalance_Resume();
                    g_track.base_speed = TRACK_BASE_SPEED_DUAL;
                    g_track.turn_limit = TRACK_TURN_LIMIT_DUAL;
                    PID_Init(&g_track.pid_line,
                             TRACK_LINE_KP_DUAL, TRACK_LINE_KI_DUAL,
                             TRACK_LINE_KD_DUAL, 999999.0f);
                    g_track.pid_line.alpha = TRACK_LINE_ALPHA_DEFAULT;
                    g_ramp_speed = 0;
                    g_prev_target_speed = 0;
                    Track_Start();
                    Motor_SpeedControl_Init();
                    g_run_ticks = 0;
                    g_run_limit = DUAL_TIMEOUT_30S;
                    g_custom_sampling = 0;
                    g_state = STATE_RUNNING;
                    g_oled_tick = 0;
                }
            }
            break;
        }
        } /* end switch */

        /* ================================================================ *
         *  OLED 显示 (每 100ms 刷新, 保证流畅)
         * ================================================================ */
        if (HAL_GetTick() - g_oled_tick > 100)
        {
            g_oled_tick = HAL_GetTick();
            char buf[22];
            OLED_Clear();

            /* ---- 标定模式独立显示 ---- */
            if (g_state == STATE_CALIBRATION)
            {
                CalStep cs = Calibration_GetStep();
                uint8_t prog = Calibration_GetSampleProgress();
                switch (cs)
                {
                case CAL_STEP_CENTER:
                    OLED_ShowString(15, 0, "Step 1/3", 12, 0);
                    OLED_ShowString(0, 2, "Place at 0cm", 12, 0);
                    snprintf(buf, sizeof(buf), "Samp %d/%d", prog, CAL_SAMPLE_COUNT);
                    OLED_ShowString(0, 5, (prog > 0) ? buf : "Press KEY", 12, 0);
                    break;
                case CAL_STEP_PLUS:
                    OLED_ShowString(15, 0, "Step 2/3", 12, 0);
                    OLED_ShowString(0, 2, "Place +5cm", 12, 0);
                    snprintf(buf, sizeof(buf), "Samp %d/%d", prog, CAL_SAMPLE_COUNT);
                    OLED_ShowString(0, 5, (prog > 0) ? buf : "Press KEY", 12, 0);
                    break;
                case CAL_STEP_MINUS:
                    OLED_ShowString(15, 0, "Step 3/3", 12, 0);
                    OLED_ShowString(0, 2, "Place -5cm", 12, 0);
                    snprintf(buf, sizeof(buf), "Samp %d/%d", prog, CAL_SAMPLE_COUNT);
                    OLED_ShowString(0, 5, (prog > 0) ? buf : "Press KEY", 12, 0);
                    break;
                case CAL_STEP_RESULT:
                    OLED_ShowString(15, 0, "Cal Done!", 12, 0);
                    snprintf(buf, sizeof(buf), "N0:%.0f N1:%.0f", Calibration_GetResult(0), Calibration_GetResult(1));
                    OLED_ShowString(0, 2, buf, 12, 0);
                    snprintf(buf, sizeof(buf), "N2:%.0f", Calibration_GetResult(2));
                    OLED_ShowString(0, 4, buf, 12, 0);
                    break;
                default:
                    break;
                }
            }
            /* ---- 零点设置独立显示 ---- */
            else if (g_state == STATE_ZERO_SET)
            {
                OLED_ShowString(15, 0, "Zero Set", 12, 0);
                OLED_ShowString(0, 2, "Motor OFF", 12, 0);
                OLED_ShowString(0, 3, "Adjust angle", 12, 0);
                OLED_ShowString(0, 5, "KEY:Set Zero", 12, 0);
                OLED_ShowString(0, 6, "KEY_R:Quit", 12, 0);
            }
            /* ---- F5 采样态显示 ---- */
            else if (g_state == STATE_CUSTOM_SAMP)
            {
                OLED_ShowString(15, 0, "CUSTOM", 12, 0);
                OLED_ShowString(0, 2, "Place Ball", 12, 0);
                OLED_ShowString(0, 3, "then press KEY", 12, 0);
                snprintf(buf, sizeof(buf), "Samp %d/%d", g_custom_sample_cnt, BAL_CUSTOM_SAMPLE_COUNT);
                OLED_ShowString(0, 5, buf, 12, 0);
            }
            /* ---- IDLE/IDLE 模式切换显示 ---- */
            else if (g_state == STATE_IDLE)
            {
                /* 第0行: 模式名 */
                snprintf(buf, sizeof(buf), "F%d: %s", (int)g_mode + 1, g_mode_names[g_mode]);
                OLED_ShowString(0, 0, buf, 12, 0);

                /* 第2行: 操作提示 */
                OLED_ShowString(0, 2, "KEY:Start", 12, 0);
                OLED_ShowString(0, 3, "L/R:Chg Mode", 12, 0);

                /* 第5-6行: 参数提示 */
                switch (g_mode)
                {
                case MODE_TRACK_PURE:
                    OLED_ShowString(0, 5, "Pure Track", 12, 0);
                    OLED_ShowString(0, 6, "No Ball Bal", 12, 0);
                    break;
                case MODE_BALL_DEMO:
                    OLED_ShowString(0, 5, "Ball Demo", 12, 0);
                    OLED_ShowString(0, 6, "Auto +-5cm", 12, 0);
                    break;
                case MODE_DUAL_8S:
                    OLED_ShowString(0, 5, "Track+Bal 8s", 12, 0);
                    OLED_ShowString(0, 6, "FeedFwd ON", 12, 0);
                    break;
                case MODE_DUAL_30S:
                    OLED_ShowString(0, 5, "Track+Bal 30s", 12, 0);
                    OLED_ShowString(0, 6, "FeedFwd ON", 12, 0);
                    break;
                case MODE_DUAL_CUSTOM:
                    OLED_ShowString(0, 5, "Custom Tgt", 12, 0);
                    OLED_ShowString(0, 6, "Place+Sample", 12, 0);
                    break;
                default:
                    break;
                }
            }
            /* ---- 运行态显示 ---- */
            else
            {
                /* 第0行: 8路循迹 + 模式 + 状态 */
                OLED_ShowNum(0, 0, (g_trace_status >> 7) & 1, 1, 12, 0);
                OLED_ShowNum(6, 0, (g_trace_status >> 6) & 1, 1, 12, 0);
                OLED_ShowNum(12, 0, (g_trace_status >> 5) & 1, 1, 12, 0);
                OLED_ShowNum(18, 0, (g_trace_status >> 4) & 1, 1, 12, 0);
                OLED_ShowNum(24, 0, (g_trace_status >> 3) & 1, 1, 12, 0);
                OLED_ShowNum(30, 0, (g_trace_status >> 2) & 1, 1, 12, 0);
                OLED_ShowNum(36, 0, (g_trace_status >> 1) & 1, 1, 12, 0);
                OLED_ShowNum(42, 0, (g_trace_status >> 0) & 1, 1, 12, 0);

                char *mode_str = g_mode_names[g_mode];
                OLED_ShowString(50, 0, mode_str, 12, 0);
                const char *sts = (g_state == STATE_RUNNING) ? "RUN" : (g_state == STATE_ESTOP) ? "STP"
                                                                                                : "   ";
                OLED_ShowString(100, 0, (char *)sts, 12, 0);

                /* 第1-2行: 运行时间 */
                {
                    uint32_t ds = g_run_ticks;
                    uint32_t sec = ds / 100;
                    uint32_t ds10 = ds % 100;
                    OLED_ShowNum(0, 1, sec, 3, 16, 0);
                    OLED_ShowChar(24, 1, '.', 16, 0);
                    OLED_ShowNum(32, 1, ds10, 2, 16, 0);
                    OLED_ShowChar(48, 1, 's', 16, 0);
                }

                /* 第3-4行: 电机状态 */
                if (g_mode != MODE_BALL_DEMO)
                {
                    snprintf(buf, sizeof(buf), "L:%+05d/%+06d", (int)g_target_speed_a, (int)g_speed_snapshot_a);
                    OLED_ShowString(0, 3, buf, 12, 0);
                    snprintf(buf, sizeof(buf), "R:%+05d/%+06d", (int)g_target_speed_b, (int)g_speed_snapshot_b);
                    OLED_ShowString(0, 4, buf, 12, 0);
                }

                /* 第5行: 误差/速度 */
                snprintf(buf, sizeof(buf), "E%+05d/%+05d BS%04d",
                         (int)(g_target_speed_a - g_speed_snapshot_a),
                         (int)(g_target_speed_b - g_speed_snapshot_b),
                         (int)g_track.base_speed);
                OLED_ShowString(0, 5, buf, 12, 0);

                /* 第6-7行: 平衡球状态 (F2-F5) */
                if (g_mode >= MODE_BALL_DEMO)
                {
                    float angle = BallBalance_GetAngle();
                    int32_t err = BallBalance_GetError();
                    float fx = BallBalance_GetFilteredX();
                    snprintf(buf, sizeof(buf), "Bal E%+05ld A%+.0f", (long)err, (double)angle);
                    OLED_ShowString(0, 6, buf, 12, 0);
                    if (g_k230_data.x != 65535)
                        snprintf(buf, sizeof(buf), "X%3u T%3u F%3.0f", g_k230_data.x, g_k230_data.target, (double)fx);
                    else
                        snprintf(buf, sizeof(buf), "X--- T%3u F%3.0f", g_k230_data.target, (double)fx);
                    OLED_ShowString(0, 7, buf, 12, 0);
                }
            }
            /* ---- 串口遥测 (与 OLED 刷新同频) ---- */
            {
                char tele[64];
                int len = snprintf(tele, sizeof(tele),
                                   "%d,%d,%d,%d\r\n",
                                   (int)g_target_speed_a,
                                   (int)g_speed_snapshot_a,
                                   (int)g_target_speed_b,
                                   (int)g_speed_snapshot_b);
                HAL_UART_Transmit(&huart2, (uint8_t *)tele, len, 10);
            }
        }
        /* USER CODE END 3 */
    }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 12;
    RCC_OscInitStruct.PLL.PLLN = 96;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
