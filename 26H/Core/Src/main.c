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
typedef enum
{
    APP_IDLE = 0,
    APP_TRACKING,
    APP_ESTOP,
} AppState;

static volatile AppState g_app_state = APP_IDLE;
static volatile uint8_t g_trace_status = 0;
static volatile uint32_t g_run_ticks = 0; /* 运行计时 (10ms分辨率) */
static uint32_t g_oled_tick = 0;
static uint8_t g_step_motor_enabled = 1; /* 步进电机使能标志 */
static uint32_t g_step_rearm_tick = 0;   /* 使能后延时回零的时间戳 */
static uint8_t g_step_rearm_pending = 0; /* 回零待执行标志 */
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
    if (g_app_state == APP_TRACKING)
        g_run_ticks++;

    /* 循迹读取 */
    g_trace_status = Trace_ReadAll();

    /* 如果正在循迹, 运行循迹控制 + 速度闭环 */
    if (g_app_state == APP_TRACKING)
    {
        /* 十字/岔路保护: 任意≥3路同时为黑 → 急停 */
        uint8_t bits = g_trace_status;
        uint8_t cnt = 0;
        for (uint8_t i = 0; i < 8; i++)
            if (bits & (1U << i))
                cnt++;
        if (cnt >= 3 && cnt <= 5 && g_run_ticks > 100) /* 启动1s内屏蔽ESTOP */
        {
            /* 急停刹车 500ms, 然后滑行停止 */
            Motor_EmergencyBrake(500);
            g_app_state = APP_ESTOP;
            return;
        }

        Track_Process(g_trace_status);
        Motor_SpeedControl_Update();
    }

    /* 滚球平衡控制 (始终运行, 不受 TRACKING/IDLE 状态影响) */
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

        /* ---- KEY_L 使能后非阻塞回零: 延时 200ms 后执行 Home ---- */
        if (g_step_rearm_pending)
        {
            if (HAL_GetTick() - g_step_rearm_tick >= 200)
            {
                g_step_rearm_pending = 0;
                StepMotor_Home(&g_step_motor); /* 回零 */
                BallBalance_Resume();          /* 恢复平衡控制 */
            }
        }

        /* ---- 按键扫描 ---- */
        uint8_t key = Key_Scan();
        uint8_t key_l = KeyL_Scan();

        /* ---- KEY_L 短按: 切换步进电机使能/失能 ---- */
        if (key_l == KEY_EVENT_SHORT)
        {
            g_step_motor_enabled = !g_step_motor_enabled;
            StepMotor_Enable(&g_step_motor, g_step_motor_enabled);
            if (g_step_motor_enabled)
            {
                /* 使能: 暂停平衡, 200ms 后自动回零并恢复平衡 */
                BallBalance_Pause();
                g_step_rearm_tick = HAL_GetTick();
                g_step_rearm_pending = 1;
            }
            else
            {
                /* 失能: 暂停平衡控制, 取消待执行的回零 */
                g_step_rearm_pending = 0;
                BallBalance_Pause();
            }
        }

        /* ---- 状态机 ---- */
        switch (g_app_state)
        {
        case APP_IDLE:
            if (key == KEY_EVENT_SHORT)
            {
                /* 启动纯循迹, 清零计时, 恢复全速 */
                g_run_ticks = 0;
                g_track.base_speed = TRACK_BASE_SPEED_DEFAULT;
                Track_Start();
                Motor_SpeedControl_Init();
                g_app_state = APP_TRACKING;
                OLED_Clear();
            }
            break;

        case APP_TRACKING:
            if (key == KEY_EVENT_LONG)
            {
                /* 长按紧急停机 */
                Track_Stop();
                g_app_state = APP_ESTOP;
                OLED_Clear();
            }
            break;

        case APP_ESTOP:
            Motor_BrakeUpdate(); /* 驱动刹车状态机 */
            if (key == KEY_EVENT_SHORT)
            {
                g_app_state = APP_IDLE;
            }
            break;
        }

        /* ---- OLED 刷新 (每 200ms) ---- */
        if (HAL_GetTick() - g_oled_tick > 200)
        {
            g_oled_tick = HAL_GetTick();
            char buf[22];

            /* 第0行: 8路循迹 + 状态标签 */
            OLED_ShowNum(0, 0, (g_trace_status >> 7) & 1, 1, 12, 0);
            OLED_ShowNum(6, 0, (g_trace_status >> 6) & 1, 1, 12, 0);
            OLED_ShowNum(12, 0, (g_trace_status >> 5) & 1, 1, 12, 0);
            OLED_ShowNum(18, 0, (g_trace_status >> 4) & 1, 1, 12, 0);
            OLED_ShowNum(24, 0, (g_trace_status >> 3) & 1, 1, 12, 0);
            OLED_ShowNum(30, 0, (g_trace_status >> 2) & 1, 1, 12, 0);
            OLED_ShowNum(36, 0, (g_trace_status >> 1) & 1, 1, 12, 0);
            OLED_ShowNum(42, 0, (g_trace_status >> 0) & 1, 1, 12, 0);

            switch (g_app_state)
            {
            case APP_IDLE:
                OLED_ShowString(50, 0, "IDL", 12, 0);
                break;
            case APP_TRACKING:
                OLED_ShowString(50, 0, "RUN", 12, 0);
                break;
            case APP_ESTOP:
                OLED_ShowString(50, 0, "STP", 12, 0);
                break;
            }

            /* 第1-2行: 运行时间 (大字号 8x16, 格式 SS.Ds) */
            {
                uint32_t ds = g_run_ticks; /* 10ms单位 → 0.1秒 */
                uint32_t sec = ds / 100;
                uint32_t ds10 = ds % 100;
                OLED_ShowNum(0, 1, sec, 3, 16, 0);
                OLED_ShowChar(24, 1, '.', 16, 0);
                OLED_ShowNum(32, 1, ds10, 2, 16, 0);
                OLED_ShowChar(48, 1, 's', 16, 0);
            }

            /* 第3行: 左电机A 目标/实际/PWM */
            snprintf(buf, sizeof(buf),
                     "L:%+05d/%+06d/%05d",
                     (int)g_target_speed_a,
                     (int)g_speed_snapshot_a,
                     (int)g_pwm_actual_a);
            OLED_ShowString(0, 3, buf, 12, 0);

            /* 第4行: 右电机B 目标/实际/PWM */
            snprintf(buf, sizeof(buf),
                     "R:%+05d/%+06d/%05d",
                     (int)g_target_speed_b,
                     (int)g_speed_snapshot_b,
                     (int)g_pwm_actual_b);
            OLED_ShowString(0, 4, buf, 12, 0);

            /* 第5行: 误差 + 基础速度 */
            snprintf(buf, sizeof(buf),
                     "E%+05d/%+05d BS%04d",
                     (int)(g_target_speed_a - g_speed_snapshot_a),
                     (int)(g_target_speed_b - g_speed_snapshot_b),
                     (int)g_track.base_speed);
            OLED_ShowString(0, 5, buf, 12, 0);

            /* ---- 第6-7行: 滚球平衡状态 ---- */
            {
                float angle = BallBalance_GetAngle();
                int32_t err = BallBalance_GetError();
                float fx = BallBalance_GetFilteredX();

                snprintf(buf, sizeof(buf), "Bal E%+05ld A%+.1f", (long)err, angle);
                OLED_ShowString(0, 6, buf, 12, 0);

                if (g_k230_data.x != 65535)
                    snprintf(buf, sizeof(buf), "X%3u T%3u F%3.0f", g_k230_data.x, g_k230_data.target, fx);
                else
                    snprintf(buf, sizeof(buf), "X--- T%3u F%3.0f", g_k230_data.target, fx);
                OLED_ShowString(0, 7, buf, 12, 0);
            }

            /* ---- 串口1遥测: 目标速度A,实际速度A,目标速度B,实际速度B ---- */
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
