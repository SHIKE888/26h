# 26H STM32F411 循迹小车 — 项目文档

## 硬件平台

- **主控**: STM32F411CEU6 (HSE 25MHz → SYSCLK 100MHz)
- **电机驱动**: TB6612 (双路PWM+方向控制)
- **编码器**: 500线 GMR ×2 (4倍频 = 2000脉冲/转)
- **循迹传感器**: 8路红外 (带反相器, 低电平=黑线)
- **显示**: 0.96" OLED SSD1306 (I2C1, 128×64)
- **按键**: PA0 (短按启动/解除, 长按急停)

## 引脚映射

| 功能 | GPIO | 外设 | 备注 |
|------|------|------|------|
| OLED_SCL | PB8 | I2C1 | |
| OLED_SDA | PB9 | I2C1 | |
| MA1 | PB12 | GPIO | 电机A方向1 |
| MA2 | PB13 | GPIO | 电机A方向2 |
| MB1 | PB14 | GPIO | 电机B方向1 |
| MB2 | PB15 | GPIO | 电机B方向2 |
| MA_PWM | PB3 | TIM2_CH2 | 电机A PWM |
| MB_PWM | PA15 | TIM2_CH1 | 电机B PWM |
| MA_ENC_A | PB4 | TIM3_CH1 | 电机A编码器A相 |
| MA_ENC_B | PB5 | TIM3_CH2 | 电机A编码器B相 |
| MB_ENC_A | PB6 | TIM4_CH1 | 电机B编码器A相 |
| MB_ENC_B | PB7 | TIM4_CH2 | 电机B编码器B相 |
| TRACE0 | PA4 | GPIO | 循迹0 (最右) |
| TRACE1 | PA5 | GPIO | 循迹1 |
| TRACE2 | PA6 | GPIO | 循迹2 |
| TRACE3 | PA7 | GPIO | 循迹3 |
| TRACE4 | PB0 | GPIO | 循迹4 |
| TRACE5 | PB1 | GPIO | 循迹5 |
| TRACE6 | PB2 | GPIO | 循迹6 |
| TRACE7 | PB10 | GPIO | 循迹7 (最左) |
| KEY | PA0 | GPIO | 控制按键 |

## 定时器

| 定时器 | 功能 | 配置 | 周期 |
|--------|------|------|------|
| TIM1 | 控制主循环 | PSC=99, ARR=9999 | 10ms |
| TIM2 | 电机PWM | PSC=0, ARR=7199 | ~13.3kHz |
| TIM3 | 编码器A | TI12 编码器模式 | - |
| TIM4 | 编码器B | TI12 编码器模式 | - |

## 状态机

```
                    ┌──────────┐
        short press  │ APP_IDLE │
     ┌──────────────│  "IDL"    │◀─────────┐
     │              └──────────┘           │
     ▼                              short press
┌───────────┐                          │
│APP_TRACKING│  long press    ┌────────┴──┐
│   "RUN"    │──────────────▶│ APP_ESTOP  │
└───────────┘                │   "STP"    │
     │                       └───────────┘
     │  ≥4路黑线急停
     └─────────────────────────▶ APP_ESTOP
```

## OLED 显示布局

```
00000000 IDL          ← 行0: 8路循迹位图 + 状态标签
 10.52s               ← 行1-2: 运行时间 (8×16大字号)
L:+0500/+000498/03200 ← 行3: 左电机 目标/实际/PWM
R:+0500/+000512/03150 ← 行4: 右电机 目标/实际/PWM
E+0002/-0012 BS0600   ← 行5: 误差 + 基础速度
```

## 串口遥测

- **USART2**, 波特率115200
- 格式: `targetA, actualA, targetB, actualB\r\n`
- 频率: 每200ms

## 快速调参 (`track.h`)

| 宏 | 默认值 | 说明 |
|----|--------|------|
| `TRACK_BASE_SPEED_DEFAULT` | 600 | 基础速度 (编码器原始值) |
| `TRACK_TURN_LIMIT_DEFAULT` | 1500 | 转弯差速上限 |
| `TRACK_LINE_KP_DEFAULT` | 0.075f | 循迹比例增益 |
| `TRACK_LINE_KI_DEFAULT` | 0.0002f | 循迹积分增益 |
| `TRACK_LINE_KD_DEFAULT` | 0.8f | 循迹微分增益 |

## 速度闭环 (`motor.h`)

| 宏 | 默认值 | 说明 |
|----|--------|------|
| `SPEED_PID_KP` | 0.5f | 速度比例增益 |
| `SPEED_PID_KI` | 0.1f | 速度积分增益 |
| `SPEED_PID_KD` | 0.0f | 速度微分增益 |
| `MOTOR_A_DIR_INVERT` | 0 | 电机A方向反转 |
| `MOTOR_B_DIR_INVERT` | 0 | 电机B方向反转 |

## 编码器 (`encoder.h`)

| 宏 | 默认值 | 说明 |
|----|--------|------|
| `ENCODER_A_INVERT` | 0 | 编码器A极性反转 |
| `ENCODER_B_INVERT` | 0 | 编码器B极性反转 |
| `ENCODER_SPEED_ALPHA` | 0.2f | 速度低通滤波 (0=无滤波) |

## 速度衰减策略

- 0~15秒: 100% 全速
- 15~16秒: 线性过渡到 50%
- 16秒+: 每秒再降 1%, 下限 40%
- 重新启动时恢复全速

## 编译配置 (Keil MDK)

1. Include Paths 添加: `User`, `User/OLED`, `User/Motor`, `User/Encoder`, `User/Control`, `User/Trace`, `User/Key`, `User/delay`
2. 添加所有 `User/**/*.c` 源文件
3. CubeMX: TIM1 ARR=9999, 开启 Update Interrupt
4. 调试模式: Serial Wire (SWD)
