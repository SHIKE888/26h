/**
 * @file    trace.h
 * @brief   8路循迹传感器驱动 (低电平 = 识别到黑线) — 移植 F411
 * @note    引脚定义由 CubeMX 生成于 main.h:
 *          TRACE0(PA4) ~ TRACE3(PA7), TRACE4(PB0)~TRACE5(PB1), TRACE6(PB2), TRACE7(PB10)
 *          已在 MX_GPIO_Init 中配置为输入, 无需额外初始化
 *
 *          ⚠️ 电平说明: 传感器模块输出经反相器(NOT门)处理,
 *             白底(反射强) → 传感器输出低 → 反相器输出高
 *             黑线(反射弱) → 传感器输出高 → 反相器输出低
 *             因此代码中 GPIO 读取后取反, 使得黑线=1, 白底=0
 */

#ifndef TRACE_TRACE_H_
#define TRACE_TRACE_H_

#include "main.h"

/* ---- 位掩码 ---- */
#define TRACE_MASK_ALL 0xFF
#define TRACE_MASK_LEFT 0xF0  /* 左侧4路: trace7~trace4 */
#define TRACE_MASK_RIGHT 0x0F /* 右侧4路: trace3~trace0 */

/* ---- 函数声明 ---- */
void Trace_Init(void);
uint8_t Trace_ReadAll(void);
uint8_t Trace_ReadLeft(void);
uint8_t Trace_ReadRight(void);
int16_t Trace_GetDeviation(uint8_t threshold);

#endif /* TRACE_TRACE_H_ */
