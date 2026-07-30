/**
 * @file    key.h
 * @brief   单按键驱动 (短按/长按) — F411 适配
 * @note    KEY(PA0), 上拉输入, 按下=低电平
 *          在主循环中调用 Key_Scan()
 */

#ifndef KEY_KEY_H_
#define KEY_KEY_H_

#include "main.h"

/* ========================== 返回事件 ================================= */
#define KEY_EVENT_NONE 0
#define KEY_EVENT_SHORT 1
#define KEY_EVENT_LONG 2

/* ========================== 时间参数 (毫秒) ========================== */
#define KEY_DEBOUNCE_MS 30
#define KEY_LONG_PRESS_MS 2000

/* ========================== 函数声明 ================================= */
void Key_Init(void);
uint8_t Key_Scan(void);
uint8_t KeyL_Scan(void);

#endif /* KEY_KEY_H_ */
