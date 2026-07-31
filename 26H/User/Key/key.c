/**
 * @file    key.c
 * @brief   单按键驱动实现 — F411 适配
 * @note    KEY(PA0), 上拉输入, 按下=低电平
 *          Key_Scan() 每次调用不阻塞, 依赖 HAL_GetTick()
 */

#include "key.h"

/* ========================== 状态机 =================================== */
typedef enum
{
    KS_IDLE,     /* 空闲 */
    KS_DEBOUNCE, /* 消抖 */
    KS_PRESSED,  /* 按下中 */
    KS_LONG,     /* 长按已触发 */
} KeyState;

static KeyState ks = KS_IDLE;
static uint32_t press_tick = 0;

static KeyState ks_l = KS_IDLE;
static uint32_t press_tick_l = 0;

static KeyState ks_r = KS_IDLE;
static uint32_t press_tick_r = 0;

/* ========================== 初始化 =================================== */
void Key_Init(void)
{
    ks = KS_IDLE;
    press_tick = 0;
    ks_l = KS_IDLE;
    press_tick_l = 0;
    ks_r = KS_IDLE;
    press_tick_r = 0;
}

/* ========================== 扫描 ===================================== */
uint8_t Key_Scan(void)
{
    uint8_t pressed = (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET);
    uint32_t now = HAL_GetTick();
    uint8_t event = KEY_EVENT_NONE;

    switch (ks)
    {
    case KS_IDLE:
        if (pressed)
        {
            ks = KS_DEBOUNCE;
            press_tick = now;
        }
        break;

    case KS_DEBOUNCE:
        if ((now - press_tick) >= KEY_DEBOUNCE_MS)
        {
            if (pressed)
            {
                ks = KS_PRESSED;
            }
            else
            {
                ks = KS_IDLE;
            }
        }
        break;

    case KS_PRESSED:
        if (!pressed)
        {
            event = KEY_EVENT_SHORT;
            ks = KS_IDLE;
        }
        else if ((now - press_tick) >= KEY_LONG_PRESS_MS)
        {
            event = KEY_EVENT_LONG;
            ks = KS_LONG;
        }
        break;

    case KS_LONG:
        if (!pressed)
        {
            ks = KS_IDLE;
        }
        break;
    }

    return event;
}

/* ========================== KEY_L 扫描 (PA1) ========================= */

uint8_t KeyL_Scan(void)
{
    uint8_t pressed = (HAL_GPIO_ReadPin(KEY_L_GPIO_Port, KEY_L_Pin) == GPIO_PIN_RESET);
    uint32_t now = HAL_GetTick();
    uint8_t event = KEY_EVENT_NONE;

    switch (ks_l)
    {
    case KS_IDLE:
        if (pressed)
        {
            ks_l = KS_DEBOUNCE;
            press_tick_l = now;
        }
        break;

    case KS_DEBOUNCE:
        if ((now - press_tick_l) >= KEY_DEBOUNCE_MS)
        {
            if (pressed)
                ks_l = KS_PRESSED;
            else
                ks_l = KS_IDLE;
        }
        break;

    case KS_PRESSED:
        if (!pressed)
        {
            event = KEY_EVENT_SHORT;
            ks_l = KS_IDLE;
        }
        else if ((now - press_tick_l) >= KEY_LONG_PRESS_MS)
        {
            event = KEY_EVENT_LONG;
            ks_l = KS_LONG;
        }
        break;

    case KS_LONG:
        if (!pressed)
            ks_l = KS_IDLE;
        break;
    }

    return event;
}

/* ========================== KEY_R 扫描 (PA8) ========================= */

uint8_t KeyR_Scan(void)
{
    uint8_t pressed = (HAL_GPIO_ReadPin(KEY_R_GPIO_Port, KEY_R_Pin) == GPIO_PIN_RESET);
    uint32_t now = HAL_GetTick();
    uint8_t event = KEY_EVENT_NONE;

    switch (ks_r)
    {
    case KS_IDLE:
        if (pressed)
        {
            ks_r = KS_DEBOUNCE;
            press_tick_r = now;
        }
        break;

    case KS_DEBOUNCE:
        if ((now - press_tick_r) >= KEY_DEBOUNCE_MS)
        {
            if (pressed)
                ks_r = KS_PRESSED;
            else
                ks_r = KS_IDLE;
        }
        break;

    case KS_PRESSED:
        if (!pressed)
        {
            event = KEY_EVENT_SHORT;
            ks_r = KS_IDLE;
        }
        else if ((now - press_tick_r) >= KEY_LONG_PRESS_MS)
        {
            event = KEY_EVENT_LONG;
            ks_r = KS_LONG;
        }
        break;

    case KS_LONG:
        if (!pressed)
            ks_r = KS_IDLE;
        break;
    }

    return event;
}
