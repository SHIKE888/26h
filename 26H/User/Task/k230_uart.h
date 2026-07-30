/**
 * @file    k230_uart.h
 * @brief   K230 视觉模块 UART 接收 (USART1 + DMA + IDLE)
 *          解析格式: "[array([X, Y, TGT\n" 或 "[],TGT\n"
 */

#ifndef K230_UART_H_
#define K230_UART_H_

#include "stm32f4xx_hal.h"

/* ---- 缓冲区大小 ---- */
#define K230_RX_BUF_SIZE 128

/* ---- 解析后的跟踪数据 ---- */
typedef struct
{
    uint16_t x;      /* 钢球 X 坐标 (0~65534 = 有效, 65535 = 无目标) */
    uint16_t y;      /* 钢球 Y 坐标 */
    uint16_t target; /* 目标 X 坐标 */
    uint8_t fresh;   /* 有新数据标记, 外部读取后清零 */
} K230TrackData;

extern volatile K230TrackData g_k230_data;

/* ---- API ---- */
void K230_UART_Init(void);
void K230_UART_IRQHandler(UART_HandleTypeDef *huart);
void K230_UART_Poll(void);

#endif /* K230_UART_H_ */
