/**
 * @file    k230_uart.c
 * @brief   K230 视觉模块 UART 接收解析 (USART1 + DMA CIRCULAR + IDLE)
 *
 * 接收格式:
 *   "[array([292, 238,400\n"  → 有目标: X=292, Y=238
 *   "[],400\n"               → 无目标
 *
 * 策略:
 *   1. DMA CIRCULAR 持续接收数据到缓冲区
 *   2. UART IDLE 中断检测帧尾
 *   3. IDLE 时提取 DMA 新数据 → 逐行解析
 */

#include "k230_uart.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>

/* ---- 全局变量 ---- */
volatile K230TrackData g_k230_data = {.x = 65535, .y = 0, .target = 0, .fresh = 0};

/* ---- 接收缓冲 ---- */
static uint8_t g_rx_buf[K230_RX_BUF_SIZE];
static volatile uint16_t g_rx_pos = 0; /* 上次处理后的 DMA 写位置 */
static uint8_t g_line[K230_RX_BUF_SIZE];
static uint8_t g_line_idx = 0;

extern UART_HandleTypeDef huart1;

/* ========================================================================== *
 *  初始化: 启动 DMA CIRCULAR 接收 + IDLE 中断
 * ========================================================================== */
void K230_UART_Init(void)
{
    memset(g_rx_buf, 0, sizeof(g_rx_buf));
    g_rx_pos = 0;
    g_line_idx = 0;

    /* 启动 DMA 循环接收 */
    HAL_UART_Receive_DMA(&huart1, g_rx_buf, K230_RX_BUF_SIZE);

    /* 使能 IDLE 中断 */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

/* ========================================================================== *
 *  解析一行: "[array([X, Y, TGT\n" 或 "[],TGT\n"
 * ========================================================================== */
static void K230_ParseLine(const uint8_t *line, uint16_t len)
{
    if (len < 3 || line[0] != '[')
        return;

    /* 无目标: "[],TGT\n" */
    if (line[1] == ']')
    {
        g_k230_data.x = 65535;
        g_k230_data.y = 0;
        g_k230_data.target = (uint16_t)atoi((const char *)line + 3);
        g_k230_data.fresh = 1;
        return;
    }

    /* 有目标: 找 "([" 后解析 X, Y, TGT */
    const uint8_t *p = line;
    while (p < line + len - 2)
    {
        if (p[0] == '(' && p[1] == '[')
        {
            p += 2;
            uint16_t x = (uint16_t)atoi((const char *)p);
            while (p < line + len && *p != ',')
                p++;
            if (p >= line + len)
                return;
            p++;
            uint16_t y = (uint16_t)atoi((const char *)p);
            while (p < line + len && *p != ',')
                p++;
            if (p >= line + len)
                return;
            p++;
            uint16_t tgt = (uint16_t)atoi((const char *)p);

            g_k230_data.x = x;
            g_k230_data.y = y;
            g_k230_data.target = tgt;
            g_k230_data.fresh = 1;
            return;
        }
        p++;
    }
}

/* ========================================================================== *
 *  从 DMA 缓冲中提取 [start, end) 区间数据并按 '\n' 分割解析
 * ========================================================================== */
static void K230_ExtractLines(uint16_t start, uint16_t end)
{
    while (start != end)
    {
        uint8_t ch = g_rx_buf[start];
        start = (start + 1) % K230_RX_BUF_SIZE;

        if (ch == '\n')
        {
            K230_ParseLine(g_line, g_line_idx);
            g_line_idx = 0;
        }
        else if (g_line_idx < K230_RX_BUF_SIZE - 1)
        {
            g_line[g_line_idx++] = ch;
        }
    }
}

/* ========================================================================== *
 *  IDLE 回调: 在 USART1_IRQHandler 中调用, 处理 IDLE 中断
 *  DMA CIRCULAR 持续运行，不停止
 * ========================================================================== */
void K230_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
        return;

    /* 处理 IDLE */
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);

        /* 读取 DMA 当前写入位置 */
        uint16_t ndtr = __HAL_DMA_GET_COUNTER(huart->hdmarx);
        uint16_t curr_pos = (ndtr < K230_RX_BUF_SIZE)
                                ? (K230_RX_BUF_SIZE - ndtr)
                                : 0;

        /* 提取新数据 */
        if (curr_pos != g_rx_pos)
        {
            K230_ExtractLines(g_rx_pos, curr_pos);
            g_rx_pos = curr_pos;
        }
    }

    /* 处理 DMA 传输错误 (如缓冲区溢出) */
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        (void)huart->Instance->DR;
    }
}
