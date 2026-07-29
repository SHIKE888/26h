/**
 * @file    trace.c
 * @brief   8路循迹传感器驱动 (低电平 = 黑线) — 移植 F411
 * @note    TRACE0~3: PA4~PA7 (连续)
 *          TRACE4~5: PB0~PB1 (连续)
 *          TRACE6: PB2
 *          TRACE7: PB10 (非连续, 需单独读取)
 */

#include "trace.h"

/**
 * @brief 循迹传感器初始化 (读取一次预热)
 */
void Trace_Init(void)
{
    Trace_ReadAll();
}

/**
 * @brief 读取全部8路循迹传感器
 * @return uint8_t 位图 [trace7][trace6]...[trace0]
 *         1 = 黑线
 */
uint8_t Trace_ReadAll(void)
{
    uint8_t value = 0;

    /* TRACE0~3 (PA4~PA7) -> bit[3:0] 取反: 低电平(黑线)=1 */
    value |= ((~(GPIOA->IDR >> 4)) & 0x0F);

    /* TRACE4~5 (PB0~PB1) -> bit[5:4] 取反 */
    value |= (((~(GPIOB->IDR >> 0)) & 0x03) << 4);

    /* TRACE6 (PB2) -> bit[6] 取反 */
    if ((GPIOB->IDR & (1U << 2)) == 0)
        value |= (1U << 6);

    /* TRACE7 (PB10) -> bit[7] 取反 */
    if ((GPIOB->IDR & (1U << 10)) == 0)
        value |= (1U << 7);

    return value;
}

/**
 * @brief 读取左侧4路 (trace7~trace4)
 */
uint8_t Trace_ReadLeft(void)
{
    return (Trace_ReadAll() >> 4) & 0x0F;
}

/**
 * @brief 读取右侧4路 (trace3~trace0)
 */
uint8_t Trace_ReadRight(void)
{
    return Trace_ReadAll() & 0x0F;
}

/**
 * @brief 加权偏差计算 (加权平均法)
 *
 *  传感器布局（俯视车头朝前）:
 *    trace7 trace6 trace5 trace4  |  trace3 trace2 trace1 trace0
 *    左                                        右
 *
 *  权重分配 (中心为0, 最左-3500, 最右+3500):
 *    trace7: -2625
 *    trace6: -1875
 *    trace5: -1125
 *    trace4: -375
 *    trace3: +375
 *    trace2: +1125
 *    trace1: +1875
 *    trace0: +2625
 *
 *  中心间距为750, 超出4个视为丢线
 *
 *  @param threshold 黑线传感器个数阈值 (>threshold则视为丢线)
 *  @return 偏差值: -3500~+3500, -9999=丢线
 */
int16_t Trace_GetDeviation(uint8_t threshold)
{
    static const int16_t weights[8] = {
        /* trace0 */ 2625,
        /* trace1 */ 1875,
        /* trace2 */ 1125,
        /* trace3 */ 375,
        /* trace4 */ -375,
        /* trace5 */ -1125,
        /* trace6 */ -1875,
        /* trace7 */ -2625};

    uint8_t status = Trace_ReadAll();
    int32_t sum = 0;
    uint8_t count = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        if (status & (1 << i))
        {
            sum += weights[i];
            count++;
        }
    }

    /* 丢线判断 */
    if (count > threshold)
    {
        return -9999;
    }

    /* 全白/全黑处理 */
    if (count == 0)
    {
        return -9999; /* 全白, 丢线 */
    }
    if (count >= 7)
    {
        return 0; /* 岔路/十字, 居中行驶 */
    }

    return (int16_t)(sum / count);
}
