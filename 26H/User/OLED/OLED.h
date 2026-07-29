/**
 * @file    OLED.h
 * @brief   0.96" OLED (SSD1306) 驱动头文件 — 使用 I2C1，不使用 DMA
 * @author  Unicorn_Li (original), shike888 (移植 F411)
 * @date    2022-07-24
 * @license MIT
 */

#ifndef OLED_OLED_H_
#define OLED_OLED_H_

#include "stm32f4xx_hal.h"
#include "oledfont.h"

/* ---- 字体大小常量 ---- */
#define OLED_8X16 16 /**< 8×16 字体 */
#define OLED_6X8 12  /**< 6×8 字体 */

/* ========================================================================== *
 *  I2C 传输模式选择
 *
 *  取消下方 #define 的注释以启用 I2C + DMA 非阻塞传输模式，
 *  可大幅提升批量数据（清屏、刷图）的刷新效率。
 *  保持注释状态则使用原始的阻塞传输模式，兼容性更好。
 * ========================================================================== */
// #define OLED_USE_DMA

#ifdef OLED_USE_DMA
/** @brief DMA 传输完成标志 */
extern volatile uint8_t OLED_DMA_TX_Completed;

/** @brief 通过 DMA 非阻塞方式发送一批数据字节 */
void OLED_DMA_WriteBlock(uint8_t *pData, uint16_t Size);

/** @brief 等待上一次 DMA 传输完成 */
void OLED_DMA_WaitReady(void);
#endif

/* ---- I2C 底层读写 ---- */
void OLED_WR_CMD(uint8_t cmd);
void OLED_WR_DATA(uint8_t data);

/* ---- 初始化与显示控制 ---- */
void OLED_Init(void);
void OLED_Clear(void);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Set_Pos(uint8_t x, uint8_t y);
void OLED_On(void);

/* ---- 字符 / 字符串 / 数值显示 ---- */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size, uint8_t Color_Turn);
void OLED_ShowString(uint8_t x, uint8_t y, char *chr, uint8_t Char_Size, uint8_t Color_Turn);
void OLED_ShowNum(uint8_t x, uint8_t y, unsigned int num, uint8_t len, uint8_t size2, uint8_t Color_Turn);
void OLED_Showdecimal(uint8_t x, uint8_t y, float num, uint8_t z_len, uint8_t f_len, uint8_t size2, uint8_t Color_Turn);
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no, uint8_t Color_Turn);

/* ---- 位图（BMP）绘制 ---- */
void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t *BMP, uint8_t Color_Turn);

/* ---- 硬件滚动与显示效果 ---- */
void OLED_HorizontalShift(uint8_t direction);
void OLED_Some_HorizontalShift(uint8_t direction, uint8_t start, uint8_t end);
void OLED_VerticalAndHorizontalShift(uint8_t direction);
void OLED_DisplayMode(uint8_t mode);
void OLED_IntensityControl(uint8_t intensity);

/** I2C 句柄外部引用 (F411: I2C1, PB8=SCL, PB9=SDA) */
extern I2C_HandleTypeDef hi2c1;

#endif /* OLED_OLED_H_ */
