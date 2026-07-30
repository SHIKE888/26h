/**
 * @file    OLED.c
 * @brief   0.96" OLED (SSD1306) 驱动实现 — 使用 I2C1，不使用 DMA
 * @author  Unicorn_Li (original), shike888 (移植 F411)
 * @date    2022-07-24
 * @license MIT
 */

#include "OLED.h"

/* ========================================================================== *
 *  SSD1306 初始化命令序列（共 23 字节）
 *  参考芯片数据手册编写
 * ========================================================================== */
uint8_t CMD_Data[] = {
    0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0xA0, 0xC0, 0xDA,
    0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0x8D, 0x14,
    0xAF};

/* ========================================================================== *
 *  I2C + DMA 非阻塞传输支持
 *  通过 OLED.h 中的 OLED_USE_DMA 宏启用
 * ========================================================================== */
#ifdef OLED_USE_DMA

/** @brief DMA 传输完成标志，由 HAL_I2C_MemTxCpltCallback 置位 */
volatile uint8_t OLED_DMA_TX_Completed = 1;

void OLED_DMA_WaitReady(void)
{
    while (OLED_DMA_TX_Completed == 0)
        ;
}

void OLED_DMA_WriteBlock(uint8_t *pData, uint16_t Size)
{
    OLED_DMA_WaitReady();
    OLED_DMA_TX_Completed = 0;
    HAL_I2C_Mem_Write_DMA(&hi2c1, 0x78, 0x40, I2C_MEMADD_SIZE_8BIT, pData, Size);
}

/**
 * @brief I2C 内存写传输完成回调（覆写 HAL 弱函数）
 * @param hi2c I2C 句柄指针
 */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        OLED_DMA_TX_Completed = 1;
    }
}

#endif /* OLED_USE_DMA */

/**
 * @brief OLED 初始化
 * @note  上电后需延迟 200 ms 再发送初始化命令
 */
void OLED_Init(void)
{
    HAL_Delay(200);

    uint8_t i = 0;
    for (i = 0; i < 23; i++)
    {
        OLED_WR_CMD(CMD_Data[i]);
    }
}

/**
 * @brief 向 OLED 发送控制命令（阻塞）
 * @param cmd 命令字节
 * @note  通过 I2C 写入 0x00 控制字节 + 命令字节
 */
void OLED_WR_CMD(uint8_t cmd)
{
    HAL_I2C_Mem_Write(&hi2c1, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 0x100);
}

/**
 * @brief 向 OLED 发送数据（阻塞）
 * @param data 数据字节
 * @note  通过 I2C 写入 0x40 控制字节 + 数据字节
 */
void OLED_WR_DATA(uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c1, 0x78, 0x40, I2C_MEMADD_SIZE_8BIT, &data, 1, 0x100);
}

/**
 * @brief 全屏填充（逐页写入 0xFF）
 * @note  DMA 模式下使用静态缓冲 + 轮询同步
 */
void OLED_On(void)
{
    uint8_t i, n;
#ifdef OLED_USE_DMA
    static uint8_t page_data[128];
    for (i = 0; i < 8; i++)
    {
        OLED_DMA_WaitReady();  // 等待前一次 DMA 完成，避免 I2C 冲突
        OLED_WR_CMD(0xb0 + i); // 设置页地址（0~7）
        OLED_WR_CMD(0x00);     // 设置显示位置—列低地址
        OLED_WR_CMD(0x10);     // 设置显示位置—列高地址
        for (n = 0; n < 128; n++)
            page_data[n] = 1;
        OLED_DMA_TX_Completed = 0;
        HAL_I2C_Mem_Write_DMA(&hi2c1, 0x78, 0x40, I2C_MEMADD_SIZE_8BIT, page_data, 128);
    }
    OLED_DMA_WaitReady(); // 等待最后一页发送完成
#else
    for (i = 0; i < 8; i++)
    {
        OLED_WR_CMD(0xb0 + i); // 设置页地址（0~7）
        OLED_WR_CMD(0x00);     // 设置显示位置—列低地址
        OLED_WR_CMD(0x10);     // 设置显示位置—列高地址
        for (n = 0; n < 128; n++)
            OLED_WR_DATA(1);
    }
#endif
}

/**
 * @brief 清屏（全屏写入 0x00）
 * @note  DMA 模式下使用静态缓冲 + 轮询同步
 */
void OLED_Clear(void)
{
    uint8_t i, n;
#ifdef OLED_USE_DMA
    static uint8_t page_data[128];
    for (i = 0; i < 8; i++)
    {
        OLED_DMA_WaitReady();  // 等待前一次 DMA 完成，避免 I2C 冲突
        OLED_WR_CMD(0xb0 + i); // 设置页地址（0~7）
        OLED_WR_CMD(0x00);     // 设置显示位置—列低地址
        OLED_WR_CMD(0x10);     // 设置显示位置—列高地址
        for (n = 0; n < 128; n++)
            page_data[n] = 0;
        OLED_DMA_TX_Completed = 0;
        HAL_I2C_Mem_Write_DMA(&hi2c1, 0x78, 0x40, I2C_MEMADD_SIZE_8BIT, page_data, 128);
    }
    OLED_DMA_WaitReady(); // 等待最后一页发送完成
#else
    for (i = 0; i < 8; i++)
    {
        OLED_WR_CMD(0xb0 + i); // 设置页地址（0~7）
        OLED_WR_CMD(0x00);     // 设置显示位置—列低地址
        OLED_WR_CMD(0x10);     // 设置显示位置—列高地址
        for (n = 0; n < 128; n++)
            OLED_WR_DATA(0);
    }
#endif
}

/**
 * @brief 开启 OLED 显示（DCDC ON + DISPLAY ON）
 */
void OLED_Display_On(void)
{
    OLED_WR_CMD(0X8D); // SET DCDC命令
    OLED_WR_CMD(0X14); // DCDC ON
    OLED_WR_CMD(0XAF); // DISPLAY ON,打开显示
}

/**
 * @brief 关闭 OLED 显示（DCDC OFF + DISPLAY OFF）
 */
void OLED_Display_Off(void)
{
    OLED_WR_CMD(0X8D); // SET DCDC命令
    OLED_WR_CMD(0X10); // DCDC OFF
    OLED_WR_CMD(0XAE); // DISPLAY OFF，关闭显示
}

/**
 * @brief 设置 OLED 写入起始坐标
 * @param x 列地址 (0 ~ 127)
 * @param y 页地址 (0 ~ 7)
 */
void OLED_Set_Pos(uint8_t x, uint8_t y)
{
    OLED_WR_CMD(0xb0 + y);                 // 设置页地址（0~7）
    OLED_WR_CMD(((x & 0xf0) >> 4) | 0x10); // 设置显示位置—列高地址
    OLED_WR_CMD(x & 0x0f);                 // 设置显示位置—列低地址
}

/**
 * @brief 整数幂运算（内部使用）
 * @param m 底数
 * @param n 指数
 * @return m 的 n 次幂
 */
unsigned int oled_pow(uint8_t m, uint8_t n)
{
    unsigned int result = 1;
    while (n--)
        result *= m;
    return result;
}

/**
 * @brief 在指定位置显示一个 ASCII 字符
 * @param x         起始列坐标 (0 ~ 127)
 * @param y         起始页坐标 (0 ~ 7)
 * @param chr       待显示的 ASCII 字符
 * @param Char_Size 字体大小 (12 = 6×8, 16 = 8×16)
 * @param Color_Turn 反相显示 (1 = 反相, 0 = 正常)
 */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size, uint8_t Color_Turn)
{
    unsigned char c = 0, i = 0;
    c = chr - ' '; // 得到偏移后的值
    if (x > 128 - 1)
    {
        x = 0;
        y = y + 2;
    }
    if (Char_Size == 16)
    {
        OLED_Set_Pos(x, y);
        for (i = 0; i < 8; i++)
        {
            if (Color_Turn)
                OLED_WR_DATA(~F8X16[c * 16 + i]);
            else
                OLED_WR_DATA(F8X16[c * 16 + i]);
        }
        OLED_Set_Pos(x, y + 1);
        for (i = 0; i < 8; i++)
        {
            if (Color_Turn)
                OLED_WR_DATA(~F8X16[c * 16 + i + 8]);
            else
                OLED_WR_DATA(F8X16[c * 16 + i + 8]);
        }
    }
    else
    {
        OLED_Set_Pos(x, y);
        for (i = 0; i < 6; i++)
        {
            if (Color_Turn)
                OLED_WR_DATA(~F6x8[c][i]);
            else
                OLED_WR_DATA(F6x8[c][i]);
        }
    }
}

/**
 * @brief 在指定位置显示字符串
 * @param x         起始列坐标 (0 ~ 127)
 * @param y         起始页坐标 (0 ~ 7)
 * @param chr       待显示的字符串（以 '\0' 结尾）
 * @param Char_Size 字体大小 (12 = 6×8, 16 = 8×16)
 * @param Color_Turn 反相显示 (1 = 反相, 0 = 正常)
 * @note  字体 12 时列步进 6，超 122 列自动换行；字体 16 时列步进 8，超 120 列自动换行
 */
void OLED_ShowString(uint8_t x, uint8_t y, char *chr, uint8_t Char_Size, uint8_t Color_Turn)
{
    uint8_t j = 0;
    while (chr[j] != '\0')
    {
        OLED_ShowChar(x, y, chr[j], Char_Size, Color_Turn);
        if (Char_Size == 12) // 6X8的字体列加6，显示下一个字符
            x += 6;
        else // 8X16的字体列加8，显示下一个字符
            x += 8;

        if (x > 122 && Char_Size == 12) // TextSize6x8如果一行不够显示了，从下一行继续显示
        {
            x = 0;
            y++;
        }
        if (x > 120 && Char_Size == 16) // TextSize8x16如果一行不够显示了，从下一行继续显示
        {
            x = 0;
            y++;
        }
        j++;
    }
}

/**
 * @brief 在指定位置显示无符号整数
 * @param x         起始列坐标 (0 ~ 126)
 * @param y         起始页坐标 (0 ~ 7)
 * @param num       待显示的数值
 * @param len       数字位数（自动消前导零）
 * @param size2     字体大小 (12 = 6×8, 16 = 8×16)
 * @param Color_Turn 反相显示 (1 = 反相, 0 = 正常)
 */
void OLED_ShowNum(uint8_t x, uint8_t y, unsigned int num, uint8_t len, uint8_t size2, uint8_t Color_Turn)
{
    uint8_t t, temp;
    uint8_t enshow = 0;
    for (t = 0; t < len; t++)
    {
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1))
        {
            if (temp == 0)
            {
                OLED_ShowChar(x + (size2 / 2) * t, y, ' ', size2, Color_Turn);
                continue;
            }
            else
                enshow = 1;
        }
        OLED_ShowChar(x + (size2 / 2) * t, y, temp + '0', size2, Color_Turn);
    }
}

/**
 * @brief 在指定位置显示浮点数（支持负数）
 * @param x         起始列坐标 (0 ~ 126)
 * @param y         起始页坐标 (0 ~ 7)
 * @param num       待显示的浮点数值
 * @param z_len     整数部分位数
 * @param f_len     小数部分位数
 * @param size2     字体大小 (12 = 6×8, 16 = 8×16)
 * @param Color_Turn 反相显示 (1 = 反相, 0 = 正常)
 */
void OLED_Showdecimal(uint8_t x, uint8_t y, float num, uint8_t z_len, uint8_t f_len, uint8_t size2, uint8_t Color_Turn)
{
    uint8_t t, temp, i = 0; // i为负数标志位
    uint8_t enshow;
    int z_temp, f_temp;
    if (num < 0)
    {
        z_len += 1;
        i = 1;
        num = -num;
    }
    z_temp = (int)num;
    // 整数部分
    for (t = 0; t < z_len; t++)
    {
        temp = (z_temp / oled_pow(10, z_len - t - 1)) % 10;
        if (enshow == 0 && t < (z_len - 1))
        {
            if (temp == 0)
            {
                OLED_ShowChar(x + (size2 / 2) * t, y, ' ', size2, Color_Turn);
                continue;
            }
            else
                enshow = 1;
        }
        OLED_ShowChar(x + (size2 / 2) * t, y, temp + '0', size2, Color_Turn);
    }
    // 小数点
    OLED_ShowChar(x + (size2 / 2) * (z_len), y, '.', size2, Color_Turn);

    f_temp = (int)((num - z_temp) * (oled_pow(10, f_len)));
    // 小数部分
    for (t = 0; t < f_len; t++)
    {
        temp = (f_temp / oled_pow(10, f_len - t - 1)) % 10;
        OLED_ShowChar(x + (size2 / 2) * (t + z_len) + 5, y, temp + '0', size2, Color_Turn);
    }
    if (i == 1) // 如果为负，就将最前的一位赋值'-'
    {
        OLED_ShowChar(x, y, '-', size2, Color_Turn);
        i = 0;
    }
}

/**
 * @brief 在指定位置显示一个 16×16 汉字
 * @param x         起始列坐标 (0 ~ 112，相邻汉字间隔 16)
 * @param y         起始页坐标 (0 ~ 6，相邻行间隔 2)
 * @param no        汉字在字库 Hzk[] 中的索引编号
 * @param Color_Turn 反相显示 (1 = 反相, 0 = 正常)
 */
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no, uint8_t Color_Turn)
{
    uint8_t t = 0;
    OLED_Set_Pos(x, y);
    for (t = 0; t < 16; t++)
    {
        if (Color_Turn)
            OLED_WR_DATA(~Hzk[2 * no][t]); // 显示汉字的上半部分
        else
            OLED_WR_DATA(Hzk[2 * no][t]); // 显示汉字的上半部分
    }

    OLED_Set_Pos(x, y + 1);
    for (t = 0; t < 16; t++)
    {
        if (Color_Turn)
            OLED_WR_DATA(~Hzk[2 * no + 1][t]); // 显示汉字的下半部分
        else
            OLED_WR_DATA(Hzk[2 * no + 1][t]); // 显示汉字的下半部分
    }
}

/**
 * @brief 在指定矩形区域显示位图（BMP）
 * @param x0        区域左上角列坐标 (0 ~ 127)
 * @param y0        区域左上角页坐标 (0 ~ 7)
 * @param x1        区域右下角列坐标 (1 ~ 128)
 * @param y1        区域右下角页坐标 (1 ~ 8)
 * @param BMP       位图数据指针
 * @param Color_Turn 反相显示 (1 = 反相, 0 = 正常)
 * @note  DMA 模式下每行通过 DMA 非阻塞发送，使用静态缓冲处理反相
 */
void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t *BMP, uint8_t Color_Turn)
{
    uint32_t j = 0;
    uint8_t x = 0, y = 0;

    if (y1 % 8 == 0)
        y = y1 / 8;
    else
        y = y1 / 8 + 1;
#ifdef OLED_USE_DMA
    static uint8_t buf[128];
    uint16_t row_len;
    for (y = y0; y < y1; y++)
    {
        OLED_DMA_WaitReady(); // 等待前一次 DMA 完成，避免 I2C 冲突
        OLED_Set_Pos(x0, y);
        row_len = x1 - x0;
        if (Color_Turn)
        {
            for (x = 0; x < row_len; x++)
                buf[x] = ~BMP[j++];
            OLED_DMA_TX_Completed = 0;
            HAL_I2C_Mem_Write_DMA(&hi2c1, 0x78, 0x40, I2C_MEMADD_SIZE_8BIT, buf, row_len);
        }
        else
        {
            OLED_DMA_TX_Completed = 0;
            HAL_I2C_Mem_Write_DMA(&hi2c1, 0x78, 0x40, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&BMP[j], row_len);
            j += row_len;
        }
    }
    OLED_DMA_WaitReady(); // 等待最后一行发送完成
#else
    for (y = y0; y < y1; y++)
    {
        OLED_Set_Pos(x0, y);
        for (x = x0; x < x1; x++)
        {
            if (Color_Turn)
                OLED_WR_DATA(~BMP[j++]); // 显示反相图片
            else
                OLED_WR_DATA(BMP[j++]); // 显示图片
        }
    }
#endif
}

/**
 * @brief 全屏水平滚动
 * @param direction 滚动方向: 0x27 = 向左, 0x26 = 向右
 */
void OLED_HorizontalShift(uint8_t direction)
{
    OLED_WR_CMD(0x2e);      // 停止滚动
    OLED_WR_CMD(direction); // 设置滚动方向
    OLED_WR_CMD(0x00);      // 虚拟字节设置，默认为0x00
    OLED_WR_CMD(0x00);      // 设置开始页地址
    OLED_WR_CMD(0x07);      // 设置每个滚动步骤之间的时间间隔的帧频
    //  0x00-5帧， 0x01-64帧， 0x02-128帧， 0x03-256帧， 0x04-3帧， 0x05-4帧， 0x06-25帧， 0x07-2帧，
    OLED_WR_CMD(0x07); // 设置结束页地址
    OLED_WR_CMD(0x00); // 虚拟字节设置，默认为0x00
    OLED_WR_CMD(0xff); // 虚拟字节设置，默认为0xff
    OLED_WR_CMD(0x2f); // 开启滚动-0x2f，禁用滚动-0x2e，禁用需要重写数据
}

/**
 * @brief 指定页范围水平滚动
 * @param direction 滚动方向: 0x27 = 向左, 0x26 = 向右
 * @param start     起始页 (0x00 ~ 0x07)
 * @param end       结束页 (0x01 ~ 0x07)
 */
void OLED_Some_HorizontalShift(uint8_t direction, uint8_t start, uint8_t end)
{
    OLED_WR_CMD(0x2e);      // 停止滚动
    OLED_WR_CMD(direction); // 设置滚动方向
    OLED_WR_CMD(0x00);      // 虚拟字节设置，默认为0x00
    OLED_WR_CMD(start);     // 设置开始页地址
    OLED_WR_CMD(0x07);      // 设置每个滚动步骤之间的时间间隔的帧频,0x07即滚动速度2帧
    OLED_WR_CMD(end);       // 设置结束页地址
    OLED_WR_CMD(0x00);      // 虚拟字节设置，默认为0x00
    OLED_WR_CMD(0xff);      // 虚拟字节设置，默认为0xff
    OLED_WR_CMD(0x2f);      // 开启滚动-0x2f，禁用滚动-0x2e，禁用需要重写数据
}

/**
 * @brief 全屏垂直 + 水平滚动
 * @param direction 滚动方向: 0x29 = 右上, 0x2A = 左上
 */
void OLED_VerticalAndHorizontalShift(uint8_t direction)
{
    OLED_WR_CMD(0x2e);      // 停止滚动
    OLED_WR_CMD(direction); // 设置滚动方向
    OLED_WR_CMD(0x00);      // 虚拟字节设置，默认为0x00
    OLED_WR_CMD(0x00);      // 设置开始页地址
    OLED_WR_CMD(0x07);      // 设置每个滚动步骤之间的时间间隔的帧频
    OLED_WR_CMD(0x07);      // 设置结束页地址
    OLED_WR_CMD(0x01);      // 垂直滚动偏移(0x01~0x3F)
    OLED_WR_CMD(0x2f);      // 开启滚动
}

/**
 * @brief 显示模式设置
 * @param mode 0x00 = 正常(黑底白字), 0x01 = 反相(白底黑字)
 */
void OLED_DisplayMode(uint8_t mode)
{
    OLED_WR_CMD(0xA6 | (mode & 0x01));
}

/**
 * @brief 亮度/对比度控制
 * @param intensity 0x00~0xFF (默认 0xCF)
 */
void OLED_IntensityControl(uint8_t intensity)
{
    OLED_WR_CMD(0x81);
    OLED_WR_CMD(intensity);
}
