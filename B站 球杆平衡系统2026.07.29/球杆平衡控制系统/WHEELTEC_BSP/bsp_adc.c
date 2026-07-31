/* ==========================================================================
 * bsp_adc.c —— ADC + DMA 采样
 * --------------------------------------------------------------------------
 * 工作方式（“不用管，一直转”）：
 *   上电调用一次 ADC_Userconfig_Init()，之后 ADC 就在硬件层面连续转换，
 *   DMA 自动把结果一个接一个写进下面的数组，写满了自动从头再来（循环模式）。
 *   CPU 全程不参与，任何时候想要数据，直接调 Get 函数取数组平均值即可。
 *
 * 为什么要取平均？
 *   单次 ADC 读数总会有一点随机跳动（噪声）。对 80 次采样取平均，
 *   噪声被明显削弱，读数更稳。代价是响应稍慢一点点，但对角度反馈完全够用。
 * ========================================================================== */

#include "bsp_adc.h"
#include "adc.h"

#define userconfig_ADCDMA_BUF_LEN 80 //设置dma搬运某个通道的个数（每个通道缓存 80 个采样点）

//ADC1 的数据落地区：二维数组。因为 ADC1 配了 2 个通道扫描转换，
//DMA 会按 [通道0, 通道1, 通道0, 通道1, ...] 的顺序交替写入，
//正好对应 g_Adc1Buf[第几轮][第几个通道] 的排布。
static uint16_t g_Adc1Buf[userconfig_ADCDMA_BUF_LEN][2] = { 0 };
//ADC2 只有 1 个通道（PA1 角位移传感器），所以是一维数组
static uint16_t g_Adc2Buf[userconfig_ADCDMA_BUF_LEN] = { 0 };

/**
 * @brief  启动 ADC 采集（只需在开机初始化时调用一次）
 */
void ADC_Userconfig_Init(void)
{
	//启动ADC采集并设置DMA传输
	//第三个参数是“要搬运多少个数据”，这里用 sizeof 自动算：
	//  ADC1：80 行 × 2 列 = 160 个
	//  ADC2：80 个
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)g_Adc1Buf,sizeof(g_Adc1Buf)/sizeof(g_Adc1Buf[0][0]));
	HAL_ADC_Start_DMA(&hadc2,(uint32_t*)g_Adc2Buf,sizeof(g_Adc2Buf)/sizeof(g_Adc2Buf[0]));
}

/**
 * @brief  取 ADC1 指定通道的平均值
 * @param  channel 0=电池电压(PB0)  1=车型识别(PB1)
 * @retval 0~4095 的平均读数；参数非法时返回 0
 */
uint16_t USER_ADC_Get_AdcBufValue(uint8_t channel)
{
	uint32_t tmp = 0;   //用 32 位累加，防止 80 个 4095 相加溢出（80*4095≈32.8万）

	//索引判断防止数组溢出
	if( channel > 1 ) return 0;

	for(uint8_t i=0;i<userconfig_ADCDMA_BUF_LEN;i++)
	{
		tmp += g_Adc1Buf[i][channel];
	}

	return tmp/userconfig_ADCDMA_BUF_LEN;   //求平均
}

/**
 * @brief  取 ADC2 的平均值 —— 【球杆系统的角度反馈就是从这里来的】
 * @retval 0~4095，对应角位移传感器输出的 0~3.3V
 *
 * 使用位置：RobotControl_task.c 里
 *   uint16_t angle_adc_raw = USER_ADC2_Get_AdcBufValue();
 * 然后减去零点宏 ROLLBALL_ANGLE_ADC_ZERO，就得到“偏离水平多少”。
 */
uint16_t USER_ADC2_Get_AdcBufValue(void)
{
	uint32_t tmp = 0;

	for(uint8_t i=0;i<userconfig_ADCDMA_BUF_LEN;i++)
	{
		tmp += g_Adc2Buf[i];
	}

	return tmp/userconfig_ADCDMA_BUF_LEN;
}

////将12位ADC2平均值转换为PA1引脚电压
//float USER_ADC2_Get_Voltage(void)
//{
//	return (float)USER_ADC2_Get_AdcBufValue() * 3.3f / 4095.0f;
//}
