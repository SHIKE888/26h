/* ==========================================================================
 * bsp_stp23L.c —— STP23L 激光测距模块的“翻译官”
 * --------------------------------------------------------------------------
 * 本文件只干两件事：
 *   1) 把串口收到的一串杂乱字节，按协议拼成一帧完整数据（用状态机）；
 *   2) 对一帧里的 12 个测距点做筛选 + 中值滤波，得到一个干净的距离值，
 *      写入全局变量 g_readonly_distance（单位 mm）供控制任务使用。
 *
 * 为什么要状态机？
 *   串口是“一个字节一个字节”来的，而且 DMA 每次交给我们的一包数据，
 *   起点不一定正好是帧头。所以要像“找门牌号”一样：
 *   先找到帧头（连续 4 个 0xAA），再按顺序数出后面每个字段的字节数，
 *   数满一帧后校验，校验通过才认为这帧数据可信。
 * ========================================================================== */

#include "bsp_stp23L.h"

#include <stdio.h>
#include <string.h>

#include "bsp_RTOSdebug.h"

//创建一个存放STP23L数据的结构体对象
//这是 DMA 的“落地筐”：串口硬件收到的字节被 DMA 自动写进这里的 Buf 数组。
OriData_STP23L_t DMABuf_oridata_stp23L;

//STP23L filtered distance used directly by the controller.
//【全局输出】滤波后的距离，单位 mm。控制任务 RobotControlTask 直接读它。
//写在这里（STP23L_Task 任务里），读在那里（控制任务里），所以要加 volatile。
volatile float g_readonly_distance = 0.0f;

//创建一个stp23L模块对象
//stp23l_sensor      ：解析完成后的“结构化数据”，可以直接 .point[0].distance 取值
//stp23l_bufferlen   ：一帧数据的字节数（= sizeof 结构体 = 195 字节）
//stp23l_buffer      ：拼帧用的临时缓冲，凑满一帧后整体 memcpy 到 stp23l_sensor
static STP23LSensorTypedef stp23l_sensor;
static const uint8_t stp23l_bufferlen = sizeof(STP23LSensorTypedef);
static uint8_t stp23l_buffer[stp23l_bufferlen];

//一帧里有几个测距点（协议固定 12 个）
#define STP23L_POINT_COUNT               12U
//12 个点里至少要有几个“看起来正常”，这一帧才算数；不够就整帧丢掉
#define STP23L_MIN_VALID_POINT_COUNT      6U
//认为合理的距离范围（mm）。超出这个范围的点判定为噪声/无效点直接扔掉。
//注意：这个上限 300mm 要根据你的导轨实际长度来改！
#define STP23L_DISTANCE_MIN_MM            1.0f
#define STP23L_DISTANCE_MAX_MM          300.0f

/**
 * @brief  求一组数的“中值”（把数据从小到大排好后取中间那个）
 * @param  values 数据数组（注意：函数内部会直接把它排序，原数组会被打乱）
 * @param  count  数据个数
 * @retval 中值
 *
 * 为什么用中值而不是平均值？
 *   平均值怕“个别极端值”。比如 12 个点里有 1 个因为反光突然变成 5mm，
 *   平均值会被明显拉偏；而中值只看排序后中间那个，单个毛刺完全不影响。
 *   对于小球位置这种“不能抖”的信号，中值滤波效果好得多。
 *
 * 这里用的是“插入排序”：像打扑克整理手牌，
 * 每次拿一张新牌，往左边已经排好的牌里插到合适位置。
 * 数据量只有 12 个，插入排序又短又快，完全够用。
 */
static float STP23L_Median(float *values, uint8_t count)
{
	uint8_t i;

	//从第 2 个元素开始，逐个往前面已排好序的部分里“插队”
	for(i = 1U; i < count; i++)
	{
		float value = values[i];   //当前要插入的这张“牌”
		uint8_t j = i;

		//只要左边那个比我大，就把它往右挪一格，给我腾位置
		while((j > 0U) && (values[j - 1U] > value))
		{
			values[j] = values[j - 1U];
			j--;
		}
		values[j] = value;         //腾好位置后放进去
	}

	//奇数个：正中间那个就是中值
	if((count & 1U) != 0U)
	{
		return values[count / 2U];
	}

	//偶数个：中间两个取平均
	return (values[(count / 2U) - 1U] + values[count / 2U]) * 0.5f;
}

/**
 * @brief  对刚解析出来的一帧数据做筛选 + 中值滤波，更新 g_readonly_distance
 * @param  无（直接用文件内的 stp23l_sensor）
 * @retval 无
 *
 * 处理流程：
 *   1. 遍历 12 个测距点，只留下落在 [1mm, 300mm] 区间内的“正常点”；
 *   2. 如果正常点少于 6 个，说明这一帧质量太差（比如小球被挡住/超出量程），
 *      直接 return，什么都不改 —— 于是 g_readonly_distance 自动保持上一次的值，
 *      控制器不会突然收到一个乱跳的数字；
 *   3. 正常点够多时，取中值写入 g_readonly_distance。
 */
static void STP23L_UpdateFilteredDistance(void)
{
	float valid_points[STP23L_POINT_COUNT];   //装“正常点”的小篮子
	uint8_t valid_count = 0U;                 //正常点计数
	uint8_t i;

	for(i = 0U; i < STP23L_POINT_COUNT; i++)
	{
		//协议里 distance 是 int16_t（毫米），转成 float 方便后面计算
		float distance_mm = (float)stp23l_sensor.point[i].distance;

		//范围判断：明显不合理的点（0、负数、超远）不要
		if((distance_mm >= STP23L_DISTANCE_MIN_MM) &&
		   (distance_mm <= STP23L_DISTANCE_MAX_MM))
		{
			valid_points[valid_count++] = distance_mm;
		}
	}

	//有效点太少时整帧丢弃，输出自动保持上一次结果。
	if(valid_count < STP23L_MIN_VALID_POINT_COUNT)
	{
		return;
	}

	//只做帧内中值；每收到一帧立即更新，不增加跨帧等待时间。
	//【重要】不做跨帧的滑动平均，是因为平均会引入“延迟”，
	//而球杆系统是不稳定系统，传感器延迟会直接让闭环变得难以稳定。
	g_readonly_distance = STP23L_Median(valid_points, valid_count);
}

/**
 * @brief  STP23L 原始字节流解析（状态机拆包）
 * @param  buffer 一包由 DMA 收到的原始数据（Buf + 实际长度 DataLen）
 * @retval 1 = 本次成功解析出一整帧并且 CRC 校验通过；0 = 还没凑齐一帧
 *
 * 【状态机是什么？】
 *   可以理解成“流水线上的工位”。数据一个字节一个字节流过来，
 *   当前在哪个工位（state_m），就按那个工位的规矩处理这个字节，
 *   处理够了就切到下一个工位。收完一帧后回到第一个工位重新开始。
 *
 * 【为什么这些变量要写 static？】
 *   因为一帧 195 字节可能被 DMA 分成好几包送进来，
 *   这次函数返回后，下次进来必须“接着上次的进度”继续拼，
 *   static 变量在函数退出后不会消失，正好保存进度。
 *
 * 【一帧的字节布局】（和 STP23LSensorTypedef 完全对应）
 *   AA AA AA AA | 设备地址(1) | 命令码(1) | 偏移地址(2) | 数据长度(2)
 *   | 12个点云数据(12*15=180) + 时间戳(4) = 184 | CRC(1)
 *   合计 4+1+1+2+2+184+1 = 195 字节
 */
uint8_t stp23L_callback(OriData_STP23L_t* buffer)
{
	uint8_t ready = 0;              //返回值：本次有没有成功解析出完整一帧

	//辅助变量（static：跨多次调用保持“拼帧进度”）
	static uint8_t last_recv = 0;   //上一个字节是什么（找连续 0xAA 帧头要用）
	static uint8_t TmpCnt = 0;      //当前工位已经数了几个字节
	static uint8_t datacount = 0;   //已经往 stp23l_buffer 里存了多少字节
	static uint8_t crc = 0;         //边收边累加，最后和帧尾的校验字节比较

	//状态机的状态内容（各个“工位”的名字）
	enum{
		WaitHead = 0, //接收帧头
		WaitDevAddr,  //收设备地址
		WaitCmdCode,  //收命令码
		WaitDiffAddr, //收偏移地址（2 字节）
		WaitDataLen,  //收数据长度（2 字节）
		WaitDATA,     //收数据体（184 字节：12个点 + 时间戳）
		CRCCheck      //收校验字节并核对
	};

	//状态机（当前处在哪个工位）
	static uint8_t state_m = WaitHead;

	//处理数据：把这一包里的每个字节依次喂进状态机
	for(uint8_t i=0;i<buffer->DataLen;i++)
	{
		switch( state_m )
		{
			case WaitHead:
				//找帧头：协议规定帧头是连续 4 个 0xAA。
				//这里的判断是“当前字节和上一个字节都是 0xAA 就 +1”，
				//所以 TmpCnt 数到 3 时，实际已经连续见到 4 个 0xAA。
				if( buffer->Buf[i]==0xAA &&last_recv==0xAA ) TmpCnt++;
				else TmpCnt = 0;   //中间断了就重新找
				//成功收到帧头
			  if( TmpCnt==3 )
				{
					//帧头本身没有存进缓冲区（判断时是逐个比对的），
					//所以这里手动把 4 个 0xAA 补写进去，保证结构体前 4 字节正确
					stp23l_buffer[0]=0xAA;stp23l_buffer[1]=0xAA;
					stp23l_buffer[2]=0xAA;stp23l_buffer[3]=0xAA;
					TmpCnt=0;
					state_m = WaitDevAddr;   //进入下一个工位
					datacount=4;             //从第 4 个字节位置继续往后存
				}
				break;

			case WaitDevAddr:
				//设备地址：只有 1 字节，收完立刻进入下一个工位
				stp23l_buffer[datacount++]=buffer->Buf[i];
				crc+=buffer->Buf[i];
				state_m = WaitCmdCode;
				break;
			
			case WaitCmdCode:
				//命令码：1 字节
				stp23l_buffer[datacount++]=buffer->Buf[i];
				crc+=buffer->Buf[i];   //从帧头之后开始，每个字节都要累加进校验和
				state_m = WaitDiffAddr;
				break;

			case WaitDiffAddr:
				//偏移地址：2 字节，所以要数够 2 个才切换工位
				stp23l_buffer[datacount++]=buffer->Buf[i];
				crc+=buffer->Buf[i];
				TmpCnt++;
				if( TmpCnt==2 )
				{
					TmpCnt=0;
					state_m = WaitDataLen;
				}
				break;

			case WaitDataLen:
				//数据长度：2 字节
				stp23l_buffer[datacount++]=buffer->Buf[i];
				crc+=buffer->Buf[i];
				TmpCnt++;
				if( TmpCnt==2 )
				{
					TmpCnt=0;
					state_m = WaitDATA;
				}
				break;

			case WaitDATA:
				//数据体：12 个点 × 15 字节 + 4 字节时间戳 = 184 字节
				stp23l_buffer[datacount++]=buffer->Buf[i];
				crc+=buffer->Buf[i];
				TmpCnt++;
				if( TmpCnt==184 )
				{
					TmpCnt=0;
					state_m = CRCCheck;
				}
				break;

			case CRCCheck:
				//最后 1 字节是校验码：把它和我们自己累加出来的 crc 比对
				stp23l_buffer[datacount++]=buffer->Buf[i];
				if( crc == buffer->Buf[i] )
				{
					//校验通过,执行数据解析
					//memcpy 把 195 字节原样“盖”到结构体上。
					//能这么干的前提就是头文件里写了 #pragma pack(1)。
					memcpy((uint8_t*)&stp23l_sensor,stp23l_buffer,stp23l_bufferlen);

					//马上做筛选 + 中值滤波，更新 g_readonly_distance
					STP23L_UpdateFilteredDistance();

					//数据就绪
					ready=1;

					//下面是调试用的打印，需要看原始测距频率和距离时可以打开
//					uint16_t freq=0;
//					freq = debug->UpdateFreq(&priv);
//					printf("freq:%d , distance=%.3f\r\n",freq,g_readonly_distance);
				}
				//注意：校验失败就直接丢弃这一帧，不做任何更新（宁可不更新也不要错数据）

				//复位所有变量，回到第一个工位，准备接收下一帧
				TmpCnt=0;
				datacount=0;
				crc=0;
				state_m = WaitHead;

				//【注意】这里故意没写 break，会“穿透”到 default。
				//因为 default 里只有一个 break，效果和直接 break 一样，不影响功能。

			default:
				break;
		}

		//记下本字节，供下一轮找帧头时比较“连续两个 0xAA”
		last_recv = buffer->Buf[i];
	}

	return ready;
}


