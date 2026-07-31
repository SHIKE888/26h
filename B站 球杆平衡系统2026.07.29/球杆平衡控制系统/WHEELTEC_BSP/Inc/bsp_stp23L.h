#ifndef __BSP_STP23L_H
#define __BSP_STP23L_H

/* ==========================================================================
 * 文件作用（小白版说明）
 * --------------------------------------------------------------------------
 * STP23L 是一颗“激光测距（TOF）模块”，它自己不停地测距离，然后通过串口
 * 把测量结果不停地发出来。我们不需要向它发命令，只要“接住”它发来的数据即可。
 *
 * 在本项目里，它被装在水管（导轨）的一端，用来测量“激光头到小球的距离”，
 * 这个距离就是小球在管子上的位置。位置是整个平衡控制的最关键输入。
 *
 * 数据是怎么进单片机的？（一条完整的链路）
 *   STP23L 的 TX 脚
 *      -> 单片机 UART5 的 RX 脚（PD2），波特率 230400
 *      -> 串口 DMA 自动把字节搬到 DMABuf_oridata_stp23L.Buf 数组里
 *      -> 串口空闲中断触发 HAL_UARTEx_RxEventCallback()（在 uart_callback.c）
 *      -> 把这一包原始数据丢进 FreeRTOS 队列 g_xQueuestp23L_Ori
 *      -> STP23L_Task 任务（在 RobotControl_task.c）从队列取出
 *      -> 调用本文件的 stp23L_callback() 做“拆包 + 校验 + 滤波”
 *      -> 最终结果写进全局变量 g_readonly_distance（单位 mm）
 *      -> 控制任务 RobotControlTask 直接读 g_readonly_distance 做闭环
 *
 * 关键词解释：
 *   DMA   ：一个“搬运工”硬件。串口收到字节后，不打扰 CPU，自动写进内存数组。
 *   空闲中断：串口线上一段时间没有新字节了，硬件就报告“这一包收完了”。
 *   队列  ：FreeRTOS 提供的“传送带”，中断里放数据，任务里取数据，安全解耦。
 * ========================================================================== */

#include <stdint.h>

//stp23L传感器 DMA数据缓冲区分配
//含义：给 DMA 准备一个 200 字节的“接货筐”。一帧完整数据约 195 字节，
//      留一点余量。这个值不要随意改小，否则一帧数据会被截断。
#define userconfig_STP23L_DMABUF_LEN 200

//stp23L DMA原始数据存放结构体
//说明：Buf 是 DMA 搬进来的“生数据”（还没解析），
//      DataLen 是本次实际收到了多少字节（由空闲中断告诉我们）。
typedef struct {
	uint8_t Buf[userconfig_STP23L_DMABUF_LEN];
	uint16_t DataLen;
}OriData_STP23L_t;

//1个点云数据包里包含的信息。一帧数据共有12个点云数据
//#pragma pack(1) 的意思是“结构体成员紧挨着排列，不要为了对齐插空字节”。
//必须这样写！因为下面要用 memcpy 把串口收到的字节流直接盖到结构体上，
//只有“1 字节对齐”才能保证每个成员正好落在协议规定的位置。
#pragma pack(1)
typedef struct {
    int16_t distance;   //距离（单位 mm，这是我们唯一真正关心的量）
    uint16_t noise;     //环境噪声（环境光越强越大，仅供参考）
    uint32_t peak;      //强度（回波信号强度，越大说明反射越好）
    uint8_t confidence; //置信度（模块自己评估这个点靠不靠谱）
    uint32_t intg;      //积分次数
    int16_t reftof;     //温度表征值
}LidarPointTypedef;

//一帧（一整包）数据的完整结构。总长度 = 4+1+1+2+2 + 12*15 + 4 + 1 = 195 字节。
//模块一帧里给 12 个测量点，相当于连测 12 次，这样我们可以做中值滤波去掉毛刺。
typedef struct {
    uint32_t Head;      //帧头,4个0xAA
		uint8_t dev_addr;   //设备地址
		uint8_t cmdcode;    //命令码
		uint16_t diff_addr; //偏移地址
		uint16_t datalen;   //数据长度
		LidarPointTypedef point[12];//12个数据测量点
		uint32_t timestamp;  //时间戳
		uint8_t crc_code;    //校验码（累加和，用来判断数据有没有传坏）
}STP23LSensorTypedef;

#pragma pack()   //恢复默认对齐方式，别影响后面其它文件的结构体

//DMA 直接往里写的原始数据缓冲区（定义在 bsp_stp23L.c）
extern OriData_STP23L_t DMABuf_oridata_stp23L;

//解析函数：喂给它一包原始字节，它内部用状态机拆包。
//返回 1 表示“本次成功解析出一整帧且校验通过”，返回 0 表示还没凑齐一帧。
uint8_t stp23L_callback(OriData_STP23L_t* buffer);

//对外唯一的“测距结果”：小球距离，单位 mm。
//volatile 表示“这个变量可能在别的地方（另一个任务）被改”，
//告诉编译器每次都要真的去内存读，不要偷懒缓存到寄存器里。
extern volatile float g_readonly_distance;

#endif /* __BSP_STP23L_H */


