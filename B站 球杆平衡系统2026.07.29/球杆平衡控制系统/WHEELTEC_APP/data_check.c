#include "main.h"

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"

//APP Include File
#include "data_task.h"

uint8_t g_f570_can_check = 0;

void DataCheckTask(void* param)
{
	//获取时基,用于辅助任务能按固定频率运行
	TickType_t preTime = xTaskGetTickCount();
	
	//本任务的控制频率,单位为Hz
	const uint16_t TaskFreq = 2;
	
	while(1)
	{
		//F570 设备CAN总线超时
		g_f570_can_check++;
		if( g_f570_can_check >= TaskFreq )
		{
			g_f570_recv_freq = 0;
			g_f570_onlineFlag = 0;
		}
		
		
		/* 延迟指定频率 */
		vTaskDelayUntil(&preTime,pdMS_TO_TICKS( (1.0f/(float)TaskFreq)*1000) );
	}
}

