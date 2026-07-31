#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <stdio.h>

//BSP Include File
#include "bsp_stp23L.h"

//串口控制互斥锁
static uint8_t serial_control_lock = 0;
static uint32_t serial_control_tick = 0;

//串口接收完成回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	extern uint8_t BlueToothBuffer;
	extern uint8_t rosbuffer;
	extern uint8_t usart1_buffer;
	
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	//蓝牙使用的串口
	if( huart == &huart2 )
	{
		/* APP写入指令 */
		extern QueueHandle_t g_xQueueBlueTooth;
		
		//中断写队列必须做非空判断,以防开机瞬间被写入导致异常
		if( g_xQueueBlueTooth!=NULL )
			xQueueSendFromISR(g_xQueueBlueTooth,&BlueToothBuffer,&xHigherPriorityTaskWoken);
		
		HAL_UART_Receive_IT(&huart2,&BlueToothBuffer,1);
	}
	
	//ROS使用的串口
	else if( huart == &huart3 )
	{
		extern QueueHandle_t g_xQueueROSserial;
		
		//数值1为最高优先级互斥锁
		serial_control_lock = 1;
		
		if( g_xQueueROSserial!=NULL )
		{
			xQueueSendFromISR(g_xQueueROSserial,&rosbuffer,&xHigherPriorityTaskWoken);
			serial_control_tick = xTaskGetTickCountFromISR();
		}
			
		HAL_UART_Receive_IT(&huart3,&rosbuffer,1);
	}
	
//	//串口1接收缓冲区
//	else if( huart == &huart1 )
//	{
//		extern QueueHandle_t g_xQueueROSserial;
//		
//		//带优先级的互斥锁判断,避免多个串口同时写1个队列
//		// 2为人为规定的优先级,数值越低优先级越高
//		if( 2 <= serial_control_lock )
//		{
//			serial_control_lock=2;
//			if( g_xQueueROSserial!=NULL )
//			{
//				xQueueSendFromISR(g_xQueueROSserial,&usart1_buffer,&xHigherPriorityTaskWoken);
//				serial_control_tick = xTaskGetTickCountFromISR();
//			}
//		}
//		else
//		{
//			if((xTaskGetTickCountFromISR() - serial_control_tick) > 2000)
//				serial_control_lock = 2;
//		}

//		HAL_UART_Receive_IT(&huart1,&usart1_buffer,1);
//	}
	
	//根据具体情况发起调度
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

//DMA传输完成与半完成中断入口函数
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	//检查高优先级任务唤醒
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if( huart == &huart5 && HAL_UART_STATE_READY == huart->RxState ) 
	{	
		//Size存放的内容是DMA已经搬运的数据数量
		
		extern QueueHandle_t g_xQueuestp23L_Ori;
		if( g_xQueuestp23L_Ori!=NULL )
		{
			DMABuf_oridata_stp23L.DataLen = Size;
			xQueueSendFromISR(g_xQueuestp23L_Ori,&DMABuf_oridata_stp23L,&xHigherPriorityTaskWoken);
		}
		
		//直接在中断里解析处理数据，有可能比较耗时，影响其他任务调度的实时性

		//重新启动DMA搬运
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5,DMABuf_oridata_stp23L.Buf,userconfig_STP23L_DMABUF_LEN);
	}
	
	//主动发起调度
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

