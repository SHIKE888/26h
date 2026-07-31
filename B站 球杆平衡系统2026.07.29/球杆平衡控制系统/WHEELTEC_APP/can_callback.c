#include "can.h"

//C Include File
#include <stdio.h>
#include <string.h>

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "queue.h"

//BSP Include File
#include "bsp_can.h"
#include "bsp_RTOSdebug.h"

//APP Include File
#include "RobotControl_task.h"
#include "data_task.h"

//F570飞行器数据接收处理
F570StatePacket_t F570_state; //F570飞行器状态,全局变量
static const uint8_t F570_packetLen = sizeof(F570_state); //包长度
static const uint8_t F570_packetNum = (F570_packetLen/7) + ((F570_packetLen%7)!=0);//数据包号
static const uint16_t F570_RecvDoneMask = (1<<F570_packetNum)-1; //解析完成标志掩码
static uint8_t F570_RecvTempBuffer[F570_packetLen];//数据临时缓冲区
static uint8_t F570_RecvRealState = 0;             //解析包的步骤记录

//CAN总线状态检测
uint8_t g_f570_onlineFlag = 0;
uint16_t g_f570_recv_freq=0;
extern uint8_t g_f570_can_check;

static pRTOSDebugInterface_t debugtimer = &RTOSDebugTimer;
static RTOSDebugPrivateVar debug_pri = { 0 };

// CAN FIFO0中断，关联CAN1
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t recvbuffer[8]={0};			    //接收缓存数组
	HAL_StatusTypeDef HAL_RetVal;		    //判断状态的枚举
	CAN_RxHeaderTypeDef RxMsg;              //接收结构体
	HAL_RetVal=HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxMsg,recvbuffer);//接收邮箱中的数据
	
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	//CAN终端接收成功
	if (HAL_OK==HAL_RetVal)
	{
		if( RxMsg.IDE == CAN_ID_STD )
		{
			//支持0x181位置使用CAN控制
			if( RxMsg.StdId==0x181 )
			{
				RobotControlCMDType_t cmd = {
					.cmdsource = CAN_CMD,
					0,0,0
				};
				
				cmd.Vx = (float)((short)(recvbuffer[0]<<8|recvbuffer[1]))/1000.0f;
				cmd.Vy = (float)((short)(recvbuffer[2]<<8|recvbuffer[3]))/1000.0f;
				cmd.Vz = (float)((short)(recvbuffer[4]<<8|recvbuffer[5]))/1000.0f;
				
				//空闲判断
				uint8_t writeflag=1;
				static uint8_t idleCount = 0;
				if( cmd.Vx==0&&cmd.Vy==0&&cmd.Vz==0 ) idleCount++;
				else idleCount=0;
				if( idleCount>10 ) writeflag=0,idleCount=10;
				
				//非空闲写入队列
				if( writeflag ) WriteRobotControlQueue(&cmd,&xHigherPriorityTaskWoken);	
			}
			
			//F570飞行器数据通过标准帧id 0x202传输
			else if( RxMsg.StdId==0x202 )
			{
				//标记已处理的包号
				F570_RecvRealState |= 1<<recvbuffer[0];
				
				//根据包号将数据存放到对应的数据缓冲区
				memcpy(F570_RecvTempBuffer+recvbuffer[0]*7,recvbuffer+1,RxMsg.DLC-1);
				
				//数据解析完成后进行解析
				if( (F570_RecvRealState&F570_RecvDoneMask)==F570_RecvDoneMask )
				{
					g_f570_onlineFlag=1; //F570在线标志位
					g_f570_can_check=0;  //清空超时计数
					
					F570_RecvRealState=0;//清空标志位等待下一轮解析
					memcpy((uint8_t*)&F570_state,F570_RecvTempBuffer,F570_packetLen);
					
					//计算数据解析的频率
					g_f570_recv_freq = debugtimer->UpdateFreq(&debug_pri);
					
//					//执行控制
//					RobotControlCMDType_t cmd = {
//						.cmdsource = CAN_CMD,
//						0,0,0
//					};
//					
//					cmd.Vx = F570_state.targetVx;
//					cmd.Vy = F570_state.targetVy;
//					cmd.Vz = F570_state.targetVz;
//					
//					//空闲判断
//					uint8_t writeflag=1;
//					static uint8_t idleCount = 0;
//					if( cmd.Vx==0&&cmd.Vy==0&&cmd.Vz==0 ) idleCount++;
//					else idleCount=0;
//					if( idleCount>10 ) writeflag=0,idleCount=10;
//					
//					
//					//非空闲写入队列
//					if( writeflag ) WriteRobotControlQueue(&cmd,&xHigherPriorityTaskWoken);	
				}

			}
			
//			//自动回充的数据写入队列
//			else if( RxMsg.StdId==0x182 )
//			{
//				CANmsgType_t data={0};
//				data.id = RxMsg.StdId;
//				memcpy(data.buffer,recvbuffer,8);
//				if( g_xQueueAutoRecharge!=NULL )
//					xQueueOverwriteFromISR(g_xQueueAutoRecharge,&data,&xHigherPriorityTaskWoken);
//			}

		}
		
		//扩展帧ID数据接收
		else if(RxMsg.IDE == CAN_ID_EXT)
		{
//			printf("CAN1 ExtId:%X\r\n",RxMsg.ExtId);
//			for(uint8_t i=0;i<RxMsg.DLC;i++)
//				printf("%2X\t",recvbuffer[i]);
//			printf("\r\n\r\n");
		}
	}
	
	//根据具体情况发起调度
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


