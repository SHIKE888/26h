#include "data_task.h"

//C Include File
#include <string.h>

//STM32 Include File
#include "usart.h"

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"

//BSP Include File
#include "bsp_icm20948.h"
#include "bsp_can.h"

//APP Include File
#include "robot_select_init.h"


//BCC校验函数
uint8_t Calculate_BCC(const uint8_t* checkdata,uint16_t datalen)
{
	char bccval = 0;
	for(uint16_t i=0;i<datalen;i++)
	{
		bccval ^= checkdata[i];
	}
	return bccval;
}

//机器人状态结构体
RobotStatePacket_t robot_state;

static const uint8_t robot_packetLen = sizeof(robot_state);

void RobotDataTransmitTask(void* param)
{
	//获取时基,用于辅助任务能按固定频率运行
	TickType_t preTime = xTaskGetTickCount();
	
	//本任务的控制频率,单位为Hz
	const uint16_t TaskFreq = 50;

	//指定使用的CAN设备
	pCANInterface_t candev = &UserCAN1Dev;
	
	//计算需要发送多少个数据包(数据包规则,第1个字节用于表示包号,后面7个字节用来存放实际数据)
	const uint8_t robot_packetNum = (robot_packetLen/7) + ((robot_packetLen%7)!=0);
	
	//计算最后一个数据包长度
	const uint8_t robot_lastPacketLen = (robot_packetLen%7);
	
	while(1)
	{
		//小车信息
		robot_state.CarYaw = AttitudeVal.yaw;
		robot_state.Vx = RobotControlParam.feedbackVx;
		robot_state.Vy = RobotControlParam.feedbackVy;
		robot_state.Vz = RobotControlParam.feedbackVz;
		
		uint8_t* sendptr = (uint8_t*)&robot_state;
		
		for(uint8_t id=0;id<robot_packetNum;id++)
		{
			uint8_t sendbuffer[8]={0}; //数据包存放
			uint8_t sendlen = 8;       //数据包长度
			sendbuffer[0] = id;
			
			//最后一个包,包号+实际数据 = 需要发送的长度
			if( id == robot_packetNum-1 && robot_lastPacketLen !=0 ) sendlen = robot_lastPacketLen+1; 

			//发送数据包
			memcpy(sendbuffer+1,sendptr+id*7,sendlen-1);
			candev->sendStd(0x201,sendbuffer,sendlen);
		}
		
		/* 延迟指定频率 */
		vTaskDelayUntil(&preTime,pdMS_TO_TICKS( (1.0f/(float)TaskFreq)*1000) );
	}
	
}

