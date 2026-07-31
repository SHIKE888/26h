#ifndef __DATA_TASK_H
#define __DATA_TASK_H

#include <stdint.h>

#pragma pack(1)
//小车 ---> F570 数据包
typedef struct{
	float CarYaw; //小车航向角
	float Vx;     //X轴速度
	float Vy;     //Y轴速度
	float Vz;     //Z轴速度
	float x_point;
	float y_point;
	uint8_t x_flag;
	uint8_t y_flag;
}RobotStatePacket_t;

//F570 ---> 小车 数据包
typedef struct{
	uint8_t StartFlag; //起飞标志位
	float Yaw;      //飞行器航向角
	float height;   //飞行器当前高度
}F570StatePacket_t;
#pragma pack()


extern RobotStatePacket_t robot_state;
extern F570StatePacket_t F570_state;
extern uint16_t g_f570_recv_freq;
extern uint8_t g_f570_onlineFlag;

#endif
