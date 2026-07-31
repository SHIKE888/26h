#ifndef __ROBOTCONTROL_TASK_H
#define __ROBOTCONTROL_TASK_H

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_armcc.h"

#define PI  3.14159265358979f

//控制指令优先级,数值越小优先级越高
//高优先级控制可打断低优先级控制
enum{
	NONE_CMD   = 0,  //用户不可修改.0表示忽略控制量,以防用户未指定控制源就写入的异常情况。
	BootLoader ,  //用户不可修改,BootLoader为最高优先级的系统控制量,用于小车无线更新程序使用,在 \
	                   程序更新前最高优先级控制量将介入控制小车停止。
	
	//用户自定义区域,优先级由高到低
	GamePad_CMD , //游戏手柄
	RCJOY_CMD   , //航模遥控
	APP_CMD     , //手机APP
	ROS_CMD     , //ROS或串口上位机
	CAN_CMD     , //CAN1接口控制
	Charger_CMD , //自动回充装备控制
	
	UnKnownCMD     //用户不可修改,此枚举需要一直保留在最后一位.目的同样为禁止未指定控制源就写入的异常情况
};

//小车控制队列的数据格式
typedef struct{
	uint8_t cmdsource;
	float Vx;
	float Vy;
	float Vz;
}RobotControlCMDType_t;

/* ==========================================================================
 * 球杆系统调试数据结构
 * --------------------------------------------------------------------------
 * 控制任务每 10ms 把所有关键量写进这个结构体，供两个地方使用：
 *   1) show_task.c 的 OLED 显示（第 0 页）
 *   2) Keil 调试时把 g_rollball_debug 拖进 Watch 窗口实时观察
 *
 * 【新手强烈推荐用 Watch 窗口】比串口打印直观得多：
 *   Keil 菜单 View → Watch Windows → Watch 1，
 *   在空行输入 g_rollball_debug 回车，展开就能看到下面每一项。
 *   全速运行时勾选 View → Periodic Window Update 才会实时刷新。
 * ========================================================================== */
typedef struct
{
	uint16_t angle_adc_raw;      //角位移传感器原始读数(0~4095)，标定零点时看这个
	float ball_distance_raw_mm;  //激光测得的原始距离(mm)，标定零点时看这个
	float angle_feedback_adc;    //去掉零点、乘过符号后的角度：0=水平
	float ball_position_mm;      //去掉零点、乘过符号后的位置：0=导轨中点
	float target_position_mm;    //小球目标位置(mm)，按键可改
	float ball_velocity_mm_s;    //小球速度(mm/s)，稳定后应在 0 附近
	float target_angle_adc;      //位置环算出的目标角度(模式1下借用作方向测试量)
	int32_t stepper_command_hz;  //实际下发的脉冲频率，负号表示反方向
	uint8_t stepper_enabled;     //步进驱动器是否已使能
	uint8_t sensor_valid;        //本轮测距是否有效(0=被挡住/超量程，正在用旧值)
}RollBallDebug_t;

//全局调试快照（定义在 RobotControl_task.c）
extern volatile RollBallDebug_t g_rollball_debug;
//闭环总开关：0=停止 1=运行中。由用户按键 PE0 长按切换。
extern volatile uint8_t g_rollball_control_enabled;

//对外公开使用接口
uint8_t WriteRobotControlQueue(RobotControlCMDType_t* cmd,BaseType_t* woken);
void _System_Reset_FromAPP_RTOS(char uart_recv);

#endif
