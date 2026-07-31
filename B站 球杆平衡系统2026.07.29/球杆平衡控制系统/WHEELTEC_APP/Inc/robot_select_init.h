#ifndef __ROBOT_SELECT_INIT_H
#define __ROBOT_SELECT_INIT_H

#include <stdint.h>

//Motor_gear_ratio
//电机减速比
#define   HALL_30F    30

//Number_of_encoder_lines
//编码器精度
#define	  Photoelectric_500 500
#define	  Hall_13           13

////////////////////////// 全向轮系列单独 ////////////////////////////
//Parameter of kinematics analysis of omnidirectional trolley
//全向轮小车专有运动学参数
#define X_PARAMETER    (sqrt(3)/2.0f)               
#define Y_PARAMETER    (0.5f)    
#define L_PARAMETER    (1.0f)

//Rotation radius of omnidirectional trolley
//全向轮小车旋转半径
#define   Omni_Turn_Radiaus_109 0.109
#define   Omni_Turn_Radiaus_164 0.164
#define   Omni_Turn_Radiaus_180 0.180
#define   Omni_Turn_Radiaus_290 0.290

//Omni wheel tire diameter series
//轮径全向轮直径系列
#define	  FullDirecion_60  0.060
#define	  FullDirecion_75  0.075
#define	  FullDirecion_127 0.127
#define	  FullDirecion_152 0.152
#define	  FullDirecion_203 0.203
#define	  FullDirecion_217 0.217
////////////////////////// 全向轮系列 END ////////////////////////////


//Wheelspacing, Mec_Car is half wheelspacing
//轮距 麦轮是一半
#define MEC_wheelspacing         0.0930
#define V550_MEC_wheelspacing    0.115

//Axlespacing, Mec_Car is half axlespacing
//轴距 麦轮是一半
#define MEC_axlespacing           0.085
#define V550_MEC_axlespacing      0.079

//Mecanum wheel tire diameter series
//麦轮轮胎直径
#define		Mecanum_60  0.060f
#define		Mecanum_75  0.075f
#define		Mecanum_100 0.100f
#define		Mecanum_127 0.127f
#define		Mecanum_152 0.152f


typedef struct{
	uint8_t CarType;        //车型
	float WheelSpacing;    //轮距
	float AxleSpacing;     //轴距
	float WheelDiameter;   //轮子直径
	float MotorRation;     //电机减速比
	float EncoderAccuracy;//编码器减速比
	float OmniCenterR;    //全向轮轮子到车中心距离
	float AkmMinTurnR;    //阿克曼最小转弯半径
}RobotParmentType_t;


//车型枚举
enum CAR_MODE{
	Mec_Car = 0, 
	Omni_Car, 
	Akm_Car, 
	Diff_Car, 
	FourWheel_Car, 
	Tank_Car,
	Mec_Car_V550,
	FourWheel_Car_V550,
	Number_of_CAR
};


typedef struct{
	float Bias;
	float LastBias;
	int Pwm;
	int LimitOutput;
	uint8_t ClearDone;
}PIDController;


//机器人硬件参数
extern RobotParmentType_t RobotHardWareParam;

//车轮变量,包含车轮目标速度和车轮实际速度
typedef struct{
	float target;
	float feedback;
}RobotMotorType_t;

//机器人控制参数
typedef struct{
	uint8_t en_flag;         //小车全局使能标志位,1使能可控制 0失能禁止控制
	uint32_t ErrNum ;        //小车报错码
	short defalutSpeed;     //小车默认遥控速度,单位mm/s
	uint8_t Enkeystate;      //小车硬件使能开关的状态,1开关弹起 0开关按下
	float Vol;              //小车电池电压
	uint8_t ChargeMode;      //自动回充模式, 1开启自动回充,0关闭自动回充
	uint8_t SecurityLevel;   //安全等级 0:若小车没有持续收到目标速度,则会主动停止 1:小车将保持最后一次的目标速度响应  
	uint8_t softwareEnflag;  //软件急停标志位,设1可使小车进入急停状态
	uint32_t LineDiffParam;  //机器人纠偏系数，0-100可调整
	uint8_t ImuAssistedFlag; //imu辅助小车走直线标志位. 0无赋值,设置1时,小车被控制时,将通过imu的数据辅助小车走直线
	uint8_t DebugLevel;      //小车报错调试等级. 0正常控制, 置1时小车将通过串口1、蓝牙串口汇报错误信息
	uint8_t LedTickState;    //小车板载LED提示状态
	
	uint8_t ParkingMode;     //私有变量,小车驻车模式标志位（用户勿操作）
	
	//机器人车轮目标值与反馈值
	RobotMotorType_t MotorA;  //车轮的目标速度和反馈速度存放于此
	RobotMotorType_t MotorB;
	RobotMotorType_t MotorC;
	RobotMotorType_t MotorD;
	RobotMotorType_t MotorE;
	RobotMotorType_t MotorF;
	
	//机器人三轴实际速度存放于此,由运动学正解获得
	float feedbackVx;
	float feedbackVy;
	float feedbackVz;
	
	//PID控制器
	
	//电机控制的pid参数
	float kp;
	float ki;
	
	//电机PID控制器
	PIDController PIDController_A;
	PIDController PIDController_B;
	PIDController PIDController_C;
	PIDController PIDController_D;
	
	
}RobotControlParmentType_t;

void Robot_Select(void);
extern RobotControlParmentType_t RobotControlParam;

#endif

