#include "robot_select_init.h"
#include "bsp_adc.h"

//定义机器人参数
RobotParmentType_t RobotHardWareParam = { 0 };

//机器人的控制软件参数
RobotControlParmentType_t RobotControlParam = { 
	.en_flag = 1,          //小车使能标志位
	.ErrNum = 0,           //小车报错码
	.defalutSpeed = 500,   //小车默认遥控速度,单位mm/s
	.ChargeMode = 0,       //自动回充模式状态,默认关闭
	.softwareEnflag = 0,   //软件使能位,默认使能
	.LineDiffParam = 50,   //纠偏系数,用于调整走直线效果
	.SecurityLevel = 0 ,   //安全等级,0为最高
	.ImuAssistedFlag = 1,  //默认启用小车走直线时用IMU辅助功能
	.kp = 300,
	.ki = 300,
	.PIDController_A={
		.Pwm=0,
		.LimitOutput = 16700,
		.ClearDone = 1,
	},
	.PIDController_B={
		.Pwm=0,
		.LimitOutput = 16700,
		.ClearDone = 1,
	},
	.PIDController_C={
		.Pwm=0,
		.LimitOutput = 16700,
		.ClearDone = 1,
	},
	.PIDController_D={
		.Pwm=0,
		.LimitOutput = 16700,
		.ClearDone = 1,
	},
	.DebugLevel = 0        //调试等级
};

static void Robot_Init(float wheelspacing, float axlespacing, float omni_turn_radiaus, float gearratio,float Accuracy,float tyre_diameter);

void Robot_Select(void)
{
	//The ADC value is variable in segments, depending on the number of car models. Currently there are 6 car models, CAR_NUMBER=6
	//ADC值分段变量，取决于小车型号数量
	uint16_t Divisor_Mode = 4095/(Number_of_CAR-1);
	
	//车型选择
	RobotHardWareParam.CarType = USER_ADC_Get_AdcBufValue(userconfigADC_CARMODE_CHANNEL)/Divisor_Mode; //Collect the pin information of potentiometer //采集电位器引脚信息	
	
	//TODO:调试暂时固定车型
	RobotHardWareParam.CarType = Omni_Car;
	
	switch(RobotHardWareParam.CarType)
	{
		case Omni_Car:
			Robot_Init(0,0,Omni_Turn_Radiaus_109,HALL_30F,Photoelectric_500,FullDirecion_60);
			break;
		case Mec_Car:
			Robot_Init(MEC_wheelspacing,MEC_axlespacing,0,HALL_30F,13,Mecanum_75);
			break;
	}
	
}

static void Robot_Init(float wheelspacing, float axlespacing, float omni_turn_radiaus, float gearratio,float Accuracy,float tyre_diameter) 
{
	RobotHardWareParam.WheelSpacing = wheelspacing;
	RobotHardWareParam.AxleSpacing = axlespacing;
	RobotHardWareParam.OmniCenterR = omni_turn_radiaus;
	RobotHardWareParam.MotorRation = gearratio;
	RobotHardWareParam.EncoderAccuracy = Accuracy;
	RobotHardWareParam.WheelDiameter = tyre_diameter;
}


