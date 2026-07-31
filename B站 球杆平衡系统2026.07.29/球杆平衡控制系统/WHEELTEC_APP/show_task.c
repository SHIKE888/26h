#include "show_task.h"

//C Include File

//FreeRTOS Include File
#include "FreeRTOS.h"
#include "task.h"

//STM32 Inclue File
#include "gpio.h"

//BSP Include File
#include "bsp_oled.h"
#include "bsp_RTOSdebug.h"
#include "bsp_adc.h"
#include "bsp_icm20948.h"

//APP Include File
#include "RobotControl_task.h"
#include "robot_select_init.h"
#include "data_task.h"

//私有变量禁止修改
extern volatile uint8_t RobotControl_CMDsource;

static uint8_t page = 0;

static pOLEDInterface_t oled = &UserOLED;

static void RollBallInfoShow(void)
{
	uint16_t angle_adc_raw = g_rollball_debug.angle_adc_raw;
	int32_t stepper_frequency_hz = g_rollball_debug.stepper_command_hz;
	float ball_position_mm = g_rollball_debug.ball_position_mm;
	float target_position_mm = g_rollball_debug.target_position_mm;
	short stepper_hz;
	short position_mm;
	short target_mm;

	if(stepper_frequency_hz < -9999)
		stepper_hz = -9999;
	else if(stepper_frequency_hz > 9999)
		stepper_hz = 9999;
	else
		stepper_hz = (short)stepper_frequency_hz;

	if(ball_position_mm <= -9999.0f)
		position_mm = -9999;
	else if(ball_position_mm >= 9999.0f)
		position_mm = 9999;
	else if(ball_position_mm >= 0.0f)
		position_mm = (short)(ball_position_mm + 0.5f);
	else
		position_mm = (short)(ball_position_mm - 0.5f);

	if(target_position_mm >= 0.0f)
		target_mm = (short)(target_position_mm + 0.5f);
	else
		target_mm = (short)(target_position_mm - 0.5f);

	//每次先清显存再统一刷新，避免较短的新数字残留旧笔画。
//	oled->Clear();

	oled->ShowString(0,0,"STATE:");
	if(g_rollball_control_enabled != 0U)
		oled->ShowString(48,0,"ON ");
	else
		oled->ShowString(48,0,"OFF");
	oled->ShowString(72,0,"T:");
	oled->ShowShort(88,0,target_mm,3,12);

	oled->ShowString(0,15,"ADC:");
	oled->ShowNumber(40,15,angle_adc_raw,4,12);

	oled->ShowString(0,30,"POS:");
	oled->ShowShort(40,30,position_mm,4,12);
	oled->ShowString(88,30,"mm");

	oled->ShowString(0,45,"SPD:");
	oled->ShowShort(40,45,stepper_hz,4,12);
	oled->ShowString(88,45,"Hz");

	oled->RefreshGram();
}

//static void HardWare_Info_show(void)
//{
//	//车型
//	oled->ShowString(0,0,"CarType:");
//	     if( RobotHardWareParam.CarType == Mec_Car ) oled->ShowString(70,0,"Mec  ");
//	else if( RobotHardWareParam.CarType == Omni_Car ) oled->ShowString(70,0,"Omni ");
//	else if( RobotHardWareParam.CarType == Akm_Car ) oled->ShowString(70,0,"Akm  ");
//	else if( RobotHardWareParam.CarType == Diff_Car ) oled->ShowString(70,0,"Diff ");
//	else if( RobotHardWareParam.CarType == FourWheel_Car ) oled->ShowString(70,0,"4WD  ");
//	else if( RobotHardWareParam.CarType == Tank_Car ) oled->ShowString(70,0,"Tank ");
//	else if( RobotHardWareParam.CarType == Mec_Car_V550 ) oled->ShowString(70,0,"Mec-V");
//	else if( RobotHardWareParam.CarType == FourWheel_Car_V550 ) oled->ShowString(70,0,"4WD-V");
//	else oled->ShowString(70,0,"UKOW ");
	
	//硬件版本
//	oled->ShowString(0,15," HW_Ver:");
//	if( get_HardWareVersion() == HW_1_0 )
//		oled->ShowString(70,15,"1.0");
//	else if( get_HardWareVersion() == HW_1_1 )
//		oled->ShowString(70,15,"1.1");
	
//	oled->RefreshGram();
//}

//static void ChargerDev_Info_show(void)
//{
//	oled->ShowString(07,00,"LA  LB  RB  RA");
//	oled->ShowNumber(0+9,10,ChargeDev.L_A,1,12);
//	oled->ShowNumber(30+9,10,ChargeDev.L_B,1,12);
//	oled->ShowNumber(60+9,10,ChargeDev.R_B,1,12);
//	oled->ShowNumber(90+9,10,ChargeDev.R_A,1,12);
//	
//	oled->ShowString(0,23,"cur:"); 
//	oled->ShowString(75,23,"A"); 
//	oled->ShowFloat(30,23,ChargeDev.ChargingCur/1000.0f,2,2);
//	
//	oled->ShowString(0,36,"st:");
//	oled->ShowNumber(30,36,ChargeDev.ChargingFlag,1,12);
//	
//	oled->ShowString(70,36,"OL:");
//	oled->ShowNumber(96,36,ChargeDev.online,1,12);
//	
//	oled->ShowString(0,50,"rcm:");
//	oled->ShowNumber(35,50,RobotControlParam.ChargeMode,1,12);
//	
//	oled->RefreshGram();
//}

static void RobotMainInfoShow(void)
{
	//首行,所有车型固定显示
	uint16_t Divisor_Mode = 4095/(Number_of_CAR-1);
	uint8_t cartypeshow = USER_ADC_Get_AdcBufValue(userconfigADC_CARMODE_CHANNEL)/Divisor_Mode;
	
	//TODO：调试临时固定
	cartypeshow = Mec_Car;
	
	//else if( RobotControlParam.ChargeMode ) oled->ShowString(0,0,"RCM   ");
	      if( cartypeshow == Mec_Car ) oled->ShowString(0,0,"Mec  ");
	else if( cartypeshow == Omni_Car ) oled->ShowString(0,0,"Omni ");
	else if( cartypeshow == Akm_Car ) oled->ShowString(0,0,"Akm  ");
	else if( cartypeshow == Diff_Car ) oled->ShowString(0,0,"Diff ");
	else if( cartypeshow == FourWheel_Car ) oled->ShowString(0,0,"4WD  ");
	else if( cartypeshow == Tank_Car ) oled->ShowString(0,0,"Tank ");
	else if( cartypeshow == Mec_Car_V550 ) oled->ShowString(0,0,"Mec-V");
	else if( cartypeshow == FourWheel_Car_V550 ) oled->ShowString(0,0,"4WD-V");
	else oled->ShowString(0,0,"UKOW ");
	
//	oled->ShowString(55,0,"Cur:");
//	oled->ShowFloat(87,0,ChargeDev.ChargingCur/1000.0f,1,2);
//	oled->ShowString(120,0,"A");
	
	//第2-4行,不同车型显示不一样的信息
	if( RobotHardWareParam.CarType == Omni_Car )
	{
		oled->ShowString(0,10,"A");
		oled->ShowShort(15,10,RobotControlParam.MotorA.target*1000,5,12);
		oled->ShowShort(60,10,RobotControlParam.MotorA.feedback*1000,5,12);
	
		oled->ShowString(0,20,"B");
		oled->ShowShort(15,20,RobotControlParam.MotorB.target*1000,5,12);
		oled->ShowShort(60,20,RobotControlParam.MotorB.feedback*1000,5,12);
		
		oled->ShowString(0,30,"C");
		oled->ShowShort(15,30,RobotControlParam.MotorC.target*1000,5,12);
		oled->ShowShort(60,30,RobotControlParam.MotorC.feedback*1000,5,12);
		
		oled->ShowString(0,40,"MOVE_Z");
		oled->ShowFloat(75,40,axis_9Val.gyro.z,3,2);
	}
	else if( RobotHardWareParam.CarType == Mec_Car || RobotHardWareParam.CarType==Mec_Car_V550 ||
              RobotHardWareParam.CarType == FourWheel_Car || RobotHardWareParam.CarType == FourWheel_Car_V550)
	{
		oled->ShowString(0,10,"A");
		oled->ShowShort(15,10,RobotControlParam.MotorA.target*1000,5,12);
		oled->ShowShort(60,10,RobotControlParam.MotorA.feedback*1000,5,12);
	
		oled->ShowString(0,20,"B");
		oled->ShowShort(15,20,RobotControlParam.MotorB.target*1000,5,12);
		oled->ShowShort(60,20,RobotControlParam.MotorB.feedback*1000,5,12);
		
		oled->ShowString(0,30,"C");
		oled->ShowShort(15,30,RobotControlParam.MotorC.target*1000,5,12);
		oled->ShowShort(60,30,RobotControlParam.MotorC.feedback*1000,5,12);
		
		oled->ShowString(0,40,"D");
		oled->ShowShort(15,40,RobotControlParam.MotorD.target*1000,5,12);
		oled->ShowShort(60,40,RobotControlParam.MotorD.feedback*1000,5,12);
	}
	
	
	//最后一行,所有车型固定显示
	//控制方式,使能状态,电池电压
	if( RobotControl_CMDsource==NONE_CMD )          oled->ShowString(0,50,"ROS ");
	else if( RobotControl_CMDsource==GamePad_CMD )  oled->ShowString(0,50,"PS2 ");
	else if( RobotControl_CMDsource==ROS_CMD )      oled->ShowString(0,50,"ROS ");
	else if( RobotControl_CMDsource==RCJOY_CMD )    oled->ShowString(0,50,"R-C ");
	else if( RobotControl_CMDsource==CAN_CMD )   oled->ShowString(0,50,"CAN ");
	else if( RobotControl_CMDsource==APP_CMD )      oled->ShowString(0,50,"APP ");
	else if( RobotControl_CMDsource==BootLoader )   oled->ShowString(0,50,"boot");
	else if( RobotControl_CMDsource==Charger_CMD )   oled->ShowString(0,50,"RCM ");
	else oled->ShowString(0,50,"UKOW");
	
	if( RobotControlParam.en_flag ) oled->ShowString(45,50," ON");
	else oled->ShowString(45,50,"OFF");
	
	oled->ShowFloat(70,50,RobotControlParam.Vol,2,2);
	oled->ShowString(112,50,"V");
	oled->ShowString(70,50," ");
	oled->RefreshGram();
	
}

//static void JoyShow(void)
//{
//	oled->ShowString(0,0,"LX:");
//	oled->ShowString(0,10,"LY:");
//	oled->ShowString(0,20,"RX:");
//	oled->ShowString(0,30,"RY:");
//	
//	oled->ShowNumber(30,0,GamePadInterface->LX,3,12);
//	oled->ShowNumber(30,10,GamePadInterface->LY,3,12);
//	oled->ShowNumber(30,20,GamePadInterface->RX,3,12);
//	oled->ShowNumber(30,30,GamePadInterface->RY,3,12);
//	
//	oled->ShowNumber(70,0,rc_remote.LX,4,12);
//	oled->ShowNumber(70,10,rc_remote.LY,4,12);
//	oled->ShowNumber(70,20,rc_remote.RX,4,12);
//	oled->ShowNumber(70,30,rc_remote.RY,4,12);
//	oled->ShowNumber(70,40,rc_remote.freq,4,12);
//	
//	oled->ShowString(0,50,"ENkey:");
//	if( get_EnKeyState()==1 )
//		oled->ShowString(50,50," ON");
//	else
//		oled->ShowString(50,50,"OFF");

//	oled->RefreshGram();
//}

static void ImuShow(void)
{
	extern IMU_DATA_t axis_9Val ;      
	extern IMU_DATA_t axis_9ValOri;  
	extern ATTITUDE_DATA_t AttitudeVal;
	
	oled->ShowFloat(0,0,axis_9Val.gyro.x*57.29578f,3,2);
	oled->ShowFloat(0,10,axis_9Val.gyro.y*57.29578f,3,2);
	oled->ShowFloat(0,20,axis_9Val.gyro.z*57.29578f,3,2);

	oled->ShowFloat(70,0,axis_9Val.accel.x,3,2);
	oled->ShowFloat(70,10,axis_9Val.accel.y,3,2);
	oled->ShowFloat(70,20,axis_9Val.accel.z,3,2);
	
	oled->ShowFloat(0,35,AttitudeVal.pitch,3,2);
	oled->ShowFloat(70,35,AttitudeVal.roll,3,2);
	oled->ShowFloat(0,45,AttitudeVal.yaw,3,2);
	
	oled->RefreshGram();
}

static void F570Show(void)
{
	oled->ShowString(0,0,"EN:");
	oled->ShowNumber(30,0,F570_state.StartFlag,1,12);
	
	oled->ShowString(0,10,"Yaw:");
	oled->ShowFloat(32,10,F570_state.Yaw,3,2);
	
	oled->ShowString(0,20,"Hei:");
	oled->ShowFloat(32,20,F570_state.height,3,2);
	
	
	oled->RefreshGram();
}

void show_task(void* param)
{
	uint8_t taskfreq = 0;
	
	uint8_t page_max = 5;
	
	pRTOSDebugInterface_t debug = &RTOSDebugTimer;
	RTOSDebugPrivateVar debugvar = { 0 };
	
	static uint8_t LastPage = 0;
	static uint32_t LedTickCnt = 0;
	while(1)
	{
		//OLED页数更变时刷新屏幕
		if( page!=LastPage ) oled->Clear();
		if( page >= page_max )
		{
			page = 0;
			continue;
		}
		LastPage = page;
		
		taskfreq = debug->UpdateFreq(&debugvar);
		
		//按键由 RobotControlTask 独占扫描，避免两个任务竞争导致事件丢失。
		if( page==0 )  RollBallInfoShow();
		else if(page==1) ImuShow();
		else if(page==2) F570Show();
		else if(page==3) RobotMainInfoShow();
		
		if( RobotControlParam.LedTickState==2 )
		{	//标零完成时LED灯慢速闪烁
			LedTickCnt++;
			if( LedTickCnt>= taskfreq/2 ) LedTickCnt=0,HAL_GPIO_TogglePin(UserLED_GPIO_Port,UserLED_Pin);
		}
		//未标零时快速闪烁
		else HAL_GPIO_TogglePin(UserLED_GPIO_Port,UserLED_Pin);
		
		vTaskDelay(50);
	}
}

