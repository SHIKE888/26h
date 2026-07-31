/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file            : usb_host.c
  * @version         : v1.0_Cube
  * @brief           : This file implements the USB Host
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_hid.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Host core handle declaration */
USBH_HandleTypeDef hUsbHostFS;
ApplicationTypeDef Appli_state = APPLICATION_IDLE;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*
 * user callback declaration
 */
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */
//C Lib Include File
#include "math.h"

//BSP Include File
#include "bsp_gamepad.h"

//APP Include File
#include "RobotControl_task.h"
#include "robot_select_init.h"
#include "Xbox360_gamepad.h"
#include "data_task.h"

//USB解码函数
extern USBH_StatusTypeDef USBH_HID_PS2_Decode(USBH_HandleTypeDef *phost);


void USBH_HID_EventCallback(USBH_HandleTypeDef *phost)
{
	//USB数据解码
	USBH_HID_PS2_Decode(phost);
	
	uint8_t writeflag = 1;
	
	RobotControlCMDType_t cmd = {
		.cmdsource = GamePad_CMD,
		0,0,0
	};
	
	//start按键
	static uint8_t startkey = 0;
	
	//空闲检测
	static uint8_t idleCount=0;
	
	//摇杆值
	float LX=127,LY=127,RX=127;
	float ThrottleTri = 255;
	float base_vz = PI/4.0f ;//Z轴基础速度

	LY = GamePadInterface->LY - 127;
	LX = 127 - GamePadInterface->LX;
	RX = 127 - GamePadInterface->RX;
	
	//忽略小幅度摇杆值
	if( fabs(LY)<30 ) LY = 0;
	if( fabs(LX)<30 ) LX = 0;
	if( fabs(RX)<30 ) RX = 0;
	
	if( gamepad_brand == Xbox360 ) 
	{
		if( GamePadKeyState_Press == GamePadInterface->getKeyState(Xbox360KEY_Menu) ) 
			startkey = 1;
		
		if( (int)LY == 0 )
		{
			if( GamePadInterface->LT == 0 && GamePadInterface->RT != 0 )
				ThrottleTri =  GamePadInterface->RT, LY = 127;
			else if( GamePadInterface->LT != 0 && GamePadInterface->RT == 0 )
				ThrottleTri =  -GamePadInterface->LT,LY = 127;
			else
				ThrottleTri = 0;
		}
	}
	
	//将摇杆值转换为3轴目标速度
	cmd.Vx = (LY/127.0f) * RobotControlParam.defalutSpeed * (ThrottleTri/255.0f);
	cmd.Vy = (LX/127.0f) * RobotControlParam.defalutSpeed;
	cmd.Vz = base_vz * (RX/127.0f) * ( RobotControlParam.defalutSpeed/500.0f );
	
	//非全向车型,取消Y轴速度
	switch(RobotHardWareParam.CarType)
	{
		case Mec_Car:case Mec_Car_V550:case Omni_Car:
			break;
		default:
			cmd.Vy=0;
			break;
	}
	
    //Unit conversion, mm/s -> m/s
    cmd.Vx /= 1000.0f;
	cmd.Vy /= 1000.0f;
	if(cmd.Vx<0)cmd.Vz=-cmd.Vz;
	
	if( cmd.Vx==0&&cmd.Vy==0&&cmd.Vz==0 ) idleCount++;
	else idleCount=0;
	if( idleCount>10 ) writeflag=0,idleCount=10;
	
	if( startkey && writeflag )
	{
		WriteRobotControlQueue(&cmd,0);
		
		/* 震动反馈添加处 */
		
		/* END */
	}
	
	//飞行器控制
//	if( GamePadKeyState_Press == GamePadInterface->getKeyState(Xbox360KEY_UP) )
//	{
//		robot_state.FlyRoll = 10.0f/57.3f;
//	}
//	else if( GamePadKeyState_Press == GamePadInterface->getKeyState(Xbox360KEY_DOWN) )
//	{
//		robot_state.FlyRoll = -10.0f/57.3f;
//	}
//	else robot_state.FlyRoll=0;
//	
//	if( GamePadKeyState_Press == GamePadInterface->getKeyState(Xbox360KEY_LEFT) )
//	{
//		robot_state.FlyPitch = 10.0f/57.3f;
//	}
//	else if( GamePadKeyState_Press == GamePadInterface->getKeyState(Xbox360KEY_RIGHT) )
//	{
//		robot_state.FlyPitch = -10.0f/57.3f;
//	}
//	else robot_state.FlyPitch=0;
//	
//	if( GamePadKeyState_Press == GamePadInterface->getKeyState(Xbox360KEY_LB) )
//	{
//		robot_state.FlyHeight = 0.012f;
//	}
//	else if( GamePadKeyState_Press == GamePadInterface->getKeyState(Xbox360KEY_RB) )
//	{
//		robot_state.FlyHeight = -0.012f;
//	}
//	else robot_state.FlyHeight=0;
}

/* USER CODE END 1 */

/**
  * Init USB host library, add supported class and start the library
  * @retval None
  */
void MX_USB_HOST_Init(void)
{
  /* USER CODE BEGIN USB_HOST_Init_PreTreatment */
	
	//TODO:新增版本V1.0无手柄返回
	
	GamePadInterface = &GamePadDefalut;
	extern USBH_ClassTypeDef  GamePad_HID_Class;
	extern USBH_ClassTypeDef  GamePad_NonStdHID_Class;
	if (USBH_Init(&hUsbHostFS, USBH_UserProcess, HOST_FS) != USBH_OK)
	{
		Error_Handler();
	}
	if (USBH_RegisterClass(&hUsbHostFS, &GamePad_HID_Class) != USBH_OK)
	{
		Error_Handler();
	}
	if (USBH_RegisterClass(&hUsbHostFS, &GamePad_NonStdHID_Class) != USBH_OK)
	{
		Error_Handler();
	}

	if (USBH_Start(&hUsbHostFS) != USBH_OK)
	{
		Error_Handler();
	}
  #pragma diag_suppress 111
  return;
  /* USER CODE END USB_HOST_Init_PreTreatment */

  /* Init host Library, add supported class and start the library. */
  if (USBH_Init(&hUsbHostFS, USBH_UserProcess, HOST_FS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_RegisterClass(&hUsbHostFS, USBH_HID_CLASS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_Start(&hUsbHostFS) != USBH_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_HOST_Init_PostTreatment */

  /* USER CODE END USB_HOST_Init_PostTreatment */
}

/*
 * user callback definition
 */
static void USBH_UserProcess  (USBH_HandleTypeDef *phost, uint8_t id)
{
  /* USER CODE BEGIN CALL_BACK_1 */
  switch(id)
  {
  case HOST_USER_SELECT_CONFIGURATION:
  break;

  case HOST_USER_DISCONNECTION:
  Appli_state = APPLICATION_DISCONNECT;
  break;

  case HOST_USER_CLASS_ACTIVE:
  Appli_state = APPLICATION_READY;
  break;

  case HOST_USER_CONNECTION:
  Appli_state = APPLICATION_START;
  break;

  default:
  break;
  }
  /* USER CODE END CALL_BACK_1 */
}

/**
  * @}
  */

/**
  * @}
  */

