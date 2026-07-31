/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//C Include File
#include <stdio.h>

//STM32 Include File
#include "usart.h"

//FreeRTOS Include File
#include "queue.h"

/////////// 测试 /////////////////

#include "bsp_oled.h"
#include "bsp_led.h"
#include "bsp_dwt.h"
#include "bsp_buzzer.h"
#include "bsp_adc.h"
#include "bsp_can.h"
#include "bsp_stp23L.h"
/////////// 测试 END /////////////

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//FreeRTOS系统调试
#define userconfig_OPEN_STACK_CHECK     0  //开启任务栈大小检查打印
#define userconfig_OPEN_CPU_USAGE_CHECK 0  //开启检查CPU占比打印
#define userconfig_OPEN_CHECK_HEAPSIZE  0  //检查剩余的堆大小

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

uint8_t BlueToothBuffer = 0;//蓝牙串口接收缓冲
uint8_t rosbuffer = 0;      //ROS串口接收缓冲区
uint8_t usart1_buffer = 0;  //串口1接收缓冲区

//蓝牙数据队列
QueueHandle_t g_xQueueBlueTooth = NULL;

//ROS数据队列
QueueHandle_t  g_xQueueROSserial = NULL;

//定义STP23L模块的队列
QueueHandle_t g_xQueuestp23L_Ori=NULL;//存放STP23L原始数据

/* USER CODE END Variables */
/* Definitions for InitTask */
osThreadId_t InitTaskHandle;
const osThreadAttr_t InitTask_attributes = {
  .name = "InitTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

//FreeRTOS系统调试任务
#if ( 1 == userconfig_OPEN_CPU_USAGE_CHECK ) || ( 1 == userconfig_OPEN_STACK_CHECK ) || ( 1 == userconfig_OPEN_CHECK_HEAPSIZE )
void CpuUsageCheckTask(void *param);
#endif

//任务声明
void RobotControlTask(void* param);
void ImuTask(void* param);
void show_task(void* param);
void BlueToothControlTask(void* param);
void APPshow_task(void* param);
void RobotDataTransmitTask(void* param);
void DataCheckTask(void* param);
void STP23L_Task(void *param);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_HOST_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{
	//使用cpu占比计时器前初始化内容
	TIM6->CNT = 0;
}

__weak unsigned long getRunTimeCounterValue(void)
{
	static unsigned long time = 0 ;
	static uint16_t lasttime = 0;
	static uint16_t nowtime = 0;
	
	nowtime = TIM6->CNT; //获得当前计数值
	
	//如果本次计数值小于上次计数值，说明发生了定时器计数溢出
	if( nowtime < lasttime )
	{
		time += (nowtime + 0xffff - lasttime); //溢出后的时间增量
	}		
	else time += ( nowtime - lasttime ) ; //未发生溢出，增量为本次时间-上次时间
	
	lasttime = nowtime;
	
	return time;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
	
	//空闲任务钩子函数
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
	
	//任务栈溢出警告
	printf("%s stack overflow\r\n",pcTaskName);
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
	
	//内存分配失败警告
	printf("malloc failed.check heapsize\r\n");
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	g_xQueueBlueTooth = xQueueCreate(50,sizeof(char));//蓝牙数据队列
	g_xQueueROSserial = xQueueCreate(50,sizeof(char));//ROS数据队列
	
	//创建STP23L激光测距模块队列
	g_xQueuestp23L_Ori = xQueueCreate(3,sizeof(OriData_STP23L_t));
	
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of InitTask */
  InitTaskHandle = osThreadNew(StartDefaultTask, NULL, &InitTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  
  //小车控制任务
  xTaskCreate(RobotControlTask,"ControlTask",128*4,NULL,osPriorityAboveNormal1,NULL);
  
  //IMU数据读取任务
  xTaskCreate(ImuTask,"ImuTask",128*4,NULL,osPriorityNormal,NULL);
  
  //数据发送任务
//  xTaskCreate(RobotDataTransmitTask,"data_task",128*4,NULL,osPriorityNormal,NULL);
  
  //蓝牙数据解析任务
  xTaskCreate(BlueToothControlTask,"BTControlTask",128*2,NULL,osPriorityNormal,NULL);
  
  //OLED显示任务
  xTaskCreate(show_task,"showTask",128*4,NULL,osPriorityBelowNormal7,NULL);

	//创建STP23L数据处理任务
	xTaskCreate(STP23L_Task,"STP23L_Task",128*2,NULL,osPriorityNormal,NULL);
	
  //APP显示任务
//  xTaskCreate(APPshow_task,"AppShow",128*2,NULL,osPriorityBelowNormal7,NULL);
  
  //全局数据检查任务
  xTaskCreate(DataCheckTask,"DataCheck",128,NULL,osPriorityNormal,NULL);
  
	//FreeRTOS调试任务
	#if ( 1 == userconfig_OPEN_CPU_USAGE_CHECK ) || ( 1 == userconfig_OPEN_STACK_CHECK ) || ( 1 == userconfig_OPEN_CHECK_HEAPSIZE )
	 static uint16_t delaytime = 5000;//打印时间间隔，单位tick
	 xTaskCreate(CpuUsageCheckTask,"DebugTask",128, &delaytime ,osPriorityAboveNormal,NULL);
	#endif
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the InitTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	
	//开机提示
	pBuzzerInterface_t tips = &UserBuzzer;
	tips->AddTask(1,800);
	
	//启用串口中断
	HAL_UART_Receive_IT(&huart2,&BlueToothBuffer,1);
	HAL_UART_Receive_IT(&huart3,&rosbuffer,1);

//	HAL_UARTEx_ReceiveToIdle_DMA(&huart5,mr20lidar_buffer,userconfig_MR20LidarDMA_LEN);
//	HAL_UARTEx_ReceiveToIdle_DMA(&huart6,mr20lidar_buffer_id2,userconfig_MR20LidarDMA_LEN);

	//启动STP23L激光测距模块的DMA搬运
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5,DMABuf_oridata_stp23L.Buf,userconfig_STP23L_DMABUF_LEN);
	
	//CAN初始化
	pCANInterface_t candev = &UserCAN1Dev;
	candev->init();
	
//	printf("system start!\r\n");
	
	vTaskDelete(NULL);
  for(;;)
  {
		
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

#if ( 1 == userconfig_OPEN_CPU_USAGE_CHECK ) || ( 1 == userconfig_OPEN_STACK_CHECK ) || ( 1 == userconfig_OPEN_CHECK_HEAPSIZE )
void CpuUsageCheckTask(void *param)
{
	uint16_t* delaytime = (uint16_t*)param;
	static char showbuf[500];
	
	while( 1 )
	{
#if 1 == userconfig_OPEN_CPU_USAGE_CHECK
		//打印CPU占比
		vTaskGetRunTimeStats(showbuf);
		printf("TaskName\tUseTime\tCPU\r\n");
		printf("%s\r\n",showbuf);
		vTaskDelay(*delaytime);
#endif

#if 1 == userconfig_OPEN_STACK_CHECK
		//打印剩余任务栈大小,单位word
		vTaskList(showbuf);
		printf("TaskName\tTaskState\tTaskPrio\tStackSize\tTaskNum\r\n");
		printf("%s\r\n",showbuf);
		vTaskDelay(*delaytime);
#endif
		
#if 1 == userconfig_OPEN_CHECK_HEAPSIZE
		//打印剩余的堆区大小,单位bytes
		printf("free heap size : %d bytes\r\n\r\n",xPortGetFreeHeapSize());
		vTaskDelay(*delaytime);
#endif
	}
}
#endif

/* USER CODE END Application */

