/* ==========================================================================
 * RobotControl_task.c —— 球杆平衡系统主控制任务
 * ==========================================================================
 *
 * 【这套系统在干什么？】
 *   一根水管（导轨）中间有个支点，可以左右倾斜。管子上放一颗小球。
 *   管子稍微一歪，小球就会往低的那头滚。我们的目标是：
 *   不停地微调管子的倾角，让小球停在指定位置不掉下去。
 *   这就是经典的“球杆系统(Ball and Beam)”，是一个天然不稳定的被控对象
 *   —— 就像用手掌托着一根竖起来的棍子，必须一直动才能保持不倒。
 *
 * 【三个硬件角色】
 *   ① STP23L 激光测距（UART5）→ 测“小球在哪”     → g_readonly_distance (mm)
 *   ② 角位移传感器  (PA1/ADC2) → 测“管子歪多少”   → USER_ADC2_Get_AdcBufValue()
 *   ③ 步进电机 (PC6/PC8/PB14)  → 转动支点改变倾角 → RollBallStepper_SetFrequency()
 *
 * 【串级 PID：为什么要套两层？】
 *
 *     目标位置 ──►[位置环 PID]──► 目标角度 ──►[角度环 PID]──► 步进频率 ──► 电机
 *        ▲            (外环 50Hz)     ▲          (内环 100Hz)                │
 *        │                            │                                     │
 *        └── 小球位置(激光测距) ───────┼─────────────────────────────────────┘
 *                                     └── 实际角度(ADC) ──────────────────────
 *
 *   直接用“小球位置”去算“电机转速”是很难调的，因为中间隔了太多环节。
 *   拆成两层就清楚多了：
 *     · 外环（位置环）：看小球偏了多少 → 决定“管子应该歪到什么角度”
 *       —— 相当于人的大脑：“球在右边太远了，我得把右边抬高一点”
 *     · 内环（角度环）：看当前角度离目标角度差多少 → 决定“电机转多快”
 *       —— 相当于人的手：“把管子转到那个角度去”
 *   内环必须比外环快（这里是 2 倍），否则外环发出的角度指令内环还没执行完，
 *   新指令又来了，两层会互相打架导致震荡。
 *
 * 【控制流程一句话总结】
 *   每 10ms：读传感器 → 位置环算目标角度(每 20ms) → 角度环算脉冲频率
 *          → 发给步进电机 → 更新调试变量 → 睡到下一个 10ms
 *
 * 【调试入口】
 *   · 全局变量 g_rollball_debug 可以直接加进 Keil 的 Watch 窗口实时观察
 *   · 串口1(PA9, 115200) 每 200ms 打印一行状态
 *   · OLED 第 0 页显示位置/目标/使能状态（见 show_task.c）
 *   · 用户按键 PE0：长按启停，单击/双击改目标位置
 * ========================================================================== */

#include "RobotControl_task.h"

//C Lib Include File
#include <math.h>
#include <stdio.h>
#include <string.h>

//STM32 Include File
#include "gpio.h"
#include "tim.h"

//BSP Include File
#include "bsp_oled.h"
#include "bsp_icm20948.h"
#include "bsp_adc.h"
#include "bsp_key.h"
#include "bsp_gamepad.h"
#include "xbox360_gamepad.h"
#include "bsp_buzzer.h"
#include "bsp_can.h"
#include "bsp_RTOSdebug.h"
#include "bsp_rollball_stepper.h"

//APP Include File
#include "robot_select_init.h"
#include "data_task.h"

/*
 * ======================== 球杆系统用户配置区 ========================
 *
 * 调试顺序：
 * 1. ROLLBALL_MODE_SENSOR_MONITOR：步进驱动失能，只看传感器；
 * 2. ROLLBALL_MODE_STEPPER_DIRECTION_TEST：按键点动确认 PC8 方向；
 * 3. ROLLBALL_MODE_ANGLE_PID：只调角度内环；
 * 4. ROLLBALL_MODE_CASCADE_PID：位置外环 + 角度内环。
 */
/* 四种调试模式。修改下面的 ROLLBALL_CONTROL_MODE 来切换，改完要重新编译下载。 */
#define ROLLBALL_MODE_SENSOR_MONITOR        0U  //只看传感器，电机完全不动（最安全）
#define ROLLBALL_MODE_STEPPER_DIRECTION_TEST 1U  //按键点动电机，确认转向对不对
#define ROLLBALL_MODE_ANGLE_PID             2U  //只调角度内环，让管子保持水平
#define ROLLBALL_MODE_CASCADE_PID           3U  //完整串级控制（正式运行模式）

/*
 * 新换电机后先做方向测试。确认“+Hz”令 angle_feedback_adc 增大后，
 * 再把这里改为 ROLLBALL_MODE_CASCADE_PID。
 *
 * 【新手必读】千万不要一上来就跑模式3！正确顺序是 0 → 1 → 2 → 3，
 * 每一步确认没问题再进下一步，否则电机可能带着机构撞到限位。
 */
#define ROLLBALL_CONTROL_MODE ROLLBALL_MODE_CASCADE_PID

//零点：实测后只需要修改这两个宏。
//【零点是什么】传感器读数本身没有“正负”概念，减去零点才有。
//  ROLLBALL_ANGLE_ADC_ZERO：管子处于水平时，ADC2 读到的数值。
//      测法：切到模式0，把管子手动摆平，看串口打印的 adc= 后面那个数，填进来。
//  ROLLBALL_BALL_DISTANCE_ZERO_MM：小球停在导轨正中间时，激光测到的距离(mm)。
//      测法：同样在模式0，把球放中间，读 dist= 后面那个数。
//【这两个数必须实测！】用别人的值几乎一定跑不起来。
#define ROLLBALL_ANGLE_ADC_ZERO            2470U
#define ROLLBALL_BALL_DISTANCE_ZERO_MM     160.0f

/*
 * 符号约定：
 * - 小球位置正方向：从导轨原点指向导轨另一端；
 * - 球杆角度正方向：能让小球向位置正方向加速的倾斜方向；
 * - 步进控制正方向：能让球杆角度反馈变大的旋转方向。
 *
 * 【为什么需要这两个符号宏？】
 * 传感器安装方向是随机的：激光头装在左端还是右端、角位移传感器
 * 顺时针装还是逆时针装，都会让读数增减方向相反。
 * 与其改接线或改一堆代码，不如在这里乘个 +1/-1 一次性掰正。
 *
 * 【怎么判断该填 +1 还是 -1？】三者必须自洽：
 *   1. 手推小球往“位置正方向”走 → ball_position_mm 必须变大 → 定 POSITION_SIGN
 *   2. 手把管子往“能让球朝正方向滚”的方向掰 → angle_feedback_adc 必须变大
 *      → 定 ANGLE_SIGN
 *   3. 步进电机发正频率 → angle_feedback_adc 必须变大 → 定 DIR_LEVEL（见下）
 * 只要有一项符号反了，PID 就会变成“越纠越偏”，小球会加速冲向一端。
 */
#define ROLLBALL_POSITION_SENSOR_SIGN      (+1.0f)
#define ROLLBALL_ANGLE_SENSOR_SIGN         (-1.0f)

/*
 * PC8 哪个电平对应“角度反馈增大”只能由装配方向确定。
 * 方向测试中单击输出 +200 Hz；若 ANG 数值下降，把 SET 改为 RESET。
 *
 * 【模式1 的用法】长按 PE0 启动 → 单击输出 +200Hz 转 250ms，
 * 观察串口打印的 step= 数值（相对基准的 ADC 变化量）：
 *     step 变正 → 当前 SET 是对的，保持不动
 *     step 变负 → 把下面的 GPIO_PIN_SET 改成 GPIO_PIN_RESET
 */
#define ROLLBALL_STEPPER_POSITIVE_DIR_LEVEL GPIO_PIN_SET

//以下四个是电机输出的“安全护栏”，防止 PID 算出离谱的值把机构打坏
#define ROLLBALL_STEPPER_MAX_COMMAND_HZ     1500.0f //最大输出频率(约0.47转/秒)，也是角度环输出限幅
#define ROLLBALL_STEPPER_MIN_RUN_HZ           20.0f //最低运行频率：太低步进电机会“咯噔咯噔”不平滑
#define ROLLBALL_STEPPER_DEADBAND_HZ            5.0f //死区：指令小于这个值直接停转，避免原地高频抖动
#define ROLLBALL_STEPPER_TEST_SPEED_HZ        200.0f //模式1 点动测试的速度
#define ROLLBALL_STEPPER_TEST_DURATION_MS      250U  //模式1 点动测试持续时间(ms)

//控制周期与目标值。
#define ROLLBALL_CONTROL_FREQUENCY_HZ      100U   //控制任务运行频率 100Hz，即每 10ms 一轮
#define ROLLBALL_POSITION_LOOP_DIVIDER     2U     //位置环分频：每 2 轮才算一次 → 位置环 50Hz
#define ROLLBALL_TARGET_POSITION_MM        0.0f   //开机默认目标位置(0 = 导轨正中间)
#define ROLLBALL_TARGET_POSITION_STEP_MM   50.0f  //按一次键，目标位置移动多少 mm
#define ROLLBALL_TARGET_POSITION_MIN_MM    (-300.0f) //目标位置下限，防止把球往管子外面赶
#define ROLLBALL_TARGET_POSITION_MAX_MM    300.0f    //目标位置上限
#define ROLLBALL_ANGLE_TEST_TARGET_ADC     0.0f   //模式2 的目标角度：0 = 保持水平
#define ROLLBALL_ANGLE_TARGET_LIMIT_ADC    300.0f //位置环输出限幅：管子最多允许歪这么多(ADC计数)

/*
 * PID 参数：
 * - 位置环输入单位为 mm，输出单位为 ADC 偏差值；
 * - 角度环输入单位为 ADC 偏差值，输出单位为 STEP 频率 Hz。
 * 舵机换成步进电机后，角度环参数必须重新整定。
 *
 * 【PID 三个参数各管什么】（拿“开车追前车”打比方）
 *   Kp 比例：差得越远踩油门越狠。太小 → 反应慢追不上；太大 → 冲过头来回震荡。
 *   Ki 积分：长期存在的小偏差会被慢慢累积消除。适合消除“稳态误差”
 *            （比如管子略有摩擦，球总是差那么一点点到不了位）。
 *            副作用是容易“积分饱和”，所以下面有 INTEGRAL_LIMIT 限幅。
 *   Kd 微分：看偏差“正在以多快速度变化”，提前刹车。它是抑制震荡的关键，
 *            但对噪声极其敏感 —— 所以配了 D_FILTER_HZ 低通滤波。
 *
 * 【本项目当前整定结果】
 *   位置环 Kp=0.2 Kd=0.2 Ki=0：球杆系统靠 D 项（也就是球速）提供阻尼，
 *       没有 D 项小球一定会来回震荡越来越大。Ki 设 0 是因为积分会让系统更不稳。
 *   角度环 Kp=8.0 其余为 0：内环只需要“快”，纯 P 就够，加 D 反而放大 ADC 噪声。
 *
 * 【调参建议顺序】先在模式2 调角度环 Kp（管子能快速稳到水平且不抖），
 *   再进模式3 调位置环：先只给 Kp（球会震荡），再慢慢加 Kd 直到震荡消失。
 */
#define ROLLBALL_POSITION_KP               0.2f    //位置环比例
#define ROLLBALL_POSITION_KI               0.0f    //位置环积分（球杆系统一般不用）
#define ROLLBALL_POSITION_KD               0.20f   //位置环微分（提供阻尼，最关键）
#define ROLLBALL_POSITION_INTEGRAL_LIMIT   500.0f  //积分限幅，防止积分饱和
#define ROLLBALL_POSITION_D_FILTER_HZ      2.0f    //D 项低通截止频率(Hz)，滤掉测距噪声

#define ROLLBALL_ANGLE_KP                  8.0f    //角度环比例：1个ADC偏差 → 8Hz 脉冲
#define ROLLBALL_ANGLE_KI                  0.0f
#define ROLLBALL_ANGLE_KD                  0.0f
#define ROLLBALL_ANGLE_INTEGRAL_LIMIT      500.0f
#define ROLLBALL_ANGLE_D_FILTER_HZ         0.0f    //填 0 表示不启用滤波


//STP23L 原始距离的合理范围，用于闭环前的基本有效性判断。
//这是第二道防线（第一道在 bsp_stp23L.c 里）。超范围时沿用上一次有效值，
//保证小球被手挡住的瞬间控制器不会收到一个乱跳的数字而发疯。
#define ROLLBALL_DISTANCE_VALID_MIN_MM     1.0f
#define ROLLBALL_DISTANCE_VALID_MAX_MM     300.0f
#define ROLLBALL_PRINT_INTERVAL_MS         200U   //串口打印间隔，太快会拖慢控制任务

//PID 控制器的“记事本”：既存参数，也存计算过程中要跨周期保留的中间量
typedef struct
{
	float kp;                     //比例系数
	float ki;                     //积分系数
	float kd;                     //微分系数
	float integral;               //误差累积值（积分项的“存钱罐”）
	float integral_limit;         //积分限幅，防止存钱罐撑爆
	float last_error;             //上一周期的误差，用来算变化率（微分）
	uint8_t initialized;          //是否已经跑过至少一轮（第一轮没有"上次误差"，不能算微分）
	float filtered_derivative;    //滤波后的微分值
	float derivative_filter_hz;   //微分低通滤波截止频率，0 = 不滤波
} RollBallPID_t;

//可以直接加入 Keil Watch 窗口观察。
//调试时把 g_rollball_debug 拖进 Keil 的 Watch 窗口，
//就能实时看到位置、角度、电机指令等所有关键量，比串口打印方便得多。
volatile RollBallDebug_t g_rollball_debug = { 0 };
//总开关：0=停止，1=闭环运行中。由用户按键 PE0 长按切换，OLED 上显示 ON/OFF。
volatile uint8_t g_rollball_control_enabled = 0U;

static RollBallPID_t g_rollball_position_pid = {
	ROLLBALL_POSITION_KP,
	ROLLBALL_POSITION_KI,
	ROLLBALL_POSITION_KD,
	0.0f,
	ROLLBALL_POSITION_INTEGRAL_LIMIT,
	0.0f,
	0U,
	0.0f,
	ROLLBALL_POSITION_D_FILTER_HZ
};

static RollBallPID_t g_rollball_angle_pid = {
	ROLLBALL_ANGLE_KP,
	ROLLBALL_ANGLE_KI,
	ROLLBALL_ANGLE_KD,
	0.0f,
	ROLLBALL_ANGLE_INTEGRAL_LIMIT,
	0.0f,
	0U,
	0.0f,
	ROLLBALL_ANGLE_D_FILTER_HZ
};

//用于存放小车的控制队列数据
QueueHandle_t  g_xQueueRobotControl = NULL;

//用于记录小车当前的控制来源
volatile uint8_t RobotControl_CMDsource = NONE_CMD; 

//内部函数
static short Read_Encoder(uint8_t EncoderNum);
static float wheelCoefficient(uint32_t diffparam,uint8_t isLeftWheel);
static void Drive_Motor(const RobotControlCMDType_t* target);
static void Smooth_control(RobotControlCMDType_t* cmd,const RobotControlCMDType_t* target,float step);
static void robot_en_check(void);
static void Robot_Forwardkinematics(RobotControlParmentType_t* robot,const RobotParmentType_t* robot_hw);
static void Incremental_PI(PIDController* pid,float Encoder,float Target);
static void Get_Velocity_Form_Encoder(uint16_t ReadFreq);
static void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo);
static void Linear_IMUHelp(RobotControlCMDType_t* target);

//外部函数,预定
float target_limit_float(float insert,float low,float high);


//IMU辅助走直线变量
static RobotControlCMDType_t LinearHelp_Target = { 0 };

//多模智控手柄按键回调函数,均与小车功能相关
void Xbox360GamePad_KeyEvent_Callback(uint8_t keyid,GamePadKeyEventType_t event)
{
	#include "data_task.h"
	
	pBuzzerInterface_t buzzer = &UserBuzzer;
	
	//LB左上按键：减速
	if( keyid == Xbox360KEY_Y && event == GamePadKeyEvent_LONGCLICK )
	{
//		robot_state.FlyStart=1;
		buzzer->AddTask(1,200);
	}
	else if(  keyid == Xbox360KEY_Y && event == GamePadKeyEvent_SINGLECLICK )
	{
//		robot_state.FlyStart=0;
		buzzer->AddTask(1,200);
	}
}


//测距
#include "bsp_stp23L.h"
static OriData_STP23L_t queue_recv;   //从队列取出来的一包原始数据

/**
 * @brief  STP23L 数据处理任务（独立的 FreeRTOS 任务，在 freertos.c 中创建）
 *
 * 它是个“搬运工 + 翻译”：
 *   串口中断把原始数据丢进队列 → 本任务取出来 → 调解析函数 → 更新全局距离
 *
 * 为什么不在中断里直接解析？
 *   解析一帧要处理近 200 字节，比较费时间。中断里待太久会影响
 *   其它中断和控制任务的实时性，所以中断只管“收下并转交”，
 *   耗时的活儿交给任务在中断外慢慢干。这是嵌入式常见的分工方式。
 *
 * portMAX_DELAY 的意思：没数据就一直睡，不占 CPU。数据一来立刻被唤醒。
 */
void STP23L_Task(void *param)
{
	extern QueueHandle_t g_xQueuestp23L_Ori;

	while(1)
	{
		//读取STP23L数据队列
		xQueueReceive(g_xQueuestp23L_Ori,&queue_recv, portMAX_DELAY ); //等待队列数据

		//解析STP23L原始数据,得到高度数据
		//解析成功后会自动更新全局变量 g_readonly_distance
		stp23L_callback(&queue_recv);
	}
}

/**
 * @brief  限幅：把一个数“夹”在上下限之间
 * @note   控制程序里到处都要用它。没有限幅的控制系统是很危险的，
 *         一次异常的传感器数据就可能让电机全速冲出去。
 */
static float RollBall_LimitFloat(float value, float min_value, float max_value)
{
	if(value < min_value) return min_value;
	if(value > max_value) return max_value;
	return value;
}

/**
 * @brief  复位 PID 控制器（把所有历史记录清空，像刚开机一样）
 *
 * 什么时候要复位？
 *   · 每次从“停止”切到“启动”时          —— 否则会用上次停机前的旧积分
 *   · 目标位置被按键改变时                —— 旧的微分/积分对新目标没意义
 *   · 启动后的 1 秒稳定期内每轮都复位     —— 等传感器读数稳下来
 * 不复位的典型后果：一启动电机就猛地窜一下（积分饱和的“开机踢腿”）。
 */
static void RollBall_PID_Reset(RollBallPID_t *pid)
{
	pid->integral = 0.0f;
	pid->last_error = 0.0f;
	pid->initialized = 0U;
	pid->filtered_derivative = 0.0f;
}

/**
 * @brief  PID 计算核心（位置环和角度环共用这一个函数）
 * @param  pid          用哪个控制器（内含参数和历史值）
 * @param  target       目标值（想要达到多少）
 * @param  feedback     反馈值（实际现在是多少）
 * @param  dt           距离上次调用过了多久，单位秒（位置环0.02s，角度环0.01s）
 * @param  output_limit 输出限幅（绝对值不超过这个数）
 * @retval PID 输出
 *
 * 标准公式：输出 = Kp×误差 + Ki×误差累积 + Kd×误差变化率
 */
static float RollBall_PID_Update(RollBallPID_t *pid,
	                              float target,
	                              float feedback,
	                              float dt,
	                              float output_limit)
{
	//第一步：算误差 = 想要的 - 实际的。误差为正说明“还不够，要加”
	float error = target - feedback;
	float derivative = 0.0f;
	float raw_derivative = 0.0f;
	float output;

	//---------- 微分项（D）：误差变化得多快 ----------
	if(pid->initialized != 0U)   //第一轮没有"上次误差"，跳过不算，避免算出巨大的假微分
	{
		//变化率 = (本次误差 - 上次误差) / 时间间隔
		raw_derivative = (error - pid->last_error) / dt;

		if(pid->derivative_filter_hz > 0.0f)
		{
			/* 一阶低通滤波（IIR）。
			 * 微分对噪声极度敏感：传感器抖 1mm，除以 0.02s 就变成 50mm/s 的假速度，
			 * 乘上 Kd 后会让电机剧烈抖动。所以必须先把微分值“抹平”。
			 *
			 * alpha 是滤波系数，取值 0~1：
			 *   接近 0 → 滤得很狠，输出很平滑但反应慢
			 *   接近 1 → 几乎不滤，反应快但噪声大
			 * 这里由截止频率换算：omega_dt = 2π×fc×dt, alpha = ωdt/(1+ωdt)
			 * 本项目 fc=2Hz, dt=0.02s → alpha ≈ 0.2，即每次只采纳 20% 的新值。 */
			float omega_dt =
				2.0f * PI * pid->derivative_filter_hz * dt;
			float alpha = omega_dt / (1.0f + omega_dt);

			//滤波公式：新输出 = 旧输出 + alpha × (新输入 - 旧输出)
			pid->filtered_derivative +=
				alpha * (raw_derivative - pid->filtered_derivative);
			derivative = pid->filtered_derivative;
		}
		else
		{
			//滤波频率填 0 表示不滤波，直接用原始微分
			pid->filtered_derivative = raw_derivative;
			derivative = raw_derivative;
		}
	}
	else
	{
		//第一轮：只做个标记，微分当作 0
		pid->initialized = 1U;
		pid->filtered_derivative = 0.0f;
	}

	//---------- 积分项（I）：误差随时间的累积 ----------
	pid->integral += error * dt;
	//必须限幅！否则长时间达不到目标时积分会越涨越大（叫"积分饱和"），
	//等到终于接近目标了，这个巨大的积分还要很久才能"泄掉"，造成严重超调。
	pid->integral = RollBall_LimitFloat(pid->integral,
	                                   -pid->integral_limit,
	                                   pid->integral_limit);

	//---------- 三项相加 ----------
	output = pid->kp * error
	       + pid->ki * pid->integral
	       + pid->kd * derivative;

	//记住本次误差，供下次算微分
	pid->last_error = error;

	//输出限幅后返回
	return RollBall_LimitFloat(output, -output_limit, output_limit);
}

/**
 * @brief  把 PID 算出的“带符号频率指令”翻译成电机能听懂的“频率 + 方向”
 * @param  command_hz  PID 输出。正负号代表方向，绝对值代表转速
 * @retval 实际执行的指令（带符号，仅用于调试显示）；0 表示已停转
 *
 * 为什么需要这一层转换？
 *   PID 的输出是一个可正可负的数（比如 -350.7），
 *   但步进驱动只认“无符号频率 + 一根方向线”，所以要拆开：
 *      符号 → DIR 引脚电平
 *      绝对值 → STEP 脉冲频率
 *
 * 三道处理（顺序不能乱）：
 *   1. 死区：|指令| ≤ 5Hz 就当作 0，直接停转。
 *      不设死区的话，小球稳定后 PID 输出会在 ±1Hz 之间反复横跳，
 *      电机不停地正一下反一下，发出"吱吱"声还容易磨损。
 *   2. 最小运行频率：真要转就至少 20Hz。步进电机在极低频下
 *      是一步一步蹦的，不平滑，不如干脆停着。
 *   3. 最大限幅：不超过 1500Hz，保护机构。
 */
static int32_t RollBall_SetStepperCommand(float command_hz)
{
	float magnitude = command_hz;   //先取绝对值
	uint32_t frequency_hz;
	GPIO_PinState direction;

	if(magnitude < 0.0f)
	{
		magnitude = -magnitude;
	}

	//① 死区处理：太小的指令直接停转
	if(magnitude <= ROLLBALL_STEPPER_DEADBAND_HZ)
	{
		RollBallStepper_Stop();
		return 0;
	}

	//② + ③ 把频率夹在 [最小运行频率, 最大频率] 之间
	magnitude = RollBall_LimitFloat(
		magnitude,
		ROLLBALL_STEPPER_MIN_RUN_HZ,
		ROLLBALL_STEPPER_MAX_COMMAND_HZ);
	frequency_hz = (uint32_t)(magnitude + 0.5f);   //+0.5 再取整 = 四舍五入

	//④ 根据符号决定 DIR 电平
	if(command_hz >= 0.0f)
	{
		//正指令 → 用配置的"正方向电平"
		direction = ROLLBALL_STEPPER_POSITIVE_DIR_LEVEL;
	}
	else
	{
		//负指令 → 取正方向电平的反面
		direction =
			(ROLLBALL_STEPPER_POSITIVE_DIR_LEVEL == GPIO_PIN_SET) ?
			GPIO_PIN_RESET : GPIO_PIN_SET;
	}

	//⑤ 下发给底层驱动。万一失败（比如驱动器没使能），保险起见停转
	if(RollBallStepper_SetFrequency(frequency_hz, direction) != HAL_OK)
	{
		RollBallStepper_Stop();
		return 0;
	}

	//返回带符号的实际值，方便在串口/Watch 窗口里看清方向
	return (command_hz >= 0.0f) ?
	       (int32_t)frequency_hz : -(int32_t)frequency_hz;
}

/**
 * @brief  球杆平衡系统主控制任务（FreeRTOS 任务，100Hz 固定周期运行）
 *
 * 这是整个系统的“大脑”，在 freertos.c 里被创建。
 * 每一轮（10ms）依次做 5 件事：
 *   1. 读传感器（角度 ADC + 激光测距）并换算成带符号的物理量
 *   2. 扫描按键，处理启停 / 改目标位置
 *   3. 按当前模式跑控制逻辑（串级 PID → 步进频率）
 *   4. 更新调试变量 g_rollball_debug（供 OLED 和 Keil Watch 观察）
 *   5. 每 200ms 串口打印一行状态，然后精确睡到下一个 10ms
 */
void RobotControlTask(void* param)
{
	//---------- 任务运行参数（编译期就算好，不会变）----------
	//控制周期：pdMS_TO_TICKS 把毫秒换算成 FreeRTOS 的 tick 数。
	//1000/100 = 10ms
	const TickType_t control_period =
		pdMS_TO_TICKS(1000U / ROLLBALL_CONTROL_FREQUENCY_HZ);
	//角度环的 dt（秒）：内环每轮都跑 → 1/100 = 0.01s
	const float angle_loop_dt = 1.0f / (float)ROLLBALL_CONTROL_FREQUENCY_HZ;
	//位置环的 dt（秒）：外环 2 轮才跑一次 → 2/100 = 0.02s
	//【注意】dt 必须和实际调用间隔一致，否则微分/积分算出来全是错的
	const float position_loop_dt =
		(float)ROLLBALL_POSITION_LOOP_DIVIDER /
		(float)ROLLBALL_CONTROL_FREQUENCY_HZ;

	//---------- 各种时间戳 ----------
	TickType_t last_wake_time = xTaskGetTickCount();  //供 vTaskDelayUntil 保持精确周期
	TickType_t mode_start_time = last_wake_time;      //本次启动的时刻（用于 1 秒稳定期）
	TickType_t last_print_time = last_wake_time;      //上次串口打印时刻
	TickType_t test_start_time = last_wake_time;      //模式1 点动测试开始时刻

	//---------- 运行时状态变量 ----------
	uint16_t direction_baseline_adc = 0U;   //模式1：点动前记下的 ADC 基准值
	float target_angle_adc = 0.0f;          //角度环的目标（由位置环产生）
	float target_position_mm = ROLLBALL_TARGET_POSITION_MM;  //小球目标位置
	float test_command_hz = 0.0f;           //模式1：本次点动的频率
	//上一次有效的测距值。传感器暂时失效时沿用它，保证控制不断链
	float last_valid_ball_distance_mm = ROLLBALL_BALL_DISTANCE_ZERO_MM;
	uint8_t position_loop_count = 0U;       //位置环分频计数器（0,1,0,1...）
	uint8_t closed_loop_settled = 0U;       //1 秒稳定期是否已过
	uint8_t test_active = 0U;               //模式1：点动是否进行中
	uint8_t key_release_required = 0U;      //要求先松手才能响应下一次按键
	pKeyInterface_t key = &UserKey;         //按键扫描接口（bsp_key.c）

	(void)param;   //本任务不使用参数，这行只是消除编译器"未使用"警告

	//---------- 上电初始化：先保证电机是"死"的，绝对安全 ----------
	RollBallStepper_Init();                 //配置引脚和 TIM8
	(void)RollBallStepper_SetEnabled(0U);   //驱动器失能（可以用手转动机构）
	g_rollball_control_enabled = 0U;        //闭环关闭，等用户长按按键才启动

	while(1)
	{
		/* ============ 第 1 步：读传感器 ============ */

		//读角位移传感器原始值（0~4095）
		uint16_t angle_adc_raw = USER_ADC2_Get_AdcBufValue();
		//读激光测距结果（由 STP23L_Task 在后台更新）
		float ball_distance_sample_mm = g_readonly_distance;
		float ball_distance_raw_mm;
		UserKeyState_t key_state = key_stateless;

		//测距有效性判断：不在合理范围内就认为这次读数不可信
		uint8_t distance_valid =
			(ball_distance_sample_mm >= ROLLBALL_DISTANCE_VALID_MIN_MM)
			&& (ball_distance_sample_mm <= ROLLBALL_DISTANCE_VALID_MAX_MM);

		/* 角度反馈换算：
		 *   (原始ADC - 水平时的ADC零点) × 符号
		 * 结果含义：0 = 管子水平；正数 = 往"能让球朝位置正方向滚"的方向倾斜
		 * 注意先转成 int32_t 再相减 —— 两个 uint16_t 相减若结果为负会变成天文数字！ */
		float angle_feedback_adc =
			(float)((int32_t)angle_adc_raw - (int32_t)ROLLBALL_ANGLE_ADC_ZERO)
			* ROLLBALL_ANGLE_SENSOR_SIGN;

		//有效就更新记录；无效则本轮沿用上次的值（"保持"策略，避免控制器收到跳变）
		if(distance_valid != 0U)
		{
			last_valid_ball_distance_mm = ball_distance_sample_mm;
		}
		ball_distance_raw_mm = last_valid_ball_distance_mm;

		/* 小球位置换算：
		 *   (测得距离 - 中点距离) × 符号
		 * 结果含义：0 = 球在导轨中点；正数 = 球偏向位置正方向那一侧 */
		float ball_position_mm =
			(ball_distance_raw_mm - ROLLBALL_BALL_DISTANCE_ZERO_MM)
			* ROLLBALL_POSITION_SENSOR_SIGN;
		uint32_t mode_elapsed_ms;

		/* ============ 第 2 步：按键处理 ============
		 * 按键接在 PE0，内部上拉，按下时读到低电平(GPIO_PIN_RESET)。
		 * bsp_key.c 的 getKeyState 能区分 单击 / 双击 / 长按(>500ms)。
		 * 【注意】按键只由本任务扫描，show_task 不再扫描，
		 *        否则两个任务抢着读会导致按键事件丢失。
		 *
		 * 按键功能表：
		 *   长按    → 启动 / 停止闭环（OLED 上 ON / OFF 切换）
		 *   单击    → 模式3：目标位置 +50mm ；模式1：正向点动
		 *   双击    → 模式3：目标位置 -50mm ；模式1：反向点动
		 * 模式0（纯监视）下完全不理按键，电机永远不动。 */
		if(ROLLBALL_CONTROL_MODE != ROLLBALL_MODE_SENSOR_MONITOR)
		{
			uint8_t key_pressed;

			//传入控制频率，按键库据此把 tick 数换算成毫秒
			key_state = key->getKeyState(ROLLBALL_CONTROL_FREQUENCY_HZ);
			//直接读引脚，判断"手指现在还按着吗"
			key_pressed =
				(HAL_GPIO_ReadPin(UserKEY_GPIO_Port, UserKEY_Pin) ==
				 GPIO_PIN_RESET);

			if(key_release_required != 0U)
			{
				//一次状态切换后必须先松开按键，防止同一次长按触发反向切换。
				//不加这个保护的话，按住 1 秒可能被识别成两次长按 → 开了又关。
				if(key_pressed == 0U)
				{
					key_release_required = 0U;
				}
			}
			else if(g_rollball_control_enabled == 0U)
			{
				//OFF 状态只有长按可以启动；单击和双击不会误启动。
				if(key_state == long_click)
				{
					//启动流程：使能驱动 → 打开闭环 → 清空所有历史状态
					(void)RollBallStepper_SetEnabled(1U);  //驱动器上电，电机被锁住
					g_rollball_control_enabled = 1U;       //闭环总开关打开
					key_release_required = 1U;             //要求先松手
					mode_start_time = xTaskGetTickCount(); //记下启动时刻，开始 1 秒稳定期
					direction_baseline_adc = angle_adc_raw;//模式1 用的基准
					position_loop_count = 0U;
					closed_loop_settled = 0U;
					//必须清 PID！否则上次停机时残留的积分会让电机开机就窜
					RollBall_PID_Reset(&g_rollball_position_pid);
					RollBall_PID_Reset(&g_rollball_angle_pid);
					RollBallStepper_Stop();                //先不发脉冲，等稳定期过
				}
			}
			else if(key_state == single_click)
			{
				if(ROLLBALL_CONTROL_MODE ==
				   ROLLBALL_MODE_STEPPER_DIRECTION_TEST)
				{
					//模式1：记下当前 ADC 作基准，然后正向点动 200Hz / 250ms。
					//之后看串口的 step= 是变正还是变负，判断 DIR 宏配置对不对。
					direction_baseline_adc = angle_adc_raw;
					test_command_hz = ROLLBALL_STEPPER_TEST_SPEED_HZ;
					test_start_time = xTaskGetTickCount();
					test_active = 1U;
				}
				else
				{
					//模式2/3：目标位置往正方向挪 50mm
					target_position_mm += ROLLBALL_TARGET_POSITION_STEP_MM;
					//限幅，别把目标设到管子外面去
					target_position_mm =
						RollBall_LimitFloat(target_position_mm,
						                    ROLLBALL_TARGET_POSITION_MIN_MM,
						                    ROLLBALL_TARGET_POSITION_MAX_MM);
					//目标变了，位置环的历史误差就没意义了，清掉重来
					RollBall_PID_Reset(&g_rollball_position_pid);
					position_loop_count = 0U;
				}
			}
			else if(key_state == double_click)
			{
				if(ROLLBALL_CONTROL_MODE ==
				   ROLLBALL_MODE_STEPPER_DIRECTION_TEST)
				{
					//模式1：反向点动（频率取负号）
					direction_baseline_adc = angle_adc_raw;
					test_command_hz = -ROLLBALL_STEPPER_TEST_SPEED_HZ;
					test_start_time = xTaskGetTickCount();
					test_active = 1U;
				}
				else
				{
					//模式2/3：目标位置往负方向挪 50mm
					target_position_mm -= ROLLBALL_TARGET_POSITION_STEP_MM;
					target_position_mm =
						RollBall_LimitFloat(target_position_mm,
						                    ROLLBALL_TARGET_POSITION_MIN_MM,
						                    ROLLBALL_TARGET_POSITION_MAX_MM);
					RollBall_PID_Reset(&g_rollball_position_pid);
					position_loop_count = 0U;
				}
			}
			else if(key_state == long_click)
			{
				//ON 状态再次长按：立即停脉冲并释放驱动器。
				//这就是"急停"，出现异常时长按按键即可让机构立刻软下来。
				g_rollball_control_enabled = 0U;
				key_release_required = 1U;
				target_angle_adc = 0.0f;
				test_command_hz = 0.0f;
				test_active = 0U;
				position_loop_count = 0U;
				closed_loop_settled = 0U;
				RollBall_PID_Reset(&g_rollball_position_pid);
				RollBall_PID_Reset(&g_rollball_angle_pid);
				(void)RollBallStepper_SetEnabled(0U);
			}
		}

		//算一下从"启动"到现在过了多少毫秒（用于 1 秒稳定期判断）
		mode_elapsed_ms =
			(uint32_t)((xTaskGetTickCount() - mode_start_time) * portTICK_PERIOD_MS);

		/* ============ 更新调试快照 ============
		 * 把本轮所有关键量存进全局结构体。
		 * OLED 显示任务和 Keil Watch 窗口都从这里取数，
		 * 好处是显示逻辑完全不用碰控制逻辑。 */
		g_rollball_debug.angle_adc_raw = angle_adc_raw;              //角度原始 ADC
		g_rollball_debug.ball_distance_raw_mm = ball_distance_raw_mm;//原始距离 mm
		g_rollball_debug.angle_feedback_adc = angle_feedback_adc;    //去零点后的角度
		g_rollball_debug.ball_position_mm = ball_position_mm;        //去零点后的位置
		g_rollball_debug.target_position_mm = target_position_mm;    //目标位置
		g_rollball_debug.sensor_valid = distance_valid;              //测距是否有效
		g_rollball_debug.stepper_enabled = RollBallStepper_IsEnabled();//驱动是否使能

		/* ============ 第 3 步：按模式执行控制 ============
		 * 下面这一长串 if/else 就是四种模式的分支，每轮只会走进其中一个。 */

		if(ROLLBALL_CONTROL_MODE == ROLLBALL_MODE_SENSOR_MONITOR)
		{
			//【模式0 纯监视】强制关闭一切输出，只看传感器数据。
			//用来标定两个零点宏，是所有调试的第一步。
			g_rollball_control_enabled = 0U;
			(void)RollBallStepper_SetEnabled(0U);
			g_rollball_debug.target_angle_adc = 0.0f;
			g_rollball_debug.stepper_command_hz = 0;
		}
		else if(g_rollball_control_enabled == 0U)
		{
			//【任意模式 + 用户未启动】保持一切归零，等长按按键。
			//这里每轮都复位 PID，保证一按下启动键就是完全干净的初始状态。
			RollBall_PID_Reset(&g_rollball_position_pid);
			RollBall_PID_Reset(&g_rollball_angle_pid);
			target_angle_adc = 0.0f;
			g_rollball_debug.target_angle_adc = 0.0f;
			g_rollball_debug.stepper_command_hz = 0;
			RollBallStepper_Stop();
		}
		else if(ROLLBALL_CONTROL_MODE ==
		        ROLLBALL_MODE_STEPPER_DIRECTION_TEST)
		{
			//【模式1 方向测试】按一下就转一小下，用来确认接线和方向宏。
			//判断条件：点动被触发 且 还没超过 250ms
			if((test_active != 0U) &&
			   ((xTaskGetTickCount() - test_start_time) <
			    pdMS_TO_TICKS(ROLLBALL_STEPPER_TEST_DURATION_MS)))
			{
				g_rollball_debug.stepper_command_hz =
					RollBall_SetStepperCommand(test_command_hz);
			}
			else
			{
				//时间到就停。限时是安全措施：万一机构卡住也只转 250ms。
				test_active = 0U;
				test_command_hz = 0.0f;
				RollBallStepper_Stop();
				g_rollball_debug.stepper_command_hz = 0;
			}

			//这个模式下 target_angle_adc 借用来显示"相对基准的 ADC 变化量"，
			//也就是串口打印里的 step= 。正 = 方向宏正确，负 = 需要改宏。
			g_rollball_debug.target_angle_adc =
				(float)((int32_t)angle_adc_raw -
				        (int32_t)direction_baseline_adc);
		}
		else
		{
			//【模式2 / 模式3】真正的闭环控制在这个分支里
			float stepper_command_hz;

			/*
			 * 启动后先等待 1 秒，给 ADC 和 STP23L 留出稳定时间。
			 * STP23L 当前帧越界或跳变过大时，位置环继续使用上一次有效距离。
			 */
			//--- 启动后的 1 秒稳定期：什么都不做，只等 ---
			if(mode_elapsed_ms < 1000U)
			{
				//ADC 的 80 点平均缓冲区要填满、STP23L 要收到几帧数据，
				//这些都需要时间。此时的读数还不可信，贸然闭环会乱动。
				RollBall_PID_Reset(&g_rollball_position_pid);
				RollBall_PID_Reset(&g_rollball_angle_pid);
				target_angle_adc = 0.0f;
				closed_loop_settled = 0U;
				g_rollball_debug.target_angle_adc = 0.0f;
				g_rollball_debug.stepper_command_hz = 0;
				RollBallStepper_Stop();
			}
			else
			{
				//--- 稳定期刚过的那一轮，再清一次 PID，从干净状态开始闭环 ---
				if(closed_loop_settled == 0U)
				{
					RollBall_PID_Reset(&g_rollball_position_pid);
					RollBall_PID_Reset(&g_rollball_angle_pid);
					closed_loop_settled = 1U;   //只执行一次
				}

				/* ---------- 外环：位置环（决定管子该歪多少）---------- */
				if(ROLLBALL_CONTROL_MODE == ROLLBALL_MODE_ANGLE_PID)
				{
					//【模式2】不跑位置环，目标角度固定为 0（保持水平）。
					//用来单独验证角度内环：手推一下管子，它应该迅速自己回正且不抖。
					target_angle_adc = ROLLBALL_ANGLE_TEST_TARGET_ADC;
				}
				else
				{
					/* 【模式3 串级】位置环 2 轮才跑一次（分频到 50Hz）。
					 * 为什么外环要慢？
					 *   控制系统的经验法则：内环至少比外环快 3~5 倍才不打架。
					 *   而且小球位置本身变化慢，激光测距也有噪声，
					 *   跑太快只会放大噪声，没有好处。 */
					if(position_loop_count == 0U)
					{
						//位置环输出 = 角度环的目标值。这就是"串级"的衔接点。
						//输出被限制在 ±300 ADC 内，即管子最多歪这么多。
						target_angle_adc =
							RollBall_PID_Update(&g_rollball_position_pid,
							                    target_position_mm,   //目标：球该在哪
							                    ball_position_mm,     //反馈：球实际在哪
							                    position_loop_dt,     //0.02s
							                    ROLLBALL_ANGLE_TARGET_LIMIT_ADC);
					}

					//分频计数器：0→1→0→1... 只有等于 0 的那轮才算位置环
					position_loop_count++;
					if(position_loop_count >= ROLLBALL_POSITION_LOOP_DIVIDER)
					{
						position_loop_count = 0U;
					}
				}

				/* ---------- 内环：角度环（决定电机转多快）---------- */
				//每轮都跑，100Hz。输出直接就是步进脉冲频率（带符号）。
				stepper_command_hz =
					RollBall_PID_Update(&g_rollball_angle_pid,
					                    target_angle_adc,      //目标：外环给的角度
					                    angle_feedback_adc,    //反馈：ADC 实测角度
					                    angle_loop_dt,         //0.01s
					                    ROLLBALL_STEPPER_MAX_COMMAND_HZ);

				//下发给电机，同时记录实际执行值供调试查看
				g_rollball_debug.target_angle_adc = target_angle_adc;
				g_rollball_debug.stepper_command_hz =
					RollBall_SetStepperCommand(stepper_command_hz);
			}
		}

		/*
		 * 位置目标不变时：d(error)/dt = -ball_velocity。
		 * 这里输出的是位置环 D 项实际使用的低通滤波后速度。
		 *
		 * 白话解释：误差 = 目标 - 位置。目标不变时，误差变快 1mm/s
		 * 就意味着球在往回走 1mm/s，所以取个负号就是球的实际速度。
		 * 这个值不参与控制，纯粹是给人看的（判断球是在冲还是在稳）。
		 */
		g_rollball_debug.ball_velocity_mm_s =
			-g_rollball_position_pid.filtered_derivative;

		/* ============ 第 5 步：串口调试打印（每 200ms 一行）============
		 * 打印口是串口1（PA9/PA10，115200），见 main.c 的 fputc 重定向。
		 * 【为什么要限速？】printf 是阻塞发送，一行几十字节在 115200 下
		 * 要花好几毫秒。如果每轮 10ms 都打印，控制周期直接被拖垮。 */
		if((xTaskGetTickCount() - last_print_time) >=
		   pdMS_TO_TICKS(ROLLBALL_PRINT_INTERVAL_MS))
		{
			last_print_time = xTaskGetTickCount();

			if(ROLLBALL_CONTROL_MODE == ROLLBALL_MODE_SENSOR_MONITOR)
			{
				/* 模式0 输出示例：
				 *   [RB][MON] adc=2470 adc_d=0 dist=160.0mm pos=0.0mm valid=1
				 * 标定方法：把管子摆水平，看 adc= 的值填进 ANGLE_ADC_ZERO；
				 *          把球放中间，看 dist= 的值填进 BALL_DISTANCE_ZERO_MM。
				 *          填对以后 adc_d 和 pos 都应该接近 0。 */
				printf("[RB][MON] adc=%u adc_d=%d dist=%.1fmm pos=%.1fmm valid=%u\r\n",
				       (unsigned int)angle_adc_raw,
				       (int)((int32_t)angle_adc_raw -
				             (int32_t)ROLLBALL_ANGLE_ADC_ZERO),
				       ball_distance_raw_mm,
				       ball_position_mm,
				       (unsigned int)distance_valid);
			}
			else if(ROLLBALL_CONTROL_MODE ==
			        ROLLBALL_MODE_STEPPER_DIRECTION_TEST)
			{
				/* 模式1 输出示例：
				 *   [RB][STEP-DIR] en=1 cmd=200Hz dir=1 adc=2530 angle=-60.0 step=60
				 * 看最后的 step=：单击（正频率）后它变正 → 方向宏正确；
				 * 变负 → 把 ROLLBALL_STEPPER_POSITIVE_DIR_LEVEL 改成 GPIO_PIN_RESET。 */
				printf("[RB][STEP-DIR] en=%u cmd=%ldHz dir=%u adc=%u angle=%.1f step=%d\r\n",
				       (unsigned int)g_rollball_control_enabled,
				       (long)g_rollball_debug.stepper_command_hz,
				       (unsigned int)RollBallStepper_GetDirection(),
				       (unsigned int)angle_adc_raw,
				       angle_feedback_adc,
				       (int)((int32_t)angle_adc_raw -
				             (int32_t)direction_baseline_adc));
			}
			else
			{
				/* 模式2/3 输出示例：
				 *   [RB][PID] en=1 pos=12.3/0.0mm vel=-5.2mm/s angle=-8.0/-2.5adc step=-44Hz valid=1
				 * 逐项含义：
				 *   en    闭环开关(1=运行中)
				 *   pos   实际位置 / 目标位置 (mm)
				 *   vel   小球速度 (mm/s)，稳定后应在 0 附近小幅波动
				 *   angle 实际角度 / 目标角度 (ADC计数)
				 *   step  正在输出的脉冲频率(带符号，负号表示反向)
				 *   valid 测距是否有效(0 说明球被挡住或超量程) */
				printf("[RB][PID] en=%u pos=%.1f/%.1fmm vel=%.1fmm/s angle=%.1f/%.1fadc step=%ldHz valid=%u\r\n",
				       (unsigned int)g_rollball_control_enabled,
				       ball_position_mm,
				       target_position_mm,
				       g_rollball_debug.ball_velocity_mm_s,
				       angle_feedback_adc,
				       target_angle_adc,
				       (long)g_rollball_debug.stepper_command_hz,
				       (unsigned int)distance_valid);
			}
		}

		/* ============ 睡到下一个控制周期 ============
		 * vTaskDelayUntil 和 vTaskDelay 的区别很关键：
		 *   vTaskDelay(10)      = "从现在起再睡 10ms"
		 *                         → 上面代码执行了 3ms，实际周期就变成 13ms
		 *   vTaskDelayUntil     = "睡到 上次唤醒时刻+10ms 那一刻"
		 *                         → 不管上面跑了多久，周期恒定 10ms
		 * 控制算法里 dt 是写死的常数，周期必须严格恒定，
		 * 否则微分/积分算出来的值全是错的，所以这里必须用 DelayUntil。 */
		vTaskDelayUntil(&last_wake_time, control_period);
	}
}

/* ==========================================================================
 * 【以下都是 WHEELTEC 原小车底盘的代码，球杆系统用不到】
 * --------------------------------------------------------------------------
 * 这些函数保留是为了兼容原来的底盘工程（四轮/麦轮/全向轮驱动、编码器测速、
 * 运动学正反解、IMU 走直线补偿等）。RobotControlTask_NoUse 这个任务
 * 已经不再被创建（见 freertos.c，创建的是上面的 RobotControlTask）。
 *
 * 读代码时可以直接跳过这一整段，不影响理解球杆系统。
 * 唯一还在被别处调用的是最下面的 target_limit_float()。
 * ========================================================================== */

//原小车底盘控制任务（当前未使用，函数名后缀 NoUse 即"不用"的意思）
void RobotControlTask_NoUse(void* param)
{
	vTaskDelay(1000);
	
	//创建控制指令队列
	g_xQueueRobotControl = xQueueCreate(1,sizeof(RobotControlCMDType_t));
	
	//获取时基,用于辅助任务能按固定频率运行
	TickType_t preTime = xTaskGetTickCount();
	
	//本任务的控制频率,单位为Hz
	const uint16_t TaskFreq = 100;

	//调试变量
//	pRTOSDebugInterface_t debug = &RTOSDebugTimer;
//	RTOSDebugPrivateVar priv = { 0 };
	
	//控制活跃计数
	TickType_t ActiveTick = xTaskGetTickCount();
	
	//队列无数据统计
	uint8_t LostCmdCounts = 0;
	
	RobotControlCMDType_t cmd = { 0 };
	
	while(1)
	{
		//从队列中读取控制指令
		BaseType_t control_queue_state = pdFAIL;
		control_queue_state = xQueueReceive(g_xQueueRobotControl,&cmd,0);
		if( control_queue_state == pdPASS )
			LostCmdCounts=0;
		else 
			LostCmdCounts++;
		
		//陀螺仪直线补偿
		if( RobotControlParam.ImuAssistedFlag == 1 )
			Linear_IMUHelp(&cmd);
		else
		{
			LinearHelp_Target.Vx=0;LinearHelp_Target.Vy=0;LinearHelp_Target.Vz=0;
		}
		
		//获取编码器数据并转换为轮速
		Get_Velocity_Form_Encoder(TaskFreq);
		
		//运动学逆解,获取车轮线速度m/s
		Drive_Motor(&cmd);
		
		//小车使能状态检查,根据自检参数判断是否允许启动小车
		robot_en_check();
		
		static uint8_t ErrTipsFlag = 0;
		if( RobotControlParam.en_flag == 1 )
		{	
			//PID计算
			Incremental_PI(&RobotControlParam.PIDController_A,RobotControlParam.MotorA.feedback,RobotControlParam.MotorA.target);
			Incremental_PI(&RobotControlParam.PIDController_B,RobotControlParam.MotorB.feedback,RobotControlParam.MotorB.target);
			Incremental_PI(&RobotControlParam.PIDController_C,RobotControlParam.MotorC.feedback,RobotControlParam.MotorC.target);
			Incremental_PI(&RobotControlParam.PIDController_D,RobotControlParam.MotorD.feedback,RobotControlParam.MotorD.target);
			
			switch( RobotHardWareParam.CarType )
			{
				case Omni_Car:
					Set_Pwm(-RobotControlParam.PIDController_A.Pwm,RobotControlParam.PIDController_B.Pwm,
				            -RobotControlParam.PIDController_C.Pwm,RobotControlParam.PIDController_D.Pwm,0);
					break;
//				case Mec_Car:case Mec_Car_V550:
//					Set_Pwm(-RobotControlParam.PIDController_A.Pwm,RobotControlParam.PIDController_B.Pwm,
//				             RobotControlParam.PIDController_C.Pwm,-RobotControlParam.PIDController_D.Pwm,0);
//					break;
				
				//霍尔版本方向
				case Mec_Car:case Mec_Car_V550:
					Set_Pwm( RobotControlParam.PIDController_A.Pwm,-RobotControlParam.PIDController_B.Pwm,
				            -RobotControlParam.PIDController_C.Pwm, RobotControlParam.PIDController_D.Pwm,0);
					break;
			}
			
			//允许使能后,关闭报错提示
			if( ErrTipsFlag==1 )
			{
				ErrTipsFlag = 0;
				//可加入取消报错后操作
			}
		}
		else //小车被失能,发送0转速停止小车
		{
			//小车被失能时收到控制命令,蜂鸣器提醒报错
			//if( control_queue_state == pdPASS ) buzzer->AddTask(3,100);
			Set_Pwm(0,0,0,0,0);
			
			//被失能后,启动报错提示.
			if( ErrTipsFlag == 0 )
			{
				ErrTipsFlag=1;
				//可加入报错后操作
			}
		}
		
		//运动学正解,根据车轮速度计算三轴速度
		Robot_Forwardkinematics(&RobotControlParam,&RobotHardWareParam);
		
		//判断当前是否有活跃的控制量
		if( cmd.Vx!=0 || cmd.Vy!=0 || cmd.Vz!=0 )
		{
			ActiveTick = xTaskGetTickCount();
			if( RobotControlParam.ParkingMode==1 )
			{
				RobotControlParam.ParkingMode=0;
			}
		}
		
		//根据控制量是否活跃来判断是否进入驻车模式,若小车停止10秒后无控制量,将进入驻车模式省电
		if( RobotControlParam.ParkingMode==0 && xTaskGetTickCount() - ActiveTick > 10000 )
		{
			//根据imu角度判断是否位于斜坡上
			if(fabs(AttitudeVal.pitch) < 10.0f && fabs(AttitudeVal.roll)<10.0f )
			{
				RobotControlParam.ParkingMode=1;
			}
			else
			{	//小车位于斜坡上,刷新用于判断的时间
				ActiveTick = xTaskGetTickCount();
			}
		}
		
		//小车失控保护设置(队列无数据超时情况)
		if( LostCmdCounts > TaskFreq )
		{
			LostCmdCounts=TaskFreq;
			
			switch( RobotControl_CMDsource )
			{
				//非上位机类型的控制默认需要保护
				//此处原本APP控制也应执行,但为了兼容IOS APP(数据非连续发送),保护无法开启.
				case GamePad_CMD:case RCJOY_CMD: case Charger_CMD:
					cmd.Vx=0;cmd.Vy=0;cmd.Vz=0;
					break;
				
				//上位机类型,根据用户设置的安全级别启用保护
				case ROS_CMD:case CAN_CMD:
					if( RobotControlParam.SecurityLevel==0 ) 
						cmd.Vx=0;cmd.Vy=0;cmd.Vz=0;
					break;
			}
		}
		
		/* 延迟指定频率 */
		vTaskDelayUntil(&preTime,pdMS_TO_TICKS( (1.0f/(float)TaskFreq)*1000) );
	}
}

/**
 * @brief  此函数用于写控制队列,所有需要控制小车的任务均需要通过此任务写入控制量.
 * @param  cmd:参数需要包含指令来源,以及指定小车的三轴目标速度大小
           woken:如果是在中断中调用,需要传入此参数。非中断中调用直接写入0
 * @retval 0写入成功,1写入失败.写入失败原因：未指定控制指令来源或当前有更高优先级控制在活跃
 *
 * @details
 * - 用户需要新增控制方式时,需要先在RobotControl_task.h头文件新增一个控制类型的枚举并确定优先级,
 *   然后通过 WriteControlQueue 写控制队列进行小车的控制
 */
uint8_t WriteRobotControlQueue(RobotControlCMDType_t* cmd,BaseType_t* woken)
{
	static uint32_t lastTick = 0;
	uint8_t writeflag = 0;
	
	//队列未创建或未指定的控制源,忽略写入
	if(g_xQueueRobotControl == NULL || cmd->cmdsource == NONE_CMD || 
		cmd->cmdsource >= UnKnownCMD ) return 1;
	
	//高优先级的控制指令,可直接抢占控制
	if( cmd->cmdsource <= RobotControl_CMDsource )
	{
		RobotControl_CMDsource = cmd->cmdsource;
		writeflag = 1;
	}
	else
	{
		//接受到低优先级控制指令,检查活跃时间戳
		uint32_t tick = 0;
		if( __get_IPSR()!=0 ) tick = xTaskGetTickCountFromISR();
		else tick = xTaskGetTickCount();
		
		//高优先级指令超2000ms未活跃,则允许低优先级指令执行
		if( tick - lastTick >= 2000 )
		{
			RobotControl_CMDsource = cmd->cmdsource;
			writeflag = 1;
		}
	}
	
	//允许写入队列
	if( writeflag )
	{
		if( __get_IPSR()!=0 ) //中断上下文判断
		{
			lastTick = xTaskGetTickCountFromISR();
			xQueueOverwriteFromISR(g_xQueueRobotControl,cmd,woken);
		}			
		else 
		{
			lastTick = xTaskGetTickCount();
			xQueueOverwrite(g_xQueueRobotControl,cmd);
		}
		return 0;
	}
	
	return 1;
}

/**
 * @brief  电机PWM输出,控制车轮转速与方向、控制舵机
 * @param  motor_a~d: 4路电机的PWM值 , servo:舵机PWM值
 * @retval 无
 *
 * @details
 * 
 *   
 */
static void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo)
{
	//电机定义编号与定时器映射关系
	#define PWMA1 	  TIM10->CCR1
	#define PWMA2 	  TIM11->CCR1
	#define PWMB1 	  TIM9->CCR1
	#define PWMB2 	  TIM9->CCR2
	#define PWMC1 	  TIM1->CCR2
	#define PWMC2 	  TIM1->CCR1
	#define PWMD1 	  TIM1->CCR4
	#define PWMD2 	  TIM1->CCR3
	
	//舵机与定时器映射关系
	#define Servo_PWM  TIM12->CCR2
	
	//Forward and reverse control of motor
	//电机正反转控制
	if(motor_a<0)			PWMA1=16799,PWMA2=16799+motor_a;
	else 	            PWMA2=16799,PWMA1=16799-motor_a;
	
	//Forward and reverse control of motor
	//电机正反转控制	
	if(motor_b<0)			PWMB1=16799,PWMB2=16799+motor_b;
	else 	            PWMB2=16799,PWMB1=16799-motor_b;

	//Forward and reverse control of motor
	//电机正反转控制	
	if(motor_c<0)			PWMC1=16799,PWMC2=16799+motor_c;
	else 	            PWMC2=16799,PWMC1=16799-motor_c;
	
	//Forward and reverse control of motor
	//电机正反转控制
	if(motor_d<0)			PWMD1=16799,PWMD2=16799+motor_d;
	else 	            PWMD2=16799,PWMD1=16799-motor_d;
	
	//Servo control
	//舵机控制
	Servo_PWM =servo;
}

/**
 * @brief  此函数用于读取车轮编码器数据
 * @param  EncoderNum:编码器编号
 * @retval 当前时刻编码器原始数值
 *
 * @details
 * 
 *   
 */
static short Read_Encoder(uint8_t EncoderNum)
{
	short Encoder_TIM;    
	switch(EncoderNum)
	{
		case 2:  Encoder_TIM = (short)TIM2 -> CNT;   TIM2 -> CNT=0;  break;
		case 3:  Encoder_TIM = (short)TIM3 -> CNT;   TIM3 -> CNT=0;  break;
		case 4:  Encoder_TIM = (short)TIM4 -> CNT;   TIM4 -> CNT=0;  break;	
		case 5:  Encoder_TIM = (short)TIM5 -> CNT;   TIM5 -> CNT=0;  break;	
		default: Encoder_TIM = 0;
	}
	return Encoder_TIM;
}

/**
 * @brief  读取编码器数值并计算车轮速度，单位m/s
           Read the encoder value and calculate the wheel speed, unit m/s
 * @param  无
 * @retval 无
 *
 * @details
 * 
 *   
 */
static void Get_Velocity_Form_Encoder(uint16_t ReadFreq)
{
	short EncoderA = Read_Encoder(2);
	short EncoderB = Read_Encoder(3);
	short EncoderC = Read_Encoder(4);
	short EncoderD = Read_Encoder(5);
	
	//根据车型不同,编码器的数值方向有所改变
	switch( RobotHardWareParam.CarType )
	{
		case Omni_Car:
			EncoderA = -EncoderA; EncoderB = -EncoderB; EncoderC = -EncoderC; EncoderD=0;
			break;
//		case Mec_Car:case Mec_Car_V550:
//		case FourWheel_Car:case FourWheel_Car_V550:
//			EncoderA = -EncoderA; EncoderB = -EncoderB; EncoderC = EncoderC; EncoderD=EncoderD;
//			break;
		
		//霍尔版本
		case Mec_Car:case Mec_Car_V550:
		case FourWheel_Car:case FourWheel_Car_V550:
			EncoderA = EncoderA; EncoderB = EncoderB; EncoderC = -EncoderC; EncoderD=-EncoderD;
	}
	
	// (轮周长/轮子旋转一圈读数)*控制频率*当前编码器读数 = 轮速
	float EncoderK = ReadFreq*RobotHardWareParam.WheelDiameter*PI/(RobotHardWareParam.EncoderAccuracy*RobotHardWareParam.MotorRation*4);
	
	//将编码器原始数值转换为轮速
	RobotControlParam.MotorA.feedback = EncoderA*EncoderK;
	RobotControlParam.MotorB.feedback = EncoderB*EncoderK;
	RobotControlParam.MotorC.feedback = EncoderC*EncoderK;
	RobotControlParam.MotorD.feedback = EncoderD*EncoderK;
}

/**
 * @brief  此函数用于速度平滑控制以及运动学逆解。根据不同的车型执行不同的逆解公式，最终获得车轮速度
 * @param  Vx、Vy、Vz:小车的X轴、Y轴、Z轴三轴速度,单位 m/s
 * @retval 无
 *
 * @details
 * 
 *   
 */
static void Drive_Motor(const RobotControlCMDType_t* target)
{
	//速度平滑值
	static RobotControlCMDType_t smooth = { 0 }; 
	
	//Wheel target speed limit 
	//车轮目标速度限幅 单位m/s
	float amplitude=3.5;
	
	//实际传入的控制为用户控制+陀螺仪补偿
	RobotControlCMDType_t control_val = { 0 };
	control_val.Vx = target->Vx ;
	control_val.Vy = target->Vy ;
	control_val.Vz = target->Vz + LinearHelp_Target.Vz;
	
	//速度平滑
	Smooth_control(&smooth,&control_val,0.02f);
	
	//纠偏系数计算
	float LeftWheelDiff = wheelCoefficient(RobotControlParam.LineDiffParam,1);
	float RightWheelDiff = wheelCoefficient(RobotControlParam.LineDiffParam,0);
	
	switch( RobotHardWareParam.CarType )
	{
		case Omni_Car:
		{
			//运动学解算
			RobotControlParam.MotorA.target = smooth.Vy + RobotHardWareParam.OmniCenterR*smooth.Vz;
			RobotControlParam.MotorB.target = -X_PARAMETER*smooth.Vx - Y_PARAMETER*smooth.Vy + RobotHardWareParam.OmniCenterR*smooth.Vz;
			RobotControlParam.MotorC.target = +X_PARAMETER*smooth.Vx - Y_PARAMETER*smooth.Vy + RobotHardWareParam.OmniCenterR*smooth.Vz;

			//纠偏系数赋值
			RobotControlParam.MotorB.target *= LeftWheelDiff;
			RobotControlParam.MotorC.target *= RightWheelDiff;
			
			//最大轮速限幅
			RobotControlParam.MotorA.target = target_limit_float(RobotControlParam.MotorA.target,-amplitude,amplitude);
			RobotControlParam.MotorB.target = target_limit_float(RobotControlParam.MotorB.target,-amplitude,amplitude);
			RobotControlParam.MotorC.target = target_limit_float(RobotControlParam.MotorC.target,-amplitude,amplitude);
			RobotControlParam.MotorD.target = 0;
			
			break;
		}
		case Mec_Car:case Mec_Car_V550:
		{
			//运动学解算
			RobotControlParam.MotorA.target = +smooth.Vy+smooth.Vx-smooth.Vz*(RobotHardWareParam.AxleSpacing+RobotHardWareParam.WheelSpacing);
			RobotControlParam.MotorB.target = -smooth.Vy+smooth.Vx-smooth.Vz*(RobotHardWareParam.AxleSpacing+RobotHardWareParam.WheelSpacing);
			RobotControlParam.MotorC.target = +smooth.Vy+smooth.Vx+smooth.Vz*(RobotHardWareParam.AxleSpacing+RobotHardWareParam.WheelSpacing);
			RobotControlParam.MotorD.target = -smooth.Vy+smooth.Vx+smooth.Vz*(RobotHardWareParam.AxleSpacing+RobotHardWareParam.WheelSpacing);
			
			//纠偏系数赋值
			RobotControlParam.MotorA.target *= LeftWheelDiff;
			RobotControlParam.MotorB.target *= LeftWheelDiff;
			RobotControlParam.MotorC.target *= RightWheelDiff;
			RobotControlParam.MotorD.target *= RightWheelDiff;
			
			RobotControlParam.MotorA.target = target_limit_float(RobotControlParam.MotorA.target,-amplitude,amplitude);
			RobotControlParam.MotorB.target = target_limit_float(RobotControlParam.MotorB.target,-amplitude,amplitude);
			RobotControlParam.MotorC.target = target_limit_float(RobotControlParam.MotorC.target,-amplitude,amplitude);
			RobotControlParam.MotorD.target = target_limit_float(RobotControlParam.MotorD.target,-amplitude,amplitude);
			break;
		}
	}
}

//速度平滑控制
static void Smooth_control(RobotControlCMDType_t* cmd,const RobotControlCMDType_t* target,float step)
{
	//X轴速度平滑
	if(target->Vx>cmd->Vx)
	{
		cmd->Vx+=step;
		if(cmd->Vx>target->Vx) cmd->Vx=target->Vx;
	}
	else if (target->Vx<cmd->Vx)
	{
		cmd->Vx-=step;
		if(cmd->Vx<target->Vx) cmd->Vx=target->Vx;
	}
	else
		 cmd->Vx=target->Vx;

	//Y轴速度平滑
	if(target->Vy>cmd->Vy)
	{
		cmd->Vy+=step;
		if(cmd->Vy>target->Vy) cmd->Vy=target->Vy;
	}
	else if (target->Vy<cmd->Vy)
	{
		cmd->Vy-=step;
		if(cmd->Vy<target->Vy) cmd->Vy=target->Vy;
	}
	else
		 cmd->Vy=target->Vy;
	
	//Z轴速度平滑
//	if(target->Vz>cmd->Vz)
//	{
//		cmd->Vz+=step*2;
//		if(cmd->Vz>target->Vz) cmd->Vz=target->Vz;
//	}
//	else if (target->Vz<cmd->Vz)
//	{
//		cmd->Vz-=step*2;
//		if(cmd->Vz<target->Vz) cmd->Vz=target->Vz;
//	}
//	else
	
	//Z轴不使用平滑
	cmd->Vz=target->Vz;
}


//小车使能状态检查
static void robot_en_check(void)
{
	//报错码默认为0,根据下列报错情况而设置
	RobotControlParam.ErrNum = 0;
	
	//计算机器人电池电压,电量不足时将禁止小车的控制
	RobotControlParam.Vol = (float)USER_ADC_Get_AdcBufValue(userconfigADC_VOL_CHANNEL)/4095.0f*3.3f*11;
	
	//软件急停开关状态
	if( RobotControlParam.softwareEnflag == 1 ) RobotControlParam.ErrNum++;
	
	//读取机器人急停开关的状态,按键切换时作出对应操作
	static uint8_t last_EnKey = 1;
	RobotControlParam.Enkeystate = HAL_GPIO_ReadPin(ENKey_GPIO_Port,ENKey_Pin);
	
	if( RobotControlParam.Enkeystate==0 ) RobotControlParam.ErrNum++;
	
	//使能开关按下
	if( RobotControlParam.Enkeystate==0 && last_EnKey==1 )
	{
		//可输入动作
	}
	
	//使能开关弹起
	else if( last_EnKey==0 && RobotControlParam.Enkeystate==1 )
	{
		//可输入动作
	}
	
	last_EnKey = RobotControlParam.Enkeystate;
	
	//电压不足.(若开启了自动回充模式,则忽略电压)
	if( RobotControlParam.Vol < 10.0f && RobotControlParam.ChargeMode==0 ) RobotControlParam.ErrNum++;
	
	if( RobotControlParam.ErrNum!=0 ) RobotControlParam.en_flag = 0;
	else RobotControlParam.en_flag = 1;
}


/**
 * @brief  此函数不同车型的运动学正解。根据不同的车型执行不同的正解公式，最终获得小车3轴速度
 * @param  robot:小车控制相关对象  robot_hw:小车硬件参数相关对象
 * @retval 无
 *
 * @details
 * 
 *   
 */
static void Robot_Forwardkinematics(RobotControlParmentType_t* robot,const RobotParmentType_t* robot_hw)
{
	switch( robot_hw->CarType )
	{
		case Omni_Car:
		{
			RobotControlParam.feedbackVx = (RobotControlParam.MotorC.feedback-RobotControlParam.MotorB.feedback)/2.0f/X_PARAMETER;
			RobotControlParam.feedbackVy = (RobotControlParam.MotorA.feedback*2-RobotControlParam.MotorB.feedback-RobotControlParam.MotorC.feedback)/3.0f;
			RobotControlParam.feedbackVz = (RobotControlParam.MotorA.feedback+RobotControlParam.MotorB.feedback+RobotControlParam.MotorC.feedback)/3.0f/RobotHardWareParam.OmniCenterR;
			break;
		}
		case Mec_Car:case Mec_Car_V550:
		{
			RobotControlParam.feedbackVx = (RobotControlParam.MotorA.feedback+RobotControlParam.MotorB.feedback+RobotControlParam.MotorC.feedback+RobotControlParam.MotorD.feedback)/4.0f;
			RobotControlParam.feedbackVy = (RobotControlParam.MotorA.feedback-RobotControlParam.MotorB.feedback+RobotControlParam.MotorC.feedback-RobotControlParam.MotorD.feedback)/4.0f;
			RobotControlParam.feedbackVz = (-RobotControlParam.MotorA.feedback-RobotControlParam.MotorB.feedback+RobotControlParam.MotorC.feedback+RobotControlParam.MotorD.feedback)/4.0f
			                               /(RobotHardWareParam.AxleSpacing+RobotHardWareParam.WheelSpacing);
			break;
		}
	}
}

/**
 * @brief  此函数执行结合IMU数据走直线功能
 * @param  小车目标速度值
 * @retval 无
 *
 * @details
 *  IMU和小车控制方向：逆时针旋转时,yaw为正,yaw速度为正. Vz为正时小车逆时针旋转
 *   
 */
static void Linear_IMUHelp(RobotControlCMDType_t* target)
{
	//用于延迟获取目标角度
	static uint8_t TargetCnt = 0;
	#define DelayCNT 70
	
	//目标角度
	static float target_angle = 0;
	
	//PID参数
	float linear_helpkp = 0.03f;
	float linear_helpkd = 0.005f;
	
	//根据不同车型设置不同的pid参数
	switch( RobotHardWareParam.CarType )
	{
		case Mec_Car: break;
		default:break;
	}
	
	static uint8_t giveup_flag = 0;
	
	//Vx、Vy均补偿
	if( (target->Vx!=0 || target->Vy!=0) && target->Vz==0 && giveup_flag==0 ) 
	{
		//延迟获得目标角度
		TargetCnt++;
		if( TargetCnt==DelayCNT )  target_angle = AttitudeVal.yaw;

		//获取到目标角度后再执行PID控制
		if( TargetCnt >= DelayCNT )
		{
			//锁住计数值防止循环
			TargetCnt=DelayCNT+1;
			
			//误差计算
			float error = target_angle-AttitudeVal.yaw;
			
			//对误差角度归一化
			if( error >  180.0f ) error -= 360.0f;
			if( error < -180.0f ) error += 360.0f;
			
			//当偏差过大时,放弃调整(如悬空控制).
			if( fabs(error)> 30 ) 
			{
				giveup_flag = 1;
				LinearHelp_Target.Vz=0;
				return;
			}
			
			//PID调整小车走直线效果
			LinearHelp_Target.Vz = linear_helpkp * error - linear_helpkd * axis_9Val.gyro.z*57.3f;
			
			//限制调整值
			LinearHelp_Target.Vz = target_limit_float(LinearHelp_Target.Vz,-1.57f,1.57f);
		}

	}
	else if( target->Vx==0 && target->Vy==0 && target->Vz==0 )
	{
		//速度全0时静止一段时间则刷新目标角度
		TargetCnt++;
		if( TargetCnt>100 )  target_angle = AttitudeVal.yaw,TargetCnt=101;
		LinearHelp_Target.Vx = 0;
		LinearHelp_Target.Vy = 0;
		LinearHelp_Target.Vz = 0;
		giveup_flag=0;
	}
	else
	{	//其他情况则延迟目标角度的刷新,防止动态切换的过程目标角度刷新不准确
		TargetCnt=0;
		LinearHelp_Target.Vx = 0;
		LinearHelp_Target.Vy = 0;
		LinearHelp_Target.Vz = 0;
	}
	
}


/**
 * @brief  此函数用于计算纠偏系数,用于纠正小车走直线的效果
 * @param  纠偏系数值,左轮/右轮
 * @retval 纠偏倍数
 *
 * @details
 * 当小车无法走直线时,可通过调整纠偏系数来改善走直线的效果. 
 * 0-100为可调值,50表示无纠偏效果.大于50小车主动往右偏,
 * 小于50小车主动往左偏. 
 *   
 */
static float wheelCoefficient(uint32_t diffparam,uint8_t isLeftWheel)
{
	if( 1 == isLeftWheel ) //左轮纠偏,对应50~100对应1.0~1.2倍的纠偏系数
	{
		if( diffparam>50 )
			return 1.0f + 0.004f*(diffparam-50);
	}
	else //右轮纠偏,50~0对应1.0~1.2倍的纠偏系数
	{
		if( diffparam<50 )
			return 1.0f + 0.004f*(50-diffparam);
	}
	
	return 1.0f;//不满足条件时,默认是1.
}

float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}


//PID控制器
static void Incremental_PI(PIDController* pid,float Encoder,float Target)
{ 	
	pid->Bias = Target - Encoder;
	pid->Pwm += RobotControlParam.kp*(pid->Bias - pid->LastBias) + RobotControlParam.ki*pid->Bias;
	if( pid->Pwm > pid->LimitOutput ) pid->Pwm = pid->LimitOutput;
	if( pid->Pwm < -pid->LimitOutput ) pid->Pwm = -pid->LimitOutput;
	pid->LastBias = pid->Bias;
	
	//驻车模式下清除PWM累计
	if( RobotControlParam.ParkingMode )
	{
		if( pid->Pwm > 0 ) pid->Pwm--;
		if( pid->Pwm < 0 ) pid->Pwm++;
		if( pid->Pwm < 2 && pid->Pwm >-2 )
		{
			pid->Pwm=0;
			pid->ClearDone = 1;
		}
	} 
}

//复位进入bootloader函数
void _System_Reset_FromAPP_RTOS(char uart_recv)
{
	static char res_buf[5];
	static char res_count=0;
	
	res_buf[res_count]=uart_recv;
	
	if( uart_recv=='r'||res_count>0 )
		res_count++;
	else
		res_count = 0;
	
	if(res_count==5)
	{
		res_count = 0;
		//接受到上位机请求的复位字符“reset”，执行软件复位
		if( res_buf[0]=='r'&&res_buf[1]=='e'&&res_buf[2]=='s'&&res_buf[3]=='e'&&res_buf[4]=='t' )
		{
			RobotControlCMDType_t cmd = {
				.cmdsource = BootLoader,
				0,0,0
			};
			
			//写入最高优先级指令停止小车运动
			for(uint8_t i=0;i<10;i++)
			{
				WriteRobotControlQueue(&cmd,0);
				vTaskDelay( pdMS_TO_TICKS(10) );
			}
			
			//复位系统
			NVIC_SystemReset();
		}
	}
}
