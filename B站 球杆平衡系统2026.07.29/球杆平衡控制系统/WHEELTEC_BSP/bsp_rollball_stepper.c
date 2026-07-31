/* ==========================================================================
 * bsp_rollball_stepper.c —— 步进电机脉冲发生器（TIM8 输出比较实现）
 * --------------------------------------------------------------------------
 * 硬件对应关系（详见《接线说明》）：
 *     STEP 脉冲 → PC6（TIM8_CH1 复用功能，硬件自动翻转）
 *     DIR  方向 → PC8（普通 GPIO 输出）
 *     EN   使能 → PB14（普通 GPIO 输出）
 *
 * 本文件的“状态”全部集中在 s_stepper 这一个结构体里，
 * 中断和任务都会碰它，所以成员都加了 volatile，
 * 修改它的地方都用“临界区”保护（临时关中断）。
 * ========================================================================== */

#include "bsp_rollball_stepper.h"

#include "tim.h"

//两次电平翻转之间至少要隔 2 个定时器 tick。
//太小的话中断来不及处理（中断还没执行完下一次比较又到了），会“跑飞”。
//2 tick @100kHz → 单次翻转最快 50kHz → 脉冲最快 25kHz，远高于实际使用频率。
#define ROLLBALL_STEPPER_MIN_TOGGLE_TICKS 2U

//步进电机当前状态（本文件私有）
typedef struct
{
	volatile uint16_t toggle_interval;  //两次翻转间隔多少个定时器 tick
	volatile uint32_t frequency_hz;     //当前设定的脉冲频率（Hz），0=停止
	volatile uint8_t running;           //1=正在输出脉冲
	volatile uint8_t enabled;           //1=驱动器已使能（EN 有效）
	volatile GPIO_PinState direction;   //当前 DIR 电平
} RollBallStepperState_t;

static RollBallStepperState_t s_stepper = {0};

/**
 * @brief  进入临界区：把所有中断关掉
 * @retval 进入之前的中断开关状态（PRIMASK），退出时要用它还原
 *
 * 为什么需要？
 *   下面很多操作要“连着改好几个寄存器”，比如同时改方向和频率。
 *   如果改到一半 TIM8 中断突然插进来，就可能用到“改了一半的状态”，
 *   导致电机抖动甚至丢步。关中断能保证这一小段代码不被打断。
 *
 * 为什么要保存 PRIMASK 而不是无脑开中断？
 *   万一调用者本身就在“已经关中断”的环境里，
 *   我们退出时若强行开中断，就破坏了外层的保护。保存-还原才安全。
 */
static uint32_t RollBallStepper_EnterCritical(void)
{
	uint32_t primask = __get_PRIMASK();   //记下当前中断开关状态
	__disable_irq();                      //关总中断
	return primask;
}

/**
 * @brief  退出临界区：只有进入前中断是开着的，才把中断重新打开
 */
static void RollBallStepper_ExitCritical(uint32_t primask)
{
	if(primask == 0U)   //0 表示进入前中断是允许的
	{
		__enable_irq();
	}
}

/**
 * @brief  求“失能电平”，即使能电平的反面
 *
 * 头文件里只定义了“使能是高还是低”，这里自动推算出“失能是低还是高”，
 * 这样换驱动器时只改一个宏就够了，不会出现两处不一致的低级错误。
 */
static GPIO_PinState RollBallStepper_DisabledLevel(void)
{
	return (ROLLBALL_STEPPER_ENABLE_ACTIVE_LEVEL == GPIO_PIN_SET) ?
	       GPIO_PIN_RESET : GPIO_PIN_SET;
}

/**
 * @brief  修改 TIM8 通道1 的输出模式
 * @param  mode  TIM_OCMODE_TOGGLE(翻转，发脉冲) 或
 *               TIM_OCMODE_FORCED_INACTIVE(强制拉低，不发脉冲)
 *
 * MODIFY_REG(寄存器, 清除哪些位, 写入什么值)：
 *   这是 HAL 提供的宏，等价于 “先把 OC1M 那几位清零，再把 mode 填进去”，
 *   不会影响 CCMR1 寄存器里的其它无关位。
 */
static void RollBallStepper_SetOutputMode(uint32_t mode)
{
	MODIFY_REG(TIM8->CCMR1, TIM_CCMR1_OC1M, mode);
}

/**
 * @brief  停止脉冲输出（“Unsafe”= 不自带临界区保护）
 *
 * 名字里的 Unsafe 是提醒：调用它之前必须已经关好中断，
 * 或者本来就在中断里执行。对外的 RollBallStepper_Stop() 才是安全版本。
 *
 * 停止的四个动作，顺序有讲究：
 *   1. 关比较中断     —— 不再产生新的中断
 *   2. 清中断标志位   —— 把“已经挂起”的那一次也抹掉，避免刚关完又进一次
 *   3. 输出强制无效   —— PC6 拉低并保持，确保不会停在高电平上
 *   4. 停定时器计数   —— 彻底停下
 */
static void RollBallStepper_StopUnsafe(void)
{
	CLEAR_BIT(TIM8->DIER, TIM_DIER_CC1IE);                      //1.关比较中断
	TIM8->SR = ~TIM_SR_CC1IF;                                   //2.清中断标志
	RollBallStepper_SetOutputMode(TIM_OCMODE_FORCED_INACTIVE);  //3.输出强制无效
	CLEAR_BIT(TIM8->CR1, TIM_CR1_CEN);                          //4.停计数器

	//软件状态也一并清掉，保持软硬件一致
	s_stepper.toggle_interval = 0U;
	s_stepper.frequency_hz = 0U;
	s_stepper.running = 0U;
}

/**
 * @brief  步进电机驱动初始化（在 RobotControlTask 开头调用一次）
 *
 * 做三件事：
 *   1. 把 DIR(PC8) 和 EN(PB14) 配成推挽输出；
 *   2. 上电默认：DIR=低，EN=失能 —— 保证上电瞬间电机绝不会乱转；
 *   3. 把 TIM8 摆成“停止 + 不输出”的安全状态，等着 SetFrequency 来启动。
 */
void RollBallStepper_Init(void)
{
	GPIO_InitTypeDef gpio_init = {0};
	uint32_t primask = RollBallStepper_EnterCritical();

	//打开 GPIO 端口时钟（STM32 的外设不开时钟就是“死的”）
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	//配置 DIR 引脚：推挽输出、不上下拉、高速
	gpio_init.Pin = STEPPER_DIR1_Pin;
	gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init.Pull = GPIO_NOPULL;
	gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(STEPPER_DIR1_GPIO_Port, &gpio_init);

	//配置 EN 引脚（沿用上面同样的推挽输出参数）
	gpio_init.Pin = STEPPER_EN_Pin;
	HAL_GPIO_Init(STEPPER_EN_GPIO_Port, &gpio_init);

	//上电安全默认值：方向给低电平，驱动器保持失能（电机可以用手转动）
	HAL_GPIO_WritePin(STEPPER_DIR1_GPIO_Port,
	                  STEPPER_DIR1_Pin,
	                  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STEPPER_EN_GPIO_Port,
	                  STEPPER_EN_Pin,
	                  RollBallStepper_DisabledLevel());

	//把 TIM8 摆到“随时可以启动，但现在什么都不输出”的状态
	CLEAR_BIT(TIM8->CR1, TIM_CR1_CEN);                          //计数器停
	CLEAR_BIT(TIM8->DIER, TIM_DIER_CC1IE);                      //比较中断关
	RollBallStepper_SetOutputMode(TIM_OCMODE_FORCED_INACTIVE);  //输出强制无效(低)
	SET_BIT(TIM8->CCER, TIM_CCER_CC1E);                         //使能通道1输出通路
	SET_BIT(TIM8->BDTR, TIM_BDTR_MOE);                          //高级定时器必须开主输出
	TIM8->SR = 0U;                                              //清掉所有历史标志

	//【易错点】TIM1/TIM8 是“高级定时器”，比普通定时器多一个总开关 MOE。
	//忘记 SET_BIT(BDTR, MOE) 的话，寄存器配置全对但引脚就是不出波形。

	s_stepper.toggle_interval = 0U;
	s_stepper.frequency_hz = 0U;
	s_stepper.running = 0U;
	s_stepper.enabled = 0U;
	s_stepper.direction = GPIO_PIN_RESET;

	RollBallStepper_ExitCritical(primask);
}

/**
 * @brief  使能/失能步进驱动器
 * @param  enabled 1=使能（电机通电、被锁住）  0=失能（断电、可用手转动）
 * @retval HAL_OK / HAL_ERROR(参数非法)
 *
 * 顺序很重要：失能之前必须先停脉冲，否则会出现
 * “驱动器断电了但脉冲还在发”的尴尬状态，恢复使能瞬间电机会猛地一跳。
 */
HAL_StatusTypeDef RollBallStepper_SetEnabled(uint8_t enabled)
{
	uint32_t primask;

	if(enabled > 1U)   //只接受 0 或 1
	{
		return HAL_ERROR;
	}

	if(enabled == 0U)
	{
		RollBallStepper_Stop();   //先停脉冲，再断使能
	}

	primask = RollBallStepper_EnterCritical();
	HAL_GPIO_WritePin(
		STEPPER_EN_GPIO_Port,
		STEPPER_EN_Pin,
		(enabled != 0U) ?
		ROLLBALL_STEPPER_ENABLE_ACTIVE_LEVEL :
		RollBallStepper_DisabledLevel());
	s_stepper.enabled = enabled;
	RollBallStepper_ExitCritical(primask);

	return HAL_OK;
}

uint8_t RollBallStepper_IsEnabled(void)
{
	return s_stepper.enabled;
}

/**
 * @brief  设置脉冲频率和方向 —— 【控制电机的唯一入口】
 * @param  frequency_hz 频率(Hz)，0 表示停止
 * @param  direction    DIR 引脚电平（GPIO_PIN_SET / GPIO_PIN_RESET）
 * @retval HAL_OK 成功；HAL_ERROR 表示未使能或频率超范围
 *
 * 这个函数被 100Hz 的控制任务每 10ms 调用一次，
 * 所以它必须做到：同一个方向上连续改频率时，不打断已有脉冲流。
 */
HAL_StatusTypeDef RollBallStepper_SetFrequency(uint32_t frequency_hz,
	                                           GPIO_PinState direction)
{
	uint32_t denominator;
	uint32_t interval;
	uint32_t primask;
	uint8_t restart_required;   //是否需要“重新起步”（首次启动 或 要换方向）

	//频率为 0 视为“停车”，不算错误
	if(frequency_hz == 0U)
	{
		RollBallStepper_Stop();
		return HAL_OK;
	}

	//安全检查：驱动器没使能就不许发脉冲；频率必须在允许范围内
	if((s_stepper.enabled == 0U) ||
	   (frequency_hz < ROLLBALL_STEPPER_MIN_FREQUENCY_HZ) ||
	   (frequency_hz > ROLLBALL_STEPPER_MAX_FREQUENCY_HZ))
	{
		return HAL_ERROR;
	}

	/* 核心换算：把“脉冲频率”换算成“定时器 tick 间隔”
	 *   一个脉冲 = 两次电平翻转，所以翻转频率 = 2 × 脉冲频率（denominator）
	 *   间隔 tick = 定时器频率 / 翻转频率 = 100000 / (2f)
	 *   式子里加上 denominator/2 是“四舍五入”的常用写法
	 *   （整数除法默认截断，加半个除数就变成就近取整）。
	 * 举例：f=500Hz → denominator=1000 → interval=(100000+500)/1000=100 tick
	 *       即每 1ms 翻转一次，2ms 一个完整脉冲，正好 500Hz。 */
	denominator = frequency_hz * 2U;
	interval =
		(ROLLBALL_STEPPER_TIMER_TICK_HZ + denominator / 2U) /
		denominator;

	//间隔太小中断处理不过来；太大超出 16 位寄存器范围（CCR1 只有 16 位）
	if((interval < ROLLBALL_STEPPER_MIN_TOGGLE_TICKS) ||
	   (interval > UINT16_MAX))
	{
		return HAL_ERROR;
	}

	primask = RollBallStepper_EnterCritical();

	//什么时候需要重新起步？① 之前是停的；② 方向要换。
	//如果只是同方向改速度，就只更新 toggle_interval，
	//中断下次自然按新间隔安排，脉冲流完全不中断，电机不会有顿挫。
	restart_required =
		(s_stepper.running == 0U) ||
		(s_stepper.direction != direction);

	if(restart_required != 0U)
	{
		//换向必须先停：步进电机在有脉冲时改 DIR 会丢步甚至堵转
		RollBallStepper_StopUnsafe();
		HAL_GPIO_WritePin(STEPPER_DIR1_GPIO_Port,
		                  STEPPER_DIR1_Pin,
		                  direction);
		s_stepper.direction = direction;
	}

	//更新软件状态（中断会读 toggle_interval）
	s_stepper.toggle_interval = (uint16_t)interval;
	s_stepper.frequency_hz = frequency_hz;
	s_stepper.running = 1U;

	if(restart_required != 0U)
	{
		//安排第一次比较事件：从“当前计数值”往后推一个间隔
		TIM8->CCR1 = (uint16_t)(TIM8->CNT + interval);
		TIM8->SR = ~TIM_SR_CC1IF;                        //清掉旧标志，防止一开中断就误触发
		RollBallStepper_SetOutputMode(TIM_OCMODE_TOGGLE);//切到翻转模式，开始出波形
		SET_BIT(TIM8->CCER, TIM_CCER_CC1E);              //使能通道输出
		SET_BIT(TIM8->DIER, TIM_DIER_CC1IE);             //开比较中断
		SET_BIT(TIM8->BDTR, TIM_BDTR_MOE);               //开主输出（高级定时器必需）
		SET_BIT(TIM8->CR1, TIM_CR1_CEN);                 //启动计数器
	}

	RollBallStepper_ExitCritical(primask);
	return HAL_OK;
}

/**
 * @brief  安全地停止脉冲输出（对外版本，自带临界区保护）
 * @note   只是停脉冲，EN 使能状态不变，电机仍然被锁住不会自由滑动。
 */
void RollBallStepper_Stop(void)
{
	uint32_t primask = RollBallStepper_EnterCritical();
	RollBallStepper_StopUnsafe();
	RollBallStepper_ExitCritical(primask);
}

//查询当前频率（调试打印和 OLED 显示会用到）
uint32_t RollBallStepper_GetFrequency(void)
{
	return s_stepper.frequency_hz;
}

//查询当前方向电平（方向测试模式下用来确认接线是否正确）
GPIO_PinState RollBallStepper_GetDirection(void)
{
	return s_stepper.direction;
}

/**
 * @brief  TIM8 比较中断处理 —— 脉冲能不停地发出来，全靠这个函数
 *
 * 被 stm32f4xx_it.c 的 TIM8_CC_IRQHandler() 调用。
 * 每次比较事件发生时：硬件已经自动翻转了 PC6 电平，
 * 软件在这里做的唯一一件事是：把 CCR1 往后挪一个间隔，安排下一次翻转。
 *
 * 【为什么是 CCR1 += interval 而不是 CNT + interval？】
 *   用“上一次的比较值”做基准累加，误差不会累积，频率长期精确。
 *   如果用当前 CNT，中断响应的抖动会一次次叠加进去，频率就漂了。
 *
 * 【溢出怎么办？】
 *   CCR1 是 16 位，加着加着会超过 65535 自动回绕到 0，
 *   而 TIM8 的 ARR 也设成 65535，计数器同样在 0~65535 循环，
 *   两者回绕节奏一致，所以溢出是“自洽”的，不需要额外处理。
 */
void RollBallStepper_TIM8_CC_IRQHandler(void)
{
	uint32_t status = TIM8->SR;             //读中断标志
	uint32_t interrupt_enable = TIM8->DIER; //读中断使能

	//必须同时满足“标志置位”且“中断已使能”才是我们要处理的事件。
	//只判断标志位的话，可能处理到根本没开启的中断源。
	if(((status & TIM_SR_CC1IF) != 0U) &&
	   ((interrupt_enable & TIM_DIER_CC1IE) != 0U))
	{
		TIM8->SR = ~TIM_SR_CC1IF;   //清标志（必须清，否则会反复进中断）

		if((s_stepper.running != 0U) && (s_stepper.enabled != 0U))
		{
			//正常情况：安排下一次翻转
			TIM8->CCR1 = (uint16_t)
				(TIM8->CCR1 + s_stepper.toggle_interval);
		}
		else
		{
			//兜底保护：万一状态被改成“停止/失能”但中断还在跑，
			//在这里直接把定时器彻底关掉，避免电机失控继续转。
			RollBallStepper_StopUnsafe();
		}
	}
}
