/**
 * @file    calibration.h
 * @brief   视觉-物理坐标三点标定模块
 * @note    长按 KEY_L 进入标定模式, 支持三步采样、Flash 存储
 *
 * 三步标定:
 *   N0 = 0cm   (中心) 视觉坐标
 *   N1 = +11.5cm        视觉坐标
 *   N2 = -11.5cm        视觉坐标
 *
 * 映射公式 (后续使用):
 *   pixel_to_cm(p) = A + B * p
 *   其中: slope  = 23.0 / (N1 - N2)
 *         offset = -slope * N0
 *         cm(x)  = offset + slope * x
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include "stm32f4xx_hal.h"

/* ========================== 闪存地址 ================================ */
/** @brief 标定数据存储地址 (Sector 7 末尾) */
#define CAL_FLASH_ADDR 0x0807F000U
#define CAL_FLASH_SECTOR FLASH_SECTOR_7
#define CAL_MAGIC 0xCA11B8A0U

/* ========================== 采样参数 ================================ */
/** @brief 每个标定点采样次数 */
#define CAL_SAMPLE_COUNT 15

/** @brief K230 视觉数据无效标记 */
#define CAL_VISION_INVALID 65535U

/* ========================== 数据类型 ================================ */

/** @brief 标定数据结构 (存入 Flash) */
typedef struct
{
    float n0;       /**< 0cm 处视觉 X 坐标 */
    float n1;       /**< +11.5cm 处视觉 X 坐标 */
    float n2;       /**< -11.5cm 处视觉 X 坐标 */
    uint32_t magic; /**< 魔数 0xCA11BRA0, 标记数据有效 */
} CalibrationData;

/** @brief 标定步骤 */
typedef enum
{
    CAL_STEP_IDLE = 0,
    CAL_STEP_CENTER, /**< 步骤1: 0cm 球放好, 等待短按 */
    CAL_STEP_PLUS,   /**< 步骤2: +11.5cm 球放好, 等待短按 */
    CAL_STEP_MINUS,  /**< 步骤3: -11.5cm 球放好, 等待短按 */
    CAL_STEP_RESULT, /**< 展示三个采样结果 */
} CalStep;

/* ========================== 全局实例 ================================ */
extern CalibrationData g_cal_data;

/* ========================== API ===================================== */

/** @brief 标定模块初始化 (开机加载 Flash 数据) */
void Calibration_Init(void);

/** @brief 进入标定模式 (电机已在零位, 暂停平衡) */
void Calibration_Enter(void);

/** @brief 标定步骤处理 (在主循环中调用)
 *  @param key_event   中间键事件 (KEY_EVENT_SHORT/KEY_EVENT_LONG/NONE)
 *  @param key_l_event 左键事件   (KEY_EVENT_SHORT/KEY_EVENT_LONG/NONE)
 *  @retval  0=仍在标定流程中  1=保存退出  2=不保存退出
 */
uint8_t Calibration_Process(uint8_t key_event, uint8_t key_l_event);

/** @brief 获取标定进度 (用于 OLED 显示) */
CalStep Calibration_GetStep(void);

/** @brief 获取采样进度 (0~CAL_SAMPLE_COUNT) */
uint8_t Calibration_GetSampleProgress(void);

/** @brief 获取指定步骤的最终均值 (步骤完成后有效) */
float Calibration_GetResult(uint8_t step_index);

/** @brief 数据是否有效 */
uint8_t Calibration_IsValid(void);

/** @brief 将像素坐标转为物理 cm */
float Calibration_PixelToCm(uint16_t pixel_x);

#endif /* CALIBRATION_H_ */
