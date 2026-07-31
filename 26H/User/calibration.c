/**
 * @file    calibration.c
 * @brief   Visual-physical coordinate 3-point calibration module
 * @note    Uses STM32F4 internal Flash for storage (Sector 7)
 */

#include "calibration.h"
#include "k230_uart.h"
#include "key.h"
#include <string.h>

/* ========================== Global =================================== */
CalibrationData g_cal_data;

/* ========================== Internal State =========================== */
static CalStep g_cal_step = CAL_STEP_IDLE;
static uint8_t g_cal_active = 0;
static uint8_t g_sample_cnt = 0;
static uint32_t g_sample_sum = 0;
static float g_results[3] = {0};
static uint32_t g_last_sample_tick = 0;

/* ========================== Flash Ops ================================ */

static void Calibration_FlashLoad(void)
{
    CalibrationData *pFlash = (CalibrationData *)CAL_FLASH_ADDR;
    if (pFlash->magic == 0xCA11B8A0)
    {
        g_cal_data = *pFlash;
    }
    else
    {
        memset(&g_cal_data, 0, sizeof(CalibrationData));
    }
}

static uint8_t Calibration_FlashSave(void)
{
    HAL_StatusTypeDef status;
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase_cfg;
    erase_cfg.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_cfg.Sector = FLASH_SECTOR_7;
    erase_cfg.NbSectors = 1;
    erase_cfg.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    uint32_t sector_error = 0;
    status = HAL_FLASHEx_Erase(&erase_cfg, &sector_error);
    if (status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return 0;
    }
    g_cal_data.magic = 0xCA11B8A0;
    uint32_t *src = (uint32_t *)&g_cal_data;
    uint32_t addr = CAL_FLASH_ADDR;
    for (uint32_t i = 0; i < (sizeof(CalibrationData) + 3) / 4; i++)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, (uint64_t)src[i]);
        if (status != HAL_OK)
        {
            HAL_FLASH_Lock();
            return 0;
        }
        addr += 4;
    }
    HAL_FLASH_Lock();
    return 1;
}

/* ========================== Init ===================================== */

void Calibration_Init(void)
{
    Calibration_FlashLoad();
    g_cal_step = CAL_STEP_IDLE;
    g_cal_active = 0;
    g_sample_cnt = 0;
    g_sample_sum = 0;
    memset(g_results, 0, sizeof(g_results));
}

/* ========================== Flow ===================================== */

void Calibration_Enter(void)
{
    g_cal_active = 1;
    g_cal_step = CAL_STEP_CENTER;
    g_sample_cnt = 0;
    g_sample_sum = 0;
    memset(g_results, 0, sizeof(g_results));
    g_last_sample_tick = 0;
}

static uint8_t Calibration_SampleOnce(void)
{
    if (g_k230_data.x == 65535)
        return 0;
    uint16_t x = g_k230_data.x;
    if (x > 800)
        return 0;
    g_sample_sum += x;
    g_sample_cnt++;
    return 1;
}

static void Calibration_FinishStep(void)
{
    float avg = 0.0f;
    if (g_sample_cnt > 0)
        avg = (float)g_sample_sum / (float)g_sample_cnt;
    switch (g_cal_step)
    {
    case CAL_STEP_CENTER:
        g_results[0] = avg;
        g_cal_step = CAL_STEP_PLUS;
        break;
    case CAL_STEP_PLUS:
        g_results[1] = avg;
        g_cal_step = CAL_STEP_MINUS;
        break;
    case CAL_STEP_MINUS:
        g_results[2] = avg;
        g_cal_step = CAL_STEP_RESULT;
        break;
    default:
        break;
    }
    g_sample_cnt = 0;
    g_sample_sum = 0;
}

uint8_t Calibration_Process(uint8_t key_event, uint8_t key_l_event)
{
    if (!g_cal_active)
        return 0;
    uint32_t now = HAL_GetTick();
    if (key_l_event == KEY_EVENT_LONG && g_cal_step != CAL_STEP_RESULT)
    {
        g_cal_active = 0;
        return 2;
    }
    if (g_cal_step == CAL_STEP_CENTER || g_cal_step == CAL_STEP_PLUS || g_cal_step == CAL_STEP_MINUS)
    {
        if (key_event == KEY_EVENT_SHORT)
        {
            if (g_sample_cnt == 0)
            {
                g_sample_sum = 0;
                g_last_sample_tick = now;
                Calibration_SampleOnce(); /* 立即采集第一个样本, g_sample_cnt → 1 */
            }
        }
        if (g_sample_cnt > 0 && g_sample_cnt < 15)
        {
            if (now - g_last_sample_tick >= 30)
            {
                g_last_sample_tick = now;
                Calibration_SampleOnce();
            }
        }
        if (g_sample_cnt >= 15)
        {
            Calibration_FinishStep();
        }
    }
    if (g_cal_step == CAL_STEP_RESULT)
    {
        if (key_event == KEY_EVENT_SHORT)
        {
            g_cal_data.n0 = g_results[0];
            g_cal_data.n1 = g_results[1];
            g_cal_data.n2 = g_results[2];
            Calibration_FlashSave();
            g_cal_active = 0;
            return 1;
        }
        if (key_event == KEY_EVENT_LONG)
        {
            g_cal_active = 0;
            return 2;
        }
    }
    return 0;
}

/* ========================== Queries ================================== */

CalStep Calibration_GetStep(void) { return g_cal_step; }
uint8_t Calibration_GetSampleProgress(void) { return g_sample_cnt; }

float Calibration_GetResult(uint8_t step_index)
{
    if (step_index < 3)
        return g_results[step_index];
    return 0.0f;
}

uint8_t Calibration_IsValid(void)
{
    return (g_cal_data.magic == 0xCA11B8A0) ? 1 : 0;
}

float Calibration_PixelToCm(uint16_t pixel_x)
{
    if (!Calibration_IsValid())
        return 0.0f;
    float n0 = g_cal_data.n0;
    float n1 = g_cal_data.n1;
    float n2 = g_cal_data.n2;
    float denom = n1 - n2;
    if (denom < 0.1f && denom > -0.1f)
        denom = 1.0f;
    float slope = 10.0f / denom; /* 标距 = 10cm (±5cm) */
    float offset = -slope * n0;
    return offset + slope * (float)pixel_x;
}

/**
 * @brief 将物理 cm 转为像素坐标 (像素 = (cm - offset) / slope)
 */
float Calibration_CmToPixel(float cm)
{
    if (!Calibration_IsValid())
        return 400.0f; /* 默认中心 */
    float n0 = g_cal_data.n0;
    float n1 = g_cal_data.n1;
    float n2 = g_cal_data.n2;
    float denom = n1 - n2;
    if (denom < 0.1f && denom > -0.1f)
        denom = 1.0f;
    float slope = 10.0f / denom;
    float offset = -slope * n0;
    return (cm - offset) / slope;
}
