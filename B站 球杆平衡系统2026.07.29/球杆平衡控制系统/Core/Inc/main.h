/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define UserLED_Pin GPIO_PIN_8
#define UserLED_GPIO_Port GPIOE
#define IIC_SCL_Pin GPIO_PIN_10
#define IIC_SCL_GPIO_Port GPIOB
#define IIC_SDA_Pin GPIO_PIN_11
#define IIC_SDA_GPIO_Port GPIOB
#define STEPPER_EN_Pin GPIO_PIN_14
#define STEPPER_EN_GPIO_Port GPIOB
#define STEPPER_DIR1_Pin GPIO_PIN_8
#define STEPPER_DIR1_GPIO_Port GPIOC
#define OLED_DC_Pin GPIO_PIN_11
#define OLED_DC_GPIO_Port GPIOD
#define OLED_RES_Pin GPIO_PIN_12
#define OLED_RES_GPIO_Port GPIOD
#define OLED_SDA_Pin GPIO_PIN_13
#define OLED_SDA_GPIO_Port GPIOD
#define OLED_SCL_Pin GPIO_PIN_14
#define OLED_SCL_GPIO_Port GPIOD
#define UserBuzzer_Pin GPIO_PIN_8
#define UserBuzzer_GPIO_Port GPIOA
#define ENKey_Pin GPIO_PIN_3
#define ENKey_GPIO_Port GPIOD
#define UserKEY_Pin GPIO_PIN_0
#define UserKEY_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
