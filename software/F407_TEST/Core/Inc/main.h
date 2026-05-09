/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define ENCODE1_A_Pin GPIO_PIN_0
#define ENCODE1_A_GPIO_Port GPIOA
#define ENCODE1_B_Pin GPIO_PIN_1
#define ENCODE1_B_GPIO_Port GPIOA
#define ENCODE2_A_Pin GPIO_PIN_6
#define ENCODE2_A_GPIO_Port GPIOA
#define ENCODE2B_Pin GPIO_PIN_7
#define ENCODE2B_GPIO_Port GPIOA
#define LCD_RS_Pin GPIO_PIN_5
#define LCD_RS_GPIO_Port GPIOC
#define LCD_BLK_Pin GPIO_PIN_1
#define LCD_BLK_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_12
#define LCD_CS_GPIO_Port GPIOB
#define LCD_SCL_Pin GPIO_PIN_13
#define LCD_SCL_GPIO_Port GPIOB
#define LCD_SDO_RST_Pin GPIO_PIN_14
#define LCD_SDO_RST_GPIO_Port GPIOB
#define LCD_SDA_Pin GPIO_PIN_15
#define LCD_SDA_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_10
#define AIN2_GPIO_Port GPIOD
#define BIN2_Pin GPIO_PIN_11
#define BIN2_GPIO_Port GPIOD
#define AIN1_Pin GPIO_PIN_12
#define AIN1_GPIO_Port GPIOD
#define BIN1_Pin GPIO_PIN_13
#define BIN1_GPIO_Port GPIOD
#define PWMA_Pin GPIO_PIN_14
#define PWMA_GPIO_Port GPIOD
#define PWMB_Pin GPIO_PIN_15
#define PWMB_GPIO_Port GPIOD
#define STBY_Pin GPIO_PIN_8
#define STBY_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
