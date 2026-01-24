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
#include "stm32g4xx_hal.h"

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
#define LED_G_Pin GPIO_PIN_13
#define LED_G_GPIO_Port GPIOC
#define LED_R_Pin GPIO_PIN_14
#define LED_R_GPIO_Port GPIOC
#define LED_B_Pin GPIO_PIN_15
#define LED_B_GPIO_Port GPIOC
#define SPI1_NSS_Pin GPIO_PIN_0
#define SPI1_NSS_GPIO_Port GPIOB
#define TEST1_Pin GPIO_PIN_10
#define TEST1_GPIO_Port GPIOB
#define TEST2_Pin GPIO_PIN_11
#define TEST2_GPIO_Port GPIOB
#define MT6835_CS_Pin GPIO_PIN_12
#define MT6835_CS_GPIO_Port GPIOB
#define MT6816_CS_Pin GPIO_PIN_15
#define MT6816_CS_GPIO_Port GPIOA
#define ENABLE_Pin GPIO_PIN_6
#define ENABLE_GPIO_Port GPIOB
#define PWML_Pin GPIO_PIN_7
#define PWML_GPIO_Port GPIOB
#define STB_Pin GPIO_PIN_9
#define STB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
