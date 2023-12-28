/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define DCC_Pin GPIO_PIN_0
#define DCC_GPIO_Port GPIOA
#define MULT1_Pin GPIO_PIN_1
#define MULT1_GPIO_Port GPIOA
#define RC2_RX_Pin GPIO_PIN_3
#define RC2_RX_GPIO_Port GPIOA
#define CUTOUT_Pin GPIO_PIN_4
#define CUTOUT_GPIO_Port GPIOA
#define DEBUG1_Pin GPIO_PIN_5
#define DEBUG1_GPIO_Port GPIOA
#define DEBUG2_Pin GPIO_PIN_6
#define DEBUG2_GPIO_Port GPIOA
#define ADDR1_Pin GPIO_PIN_0
#define ADDR1_GPIO_Port GPIOB
#define ADDR2_Pin GPIO_PIN_1
#define ADDR2_GPIO_Port GPIOB
#define ADDR4_Pin GPIO_PIN_2
#define ADDR4_GPIO_Port GPIOB
#define MTB_TX_Pin GPIO_PIN_10
#define MTB_TX_GPIO_Port GPIOB
#define MTB_RX_Pin GPIO_PIN_11
#define MTB_RX_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_12
#define LED_RED_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_13
#define LED_GREEN_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_14
#define LED_BLUE_GPIO_Port GPIOB
#define BUTTON_Pin GPIO_PIN_15
#define BUTTON_GPIO_Port GPIOB
#define RC1_RX_Pin GPIO_PIN_10
#define RC1_RX_GPIO_Port GPIOA
#define ADDR8_Pin GPIO_PIN_3
#define ADDR8_GPIO_Port GPIOB
#define ADDR16_Pin GPIO_PIN_4
#define ADDR16_GPIO_Port GPIOB
#define ADDR32_Pin GPIO_PIN_5
#define ADDR32_GPIO_Port GPIOB
#define ADDR64_Pin GPIO_PIN_6
#define ADDR64_GPIO_Port GPIOB
#define ADDR128_Pin GPIO_PIN_7
#define ADDR128_GPIO_Port GPIOB
#define MTB_TE_Pin GPIO_PIN_9
#define MTB_TE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
