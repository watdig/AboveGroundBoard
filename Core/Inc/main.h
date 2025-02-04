/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32c0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define NUM_HOLDING_REGISTERS 59
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCU_DCV_A_Pin GPIO_PIN_0
#define MCU_DCV_A_GPIO_Port GPIOB
#define MCU_DCV_B_Pin GPIO_PIN_1
#define MCU_DCV_B_GPIO_Port GPIOB
#define HPU_Gate_Pin GPIO_PIN_12
#define HPU_Gate_GPIO_Port GPIOB
#define Oil_High_Pin GPIO_PIN_15
#define Oil_High_GPIO_Port GPIOA
#define Oil_Low_Pin GPIO_PIN_0
#define Oil_Low_GPIO_Port GPIOD
#define Oil_E_Stop_Pin GPIO_PIN_1
#define Oil_E_Stop_GPIO_Port GPIOD
#define Water_Solinoid_Pin GPIO_PIN_2
#define Water_Solinoid_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
