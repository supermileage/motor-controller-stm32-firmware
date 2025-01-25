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
#include "stm32l4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOA
#define Hall_Sensor_C_Pin GPIO_PIN_1
#define Hall_Sensor_C_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define Hall_Sensor_B_Pin GPIO_PIN_3
#define Hall_Sensor_B_GPIO_Port GPIOA
#define Hall_Sensor_A_Pin GPIO_PIN_4
#define Hall_Sensor_A_GPIO_Port GPIOA
#define Throttle_Pin GPIO_PIN_5
#define Throttle_GPIO_Port GPIOA
#define Mosfet_C_LOW_Pin GPIO_PIN_8
#define Mosfet_C_LOW_GPIO_Port GPIOA
#define Mosfet_C_HIGH_Pin GPIO_PIN_9
#define Mosfet_C_HIGH_GPIO_Port GPIOA
#define Mosfet_B_HIGH_Pin GPIO_PIN_10
#define Mosfet_B_HIGH_GPIO_Port GPIOA
#define Mosfet_A_HIGH_Pin GPIO_PIN_11
#define Mosfet_A_HIGH_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define Mosfet_A_LOW_Pin GPIO_PIN_5
#define Mosfet_A_LOW_GPIO_Port GPIOB
#define Mosfet_B_LOW_Pin GPIO_PIN_6
#define Mosfet_B_LOW_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
