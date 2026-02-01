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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
// inline GPIO helpers to reduce HAL equivalents function call overheads
static inline uint8_t gpio_read(GPIO_TypeDef *port, uint16_t pin)
{
	return (port->IDR & pin) ? 1 : 0;
}
static inline void gpio_write(GPIO_TypeDef *port, uint16_t pin, bool on)
{
	if (on) port->BSRR = pin;
	else port->BRR = pin;
}
static inline uint16_t adc_read(const ADC_HandleTypeDef *hadc)
{
	return hadc->Instance->DR;
}
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HALL_C_Pin GPIO_PIN_1
#define HALL_C_GPIO_Port GPIOA
#define HALL_C_EXTI_IRQn EXTI1_IRQn
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define HALL_B_Pin GPIO_PIN_3
#define HALL_B_GPIO_Port GPIOA
#define HALL_B_EXTI_IRQn EXTI3_IRQn
#define HALL_A_Pin GPIO_PIN_4
#define HALL_A_GPIO_Port GPIOA
#define HALL_A_EXTI_IRQn EXTI4_IRQn
#define THROTTLE_Pin GPIO_PIN_5
#define THROTTLE_GPIO_Port GPIOA
#define MOSFET_B_HI_Pin GPIO_PIN_0
#define MOSFET_B_HI_GPIO_Port GPIOB
#define MOSFET_C_HI_Pin GPIO_PIN_1
#define MOSFET_C_HI_GPIO_Port GPIOB
#define MOSFET_C_LO_Pin GPIO_PIN_8
#define MOSFET_C_LO_GPIO_Port GPIOA
#define MOSFET_A_HI_Pin GPIO_PIN_11
#define MOSFET_A_HI_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define MOSFET_A_LO_Pin GPIO_PIN_5
#define MOSFET_A_LO_GPIO_Port GPIOB
#define MOSFET_B_LO_Pin GPIO_PIN_6
#define MOSFET_B_LO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
