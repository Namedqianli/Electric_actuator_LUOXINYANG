/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define KEY0_Pin GPIO_PIN_4
#define KEY0_GPIO_Port GPIOE
#define MOTO3_LINE1_Pin GPIO_PIN_0
#define MOTO3_LINE1_GPIO_Port GPIOF
#define MOTO3_LINE2_Pin GPIO_PIN_1
#define MOTO3_LINE2_GPIO_Port GPIOF
#define LOCK_Pin GPIO_PIN_10
#define LOCK_GPIO_Port GPIOF
#define NRF_CSN_Pin GPIO_PIN_3
#define NRF_CSN_GPIO_Port GPIOA
#define NRF_CE_Pin GPIO_PIN_4
#define NRF_CE_GPIO_Port GPIOA
#define MOTO1_EN_Pin GPIO_PIN_0
#define MOTO1_EN_GPIO_Port GPIOB
#define MOTO1_LINE1_Pin GPIO_PIN_12
#define MOTO1_LINE1_GPIO_Port GPIOF
#define MOTO1_LINE2_Pin GPIO_PIN_13
#define MOTO1_LINE2_GPIO_Port GPIOF
#define MOTO2_EN_Pin GPIO_PIN_14
#define MOTO2_EN_GPIO_Port GPIOF
#define MOTO2_LINE1_Pin GPIO_PIN_15
#define MOTO2_LINE1_GPIO_Port GPIOF
#define NRF_IRQ_Pin GPIO_PIN_10
#define NRF_IRQ_GPIO_Port GPIOB
#define MOTO2_LINE2_Pin GPIO_PIN_14
#define MOTO2_LINE2_GPIO_Port GPIOD
#define MOTO3_EN_Pin GPIO_PIN_15
#define MOTO3_EN_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
