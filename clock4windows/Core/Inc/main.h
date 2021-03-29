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
#define led13_Pin GPIO_PIN_13
#define led13_GPIO_Port GPIOC
#define DINDON_Pin GPIO_PIN_7
#define DINDON_GPIO_Port GPIOA
#define KEY_1_Pin GPIO_PIN_0
#define KEY_1_GPIO_Port GPIOB
#define KEY_2_Pin GPIO_PIN_1
#define KEY_2_GPIO_Port GPIOB
#define KEY_3_Pin GPIO_PIN_10
#define KEY_3_GPIO_Port GPIOB
#define KEY_4_Pin GPIO_PIN_11
#define KEY_4_GPIO_Port GPIOB
#define LED_SOUND_Pin GPIO_PIN_8
#define LED_SOUND_GPIO_Port GPIOA
#define ST_CP_Pin GPIO_PIN_6
#define ST_CP_GPIO_Port GPIOB
#define OE_Pin GPIO_PIN_7
#define OE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
//#define	debug_0	TRUE	//включить вывод значений RTC для отладки
//#define	debug_1	TRUE	//включить вывод значений кнопок для отладки
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
