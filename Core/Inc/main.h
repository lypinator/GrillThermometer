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
#include "stm32g0xx_hal.h"

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
#define TC_OUTPUT_Pin GPIO_PIN_4
#define TC_OUTPUT_GPIO_Port GPIOA
#define EXTRA_LED3_Pin GPIO_PIN_5
#define EXTRA_LED3_GPIO_Port GPIOA
#define Beeper_Pin GPIO_PIN_7
#define Beeper_GPIO_Port GPIOA
#define SW4_Pin GPIO_PIN_0
#define SW4_GPIO_Port GPIOB
#define POTENTIOMETER_Pin GPIO_PIN_1
#define POTENTIOMETER_GPIO_Port GPIOB
#define CJ_TEMP_Pin GPIO_PIN_12
#define CJ_TEMP_GPIO_Port GPIOB
#define SW3_Pin GPIO_PIN_8
#define SW3_GPIO_Port GPIOA
#define EXTRA_LED1_Pin GPIO_PIN_9
#define EXTRA_LED1_GPIO_Port GPIOA
#define EXTRA_LED2_Pin GPIO_PIN_7
#define EXTRA_LED2_GPIO_Port GPIOC
#define SW1_Pin GPIO_PIN_10
#define SW1_GPIO_Port GPIOA
#define SW2_Pin GPIO_PIN_5
#define SW2_GPIO_Port GPIOB
#define SW4B6_Pin GPIO_PIN_6
#define SW4B6_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define false 0
#define true  1

extern uint32_t timerValue;
extern TIM_HandleTypeDef htim17;

#define START_COOKING_TIMER timerValue = 0; \
                            HAL_TIM_Base_Init(&htim17); \
                            HAL_TIM_Base_Start_IT(&htim17); 

#define STOP_COOKING_TIMER  HAL_TIM_Base_Stop_IT(&htim17);



//#define START_COOKING_TIMER timerValue = 0;


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
