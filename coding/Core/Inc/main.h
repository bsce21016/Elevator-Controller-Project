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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define stopLED_Pin GPIO_PIN_2
#define stopLED_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOE
#define LED4_Pin GPIO_PIN_4
#define LED4_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOE
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define Door_Pin GPIO_PIN_1
#define Door_GPIO_Port GPIOA
#define MotorSpeed_Pin GPIO_PIN_2
#define MotorSpeed_GPIO_Port GPIOA
#define IN1_Pin GPIO_PIN_4
#define IN1_GPIO_Port GPIOA
#define IN2_Pin GPIO_PIN_5
#define IN2_GPIO_Port GPIOA
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define upLED_Pin GPIO_PIN_7
#define upLED_GPIO_Port GPIOE
#define downLED_Pin GPIO_PIN_8
#define downLED_GPIO_Port GPIOE
#define BTN1_Pin GPIO_PIN_10
#define BTN1_GPIO_Port GPIOE
#define BTN1_EXTI_IRQn EXTI15_10_IRQn
#define BTN2_Pin GPIO_PIN_11
#define BTN2_GPIO_Port GPIOE
#define BTN2_EXTI_IRQn EXTI15_10_IRQn
#define BTN3_Pin GPIO_PIN_12
#define BTN3_GPIO_Port GPIOE
#define BTN3_EXTI_IRQn EXTI15_10_IRQn
#define BTN4_Pin GPIO_PIN_13
#define BTN4_GPIO_Port GPIOE
#define BTN4_EXTI_IRQn EXTI15_10_IRQn
#define STOP_BTN_Pin GPIO_PIN_14
#define STOP_BTN_GPIO_Port GPIOE
#define STOP_BTN_EXTI_IRQn EXTI15_10_IRQn
#define F1BU_Pin GPIO_PIN_10
#define F1BU_GPIO_Port GPIOB
#define F2BD_Pin GPIO_PIN_11
#define F2BD_GPIO_Port GPIOB
#define F2BU_Pin GPIO_PIN_12
#define F2BU_GPIO_Port GPIOB
#define F3BD_Pin GPIO_PIN_13
#define F3BD_GPIO_Port GPIOB
#define F3BU_Pin GPIO_PIN_14
#define F3BU_GPIO_Port GPIOB
#define F4BD_Pin GPIO_PIN_8
#define F4BD_GPIO_Port GPIOD
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define segmentG_Pin GPIO_PIN_6
#define segmentG_GPIO_Port GPIOC
#define segmentC_Pin GPIO_PIN_7
#define segmentC_GPIO_Port GPIOC
#define segmentF_Pin GPIO_PIN_8
#define segmentF_GPIO_Port GPIOC
#define segmentD_Pin GPIO_PIN_9
#define segmentD_GPIO_Port GPIOC
#define segmentA_Pin GPIO_PIN_8
#define segmentA_GPIO_Port GPIOA
#define segmentE_Pin GPIO_PIN_9
#define segmentE_GPIO_Port GPIOA
#define segmentB_Pin GPIO_PIN_10
#define segmentB_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define F1SD_Pin GPIO_PIN_0
#define F1SD_GPIO_Port GPIOD
#define F1SD_EXTI_IRQn EXTI0_IRQn
#define F1SU_Pin GPIO_PIN_1
#define F1SU_GPIO_Port GPIOD
#define F1SU_EXTI_IRQn EXTI1_IRQn
#define F2SD_Pin GPIO_PIN_2
#define F2SD_GPIO_Port GPIOD
#define F2SD_EXTI_IRQn EXTI2_IRQn
#define F2SU_Pin GPIO_PIN_3
#define F2SU_GPIO_Port GPIOD
#define F2SU_EXTI_IRQn EXTI3_IRQn
#define F3SU_Pin GPIO_PIN_5
#define F3SU_GPIO_Port GPIOD
#define F3SU_EXTI_IRQn EXTI9_5_IRQn
#define F4SD_Pin GPIO_PIN_6
#define F4SD_GPIO_Port GPIOD
#define F4SD_EXTI_IRQn EXTI9_5_IRQn
#define F4SU_Pin GPIO_PIN_7
#define F4SU_GPIO_Port GPIOD
#define F4SU_EXTI_IRQn EXTI9_5_IRQn
#define F3SD_Pin GPIO_PIN_4
#define F3SD_GPIO_Port GPIOB
#define F3SD_EXTI_IRQn EXTI4_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
