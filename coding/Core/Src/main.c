/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Door TIM2->CCR2
#define MotorSpeed TIM2->CCR3
#define directionDown HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
#define directionUp HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET); HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
bool floor1, floor2, floor3, floor4 = false;
bool isDirectionDown, isDirectionUp = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void zero();
void one();
void two();
void three();
void four();
void five();
void six();
void seven();
void eight();
void nine();
void digitClear();
void controllingMotor();
void initialStageDetect();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

	for (int i = 0; i < 100; i++) {
		Door = i;
		HAL_Delay(10);
	}
//	Door = 10;
	MotorSpeed = 25;

	zero();
	HAL_Delay(200);
	one();
	HAL_Delay(200);
	two();
	HAL_Delay(200);
	three();
	HAL_Delay(200);
	four();
	HAL_Delay(200);
	five();
	HAL_Delay(200);
	six();
	HAL_Delay(200);
	seven();
	HAL_Delay(200);
	eight();
	HAL_Delay(200);
	nine();
	HAL_Delay(200);
	digitClear();
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(stopLED_GPIO_Port, stopLED_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(stopLED_GPIO_Port, stopLED_Pin, GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(upLED_GPIO_Port, upLED_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(upLED_GPIO_Port, upLED_Pin, GPIO_PIN_RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(downLED_GPIO_Port, downLED_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(downLED_GPIO_Port, downLED_Pin, GPIO_PIN_RESET);
	HAL_Delay(200);

     initialStageDetect();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (floor1) {
			floor1 = false;
			one();
		}
		if (floor2) {
			floor2 = false;
			two();
		}
		if (floor3) {
			floor3 = false;
			three();
		}
		if (floor4) {
			floor4 = false;
			four();
		}
		directionDown
		;
		HAL_Delay(3000);
		directionUp
		;
		HAL_Delay(6000);
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 6400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, stopLED_Pin|LED3_Pin|LED4_Pin|LED2_Pin
                          |LED1_Pin|upLED_Pin|downLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IN1_Pin|IN2_Pin|segmentA_Pin|segmentE_Pin
                          |segmentB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, segmentG_Pin|segmentC_Pin|segmentF_Pin|segmentD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : stopLED_Pin LED3_Pin LED4_Pin LED2_Pin
                           LED1_Pin upLED_Pin downLED_Pin */
  GPIO_InitStruct.Pin = stopLED_Pin|LED3_Pin|LED4_Pin|LED2_Pin
                          |LED1_Pin|upLED_Pin|downLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin IN2_Pin segmentA_Pin segmentE_Pin
                           segmentB_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin|segmentA_Pin|segmentE_Pin
                          |segmentB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BOOT1_Pin F1BU_Pin F2BD_Pin F2BU_Pin
                           F3BD_Pin F3BU_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin|F1BU_Pin|F2BD_Pin|F2BU_Pin
                          |F3BD_Pin|F3BU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN1_Pin BTN2_Pin BTN3_Pin BTN4_Pin
                           STOP_BTN_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin|BTN2_Pin|BTN3_Pin|BTN4_Pin
                          |STOP_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : F4BD_Pin */
  GPIO_InitStruct.Pin = F4BD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(F4BD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : segmentG_Pin segmentC_Pin segmentF_Pin segmentD_Pin */
  GPIO_InitStruct.Pin = segmentG_Pin|segmentC_Pin|segmentF_Pin|segmentD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : F1SD_Pin F1SU_Pin F2SD_Pin F2SU_Pin
                           F3SU_Pin F4SD_Pin F4SU_Pin */
  GPIO_InitStruct.Pin = F1SD_Pin|F1SU_Pin|F2SD_Pin|F2SU_Pin
                          |F3SU_Pin|F4SD_Pin|F4SU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : F3SD_Pin */
  GPIO_InitStruct.Pin = F3SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(F3SD_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void zero() {
	HAL_GPIO_WritePin(segmentA_GPIO_Port, segmentA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentB_GPIO_Port, segmentB_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentC_GPIO_Port, segmentC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentD_GPIO_Port, segmentD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentE_GPIO_Port, segmentE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentF_GPIO_Port, segmentF_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentG_GPIO_Port, segmentG_Pin, GPIO_PIN_RESET);
}

void one() {
	HAL_GPIO_WritePin(segmentA_GPIO_Port, segmentA_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentB_GPIO_Port, segmentB_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentC_GPIO_Port, segmentC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentD_GPIO_Port, segmentD_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentE_GPIO_Port, segmentE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentF_GPIO_Port, segmentF_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentG_GPIO_Port, segmentG_Pin, GPIO_PIN_RESET);
}

void two() {
	HAL_GPIO_WritePin(segmentA_GPIO_Port, segmentA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentB_GPIO_Port, segmentB_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentC_GPIO_Port, segmentC_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentD_GPIO_Port, segmentD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentE_GPIO_Port, segmentE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentF_GPIO_Port, segmentF_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentG_GPIO_Port, segmentG_Pin, GPIO_PIN_SET);
}

void three() {
	HAL_GPIO_WritePin(segmentA_GPIO_Port, segmentA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentB_GPIO_Port, segmentB_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentC_GPIO_Port, segmentC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentD_GPIO_Port, segmentD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentE_GPIO_Port, segmentE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentF_GPIO_Port, segmentF_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentG_GPIO_Port, segmentG_Pin, GPIO_PIN_SET);
}

void four() {
	HAL_GPIO_WritePin(segmentA_GPIO_Port, segmentA_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentB_GPIO_Port, segmentB_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentC_GPIO_Port, segmentC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentD_GPIO_Port, segmentD_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentE_GPIO_Port, segmentE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentF_GPIO_Port, segmentF_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentG_GPIO_Port, segmentG_Pin, GPIO_PIN_SET);
}

void five() {
	HAL_GPIO_WritePin(segmentA_GPIO_Port, segmentA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentB_GPIO_Port, segmentB_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentC_GPIO_Port, segmentC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentD_GPIO_Port, segmentD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentE_GPIO_Port, segmentE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentF_GPIO_Port, segmentF_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentG_GPIO_Port, segmentG_Pin, GPIO_PIN_SET);
}

void six() {
	HAL_GPIO_WritePin(segmentA_GPIO_Port, segmentA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentB_GPIO_Port, segmentB_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentC_GPIO_Port, segmentC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentD_GPIO_Port, segmentD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentE_GPIO_Port, segmentE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentF_GPIO_Port, segmentF_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentG_GPIO_Port, segmentG_Pin, GPIO_PIN_SET);
}

void seven() {
	HAL_GPIO_WritePin(segmentA_GPIO_Port, segmentA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentB_GPIO_Port, segmentB_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentC_GPIO_Port, segmentC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentD_GPIO_Port, segmentD_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentE_GPIO_Port, segmentE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentF_GPIO_Port, segmentF_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentG_GPIO_Port, segmentG_Pin, GPIO_PIN_RESET);
}

void eight() {
	HAL_GPIO_WritePin(segmentA_GPIO_Port, segmentA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentB_GPIO_Port, segmentB_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentC_GPIO_Port, segmentC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentD_GPIO_Port, segmentD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentE_GPIO_Port, segmentE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentF_GPIO_Port, segmentF_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentG_GPIO_Port, segmentG_Pin, GPIO_PIN_SET);
}

void nine() {
	HAL_GPIO_WritePin(segmentA_GPIO_Port, segmentA_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentB_GPIO_Port, segmentB_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentC_GPIO_Port, segmentC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentD_GPIO_Port, segmentD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentE_GPIO_Port, segmentE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentF_GPIO_Port, segmentF_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(segmentG_GPIO_Port, segmentG_Pin, GPIO_PIN_SET);
}

void digitClear() {
	HAL_GPIO_WritePin(segmentA_GPIO_Port, segmentA_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentB_GPIO_Port, segmentB_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentC_GPIO_Port, segmentC_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentD_GPIO_Port, segmentD_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentE_GPIO_Port, segmentE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentF_GPIO_Port, segmentF_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentG_GPIO_Port, segmentG_Pin, GPIO_PIN_RESET);
}

void controllingMotor() {
	//motorSpeed = 50;

}

void initialStageDetect() {

	if (HAL_GPIO_ReadPin(F1SU_GPIO_Port, F1SU_Pin) == 0
			&& HAL_GPIO_ReadPin(F1SD_GPIO_Port, F1SD_Pin) == 0) {
		return;
	}
	if (HAL_GPIO_ReadPin(F2SU_GPIO_Port, F2SU_Pin) == 0
			&& HAL_GPIO_ReadPin(F2SD_GPIO_Port, F2SD_Pin) == 0) {
		return;
	}
	if (HAL_GPIO_ReadPin(F3SU_GPIO_Port, F3SU_Pin) == 0
			&& HAL_GPIO_ReadPin(F3SD_GPIO_Port, F3SD_Pin) == 0) {
		return;
	}
	if (HAL_GPIO_ReadPin(F4SU_GPIO_Port, F4SU_Pin) == 0
			&& HAL_GPIO_ReadPin(F4SD_GPIO_Port, F4SD_Pin) == 0) {
		return;
	}
	if (HAL_GPIO_ReadPin(F1SU_GPIO_Port, F1SU_Pin) == 0) {
		directionDown
		;
		isDirectionDown = true;
		MotorSpeed = 25;
		return;
	}
	if (HAL_GPIO_ReadPin(F2SU_GPIO_Port, F2SU_Pin) == 0) {
		directionDown
		;
		isDirectionDown = true;
		MotorSpeed = 25;
		return;

	}
	if (HAL_GPIO_ReadPin(F3SU_GPIO_Port, F3SU_Pin) == 0) {
		directionDown
		;
		isDirectionDown = true;
		MotorSpeed = 25;
		return;
	}
	if (HAL_GPIO_ReadPin(F4SU_GPIO_Port, F4SU_Pin) == 0) {
		directionDown
		;
		isDirectionDown = true;
		MotorSpeed = 25;
		return;
	}
	if (HAL_GPIO_ReadPin(F1SD_GPIO_Port, F1SD_Pin) == 0) {
		directionUp
		;
		isDirectionUp = true;
		MotorSpeed = 25;
		return;
	}
	if (HAL_GPIO_ReadPin(F2SD_GPIO_Port, F2SD_Pin) == 0) {
		directionUp
		;
		isDirectionUp = true;
		MotorSpeed = 25;
		return;
	}
	if (HAL_GPIO_ReadPin(F3SD_GPIO_Port, F3SD_Pin) == 0) {
		directionUp
		;
		isDirectionUp = true;
		MotorSpeed = 25;
		return;
	}
	if (HAL_GPIO_ReadPin(F4SD_GPIO_Port, F4SD_Pin) == 0) {
		directionUp
		;
		isDirectionUp = true;
		MotorSpeed = 25;
		return;
	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (isDirectionDown && GPIO_Pin == F1SD_Pin) {
		MotorSpeed = 0;
		isDirectionDown = false;
	}
	if (isDirectionDown && GPIO_Pin == F2SD_Pin) {
		MotorSpeed = 0;
		isDirectionDown = false;
	}
	if (isDirectionDown && GPIO_Pin == F3SD_Pin) {
		MotorSpeed = 0;
		isDirectionDown = false;
	}
	if (isDirectionDown && GPIO_Pin == F4SD_Pin) {
		MotorSpeed = 0;
		isDirectionDown = false;
	}
	if (isDirectionUp && GPIO_Pin == F1SU_Pin) {
		MotorSpeed = 0;
		isDirectionUp = false;
	}
	if (isDirectionUp && GPIO_Pin == F2SU_Pin) {
		MotorSpeed = 0;
		isDirectionUp = false;
	}
	if (isDirectionUp && GPIO_Pin == F3SU_Pin) {
		MotorSpeed = 0;
		isDirectionUp = false;
	}
	if (isDirectionUp && GPIO_Pin == F4SU_Pin) {
		MotorSpeed = 0;
		isDirectionUp = false;
	}

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
