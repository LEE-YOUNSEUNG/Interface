/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */


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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  char rx_msg[50]; //Set up receive massage transmit massage array
  void forward(uint32_t dutyCycle, uint32_t delay_ms);
  void backward(uint32_t dutyCycle, uint32_t delay_ms);
  void turnRight(uint32_t initialDutyCycle, uint32_t minDutyCycle, uint32_t delay_ms);
  void turnLeft(uint32_t initialDutyCycle, uint32_t minDutyCycle, uint32_t delay_ms);


 {

 }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_UART_Receive(&huart1, (uint8_t*)rx_msg, 50, 2000);
	  if(rx_msg[0]=='L'&&rx_msg[1]=='e'&&rx_msg[2]=='f'&&rx_msg[3]=='t')
	  {
	  turnLeft(1000,300, 100);
	  	  for(int i=0;i<4;i++)
	  	  {
		  rx_msg[i]=0;      //reset
	  	  }
	  }

	  else if(rx_msg[0]=='R'&&rx_msg[1]=='i'&&rx_msg[2]=='g'&&rx_msg[3]=='h'&&rx_msg[4]=='t')
	  {
	  turnRight(1000,300,100);
	  	  for(int i=0;i<4;i++)
	  	  {
	  	  rx_msg[i]=0;      //reset
	  	  }
	  }
	  else if(rx_msg[0]=='f'&&rx_msg[1]=='o'&&rx_msg[2]=='w'&&rx_msg[3]=='a'&&rx_msg[4]=='r'&&rx_msg[5]=='d')
	  {
	  forward(1000,1000);
	  	  for(int i=0;i<4;i++)
	  	  	  {
	  	  	  rx_msg[i]=0;      //reset
	  	  	  }
	  }
	  else if(rx_msg[0]=='b'&&rx_msg[1]=='a'&&rx_msg[2]=='c'&&rx_msg[3]=='k'&&rx_msg[4]=='w'&&rx_msg[5]=='a'&&rx_msg[6]=='r'&&rx_msg[7]=='d')
	  {
	  backward(1000,1000);
	  	  for(int i=0;i<4;i++)
	  	  	  {
	  	  	  rx_msg[i]=0;      //reset
	  	  	  }
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 64-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 65535;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, motor_1_IN1_Pin|motor_1_IN2_Pin|motor_2_IN3_Pin|motor_2_IN4_Pin
                          |motor_4_IN3_Pin|motor_4_IN4_Pin|motor_3_IN1_Pin|motor_3_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : motor_1_IN1_Pin motor_1_IN2_Pin motor_2_IN3_Pin motor_2_IN4_Pin
                           motor_4_IN3_Pin motor_4_IN4_Pin motor_3_IN1_Pin motor_3_IN2_Pin */
  GPIO_InitStruct.Pin = motor_1_IN1_Pin|motor_1_IN2_Pin|motor_2_IN3_Pin|motor_2_IN4_Pin
                          |motor_4_IN3_Pin|motor_4_IN4_Pin|motor_3_IN1_Pin|motor_3_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void forward(uint32_t dutyCycle, uint32_t delay_ms) {

    HAL_GPIO_WritePin(GPIOC, motor_1_IN1_Pin|motor_2_IN3_Pin|motor_3_IN1_Pin|motor_4_IN3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, motor_1_IN2_Pin|motor_2_IN4_Pin|motor_3_IN2_Pin|motor_4_IN4_Pin, GPIO_PIN_RESET);


    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);


    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dutyCycle);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dutyCycle);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dutyCycle);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, dutyCycle);


    HAL_Delay(delay_ms);


    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
}
void backward(uint32_t dutyCycle, uint32_t delay_ms) {
    // 1. 媛? 紐⑦��?�� 諛⑺�� ?��?�� 諛? PWM 梨��� ?��?��?��
    HAL_GPIO_WritePin(GPIOC, motor_1_IN1_Pin|motor_2_IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, motor_1_IN2_Pin|motor_2_IN4_Pin, GPIO_PIN_SET);

    // PWM 梨��� ?��?��
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    // PWM duty cycle ?��?��
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dutyCycle);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dutyCycle);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dutyCycle);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, dutyCycle);

    // ?��?�� ?��媛? ?��?�� ??湲?
    HAL_Delay(delay_ms);

    // 2. 紐⑦�� ?��吏? (PWM duty cycle?�� 0?�쇰�? ?��?��?��?�� 紐⑦�� ?��吏?)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
}

void turnLeft(uint32_t initialDutyCycle, uint32_t minDutyCycle, uint32_t delay_ms) {
    // 1. 紐⑦�� 1踰�怨� 3踰��� 諛⑺��?�� ?��?��?��?�� PWM 異���
    HAL_GPIO_WritePin(GPIOC, motor_1_IN1_Pin | motor_3_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, motor_1_IN2_Pin | motor_3_IN2_Pin, GPIO_PIN_RESET);

    // PWM 梨��� ?��?��
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    // 珥�湲� ???�� ?��?��?�� ?��?��
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, initialDutyCycle);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, initialDutyCycle);

    // ???�� ?��?��?�� 媛��� 猷⑦��
    for (uint32_t dutyCycle = initialDutyCycle; dutyCycle >= minDutyCycle; dutyCycle -= 10) {
        // ???�� ?��?��?�� ?��?��
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dutyCycle);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dutyCycle);
        // 吏??�� ?��媛? ??湲?
        HAL_Delay(delay_ms);
    }

    // 紐⑦�� ?��吏? (PWM duty cycle?�� 0?�쇰�? ?��?��?��?�� 紐⑦�곕�? ?��吏?)

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
}
void turnRight(uint32_t initialDutyCycle, uint32_t minDutyCycle, uint32_t delay_ms) {
    // 1. 紐⑦�� 2踰�怨� 4踰��� 諛⑺��?�� ?��?��?��?�� PWM 異���
    HAL_GPIO_WritePin(GPIOC, motor_2_IN3_Pin | motor_4_IN3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, motor_2_IN4_Pin | motor_4_IN4_Pin, GPIO_PIN_RESET);

    // PWM 梨��� ?��?��

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    // 珥�湲� ???�� ?��?��?�� ?��?��
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, initialDutyCycle);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, initialDutyCycle);

    // ???�� ?��?��?�� 媛��� 猷⑦��
    for (uint32_t dutyCycle = initialDutyCycle; dutyCycle >= minDutyCycle; dutyCycle -= 10) {
        // ???�� ?��?��?�� ?��?��
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dutyCycle);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, dutyCycle);
        // 吏??�� ?��媛? ??湲?
        HAL_Delay(delay_ms);
    }

    // 紐⑦�� ?��吏? (PWM duty cycle?�� 0?�쇰�? ?��?��?��?�� 紐⑦�곕�? ?��吏?)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
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
  while (1)
  {
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
