/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
volatile uint16_t	direction_status = 0;
volatile uint16_t 	speed_status = 0;
volatile uint8_t	buf[1];
volatile uint8_t aTxBuffer = 0;
volatile uint8_t aRxBuffer = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void HandleCommand(uint8_t);
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	//HAL_I2C_Slave_Receive_IT(&hi2c1,(uint8_t*) buf, 1);
	if(HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t *)buf,1) != HAL_OK)
  {
    Error_Handler();
  }
	/*while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
  {
  }*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = I2C_ADDRESS;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0xFF;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA10 PA11 PA12 
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		if(direction_status & MOVE_UP_BIT)
			{
				//forward servo 2
				HAL_GPIO_WritePin(MOVE_UP_PORT, MOVE_UP_PIN, GPIO_PIN_SET);
			}
			else{
				//stop servo 2 if not backward
				if(!(direction_status & MOVE_DOWN_BIT))
				{
					HAL_GPIO_WritePin(MOVE_UP_PORT, MOVE_UP_PIN, GPIO_PIN_RESET);
				}
			}
			
			if(direction_status & MOVE_DOWN_BIT)
			{
				//backward servo 2
				HAL_GPIO_WritePin(MOVE_DOWN_PORT, MOVE_DOWN_PIN, GPIO_PIN_SET);
			}
			else{
				//stop servo 2 if not forward
				if(!(direction_status & MOVE_UP_BIT))
				{
					HAL_GPIO_WritePin(MOVE_DOWN_PORT, MOVE_DOWN_PIN, GPIO_PIN_RESET);
				}
			}
			
			if(direction_status & MOVE_LEFT_BIT)
			{
				//backward servo 1
				HAL_GPIO_WritePin(MOVE_LEFT_PORT, MOVE_LEFT_PIN, GPIO_PIN_SET);
			}
			else{
				//stop servo 1 if not forward
				if(!(direction_status & MOVE_RIGHT_BIT))
				{
					HAL_GPIO_WritePin(MOVE_LEFT_PORT, MOVE_LEFT_PIN, GPIO_PIN_RESET);
				}
			}
			
			if(direction_status & MOVE_RIGHT_BIT)
			{
				//forward servo 2
				HAL_GPIO_WritePin(MOVE_RIGHT_PORT, MOVE_RIGHT_PIN, GPIO_PIN_SET);
			}
			else{
				//stop servo 2 if not backward
				if(!(direction_status & MOVE_LEFT_BIT))
				{
					HAL_GPIO_WritePin(MOVE_RIGHT_PORT, MOVE_RIGHT_PIN, GPIO_PIN_RESET);
				}
			}
			
			if( SPEED_UP_BIT & speed_status)
			{
				//duty_cycle_1 < MAX_SERVO - 0xFF && duty_cycle_2 < MAX_SERVO - 0xFF && 
				HAL_GPIO_WritePin(SPEED_UP_PORT, SPEED_UP_PIN, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(SPEED_UP_PORT, SPEED_UP_PIN, GPIO_PIN_RESET);
			}
			if ( SPEED_DOWN_BIT & speed_status)
			{
				//duty_cycle_1 > MIN_SERVO + 0xFF && duty_cycle_2 > MIN_SERVO + 0xFF && 
				HAL_GPIO_WritePin(SPEED_DOWN_PORT, SPEED_DOWN_PIN, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(SPEED_DOWN_PORT, SPEED_DOWN_PIN, GPIO_PIN_RESET);
			}
	}
}

void HandleCommand(uint8_t command)
{
	if(command == MOVE_UP_COMMAND_END)// if master release move up
	{
		direction_status &= ~MOVE_UP_BIT;
	}
	
	if(command == MOVE_UP_COMMAND_START)// if master push move up
	{
		direction_status |= MOVE_UP_BIT;
	}
	
	if(command == MOVE_DOWN_COMMAND_END)// if master release move down
	{
		direction_status &= ~MOVE_DOWN_BIT;
	}
	
	if(command == MOVE_DOWN_COMMAND_START)// if master push move down
	{
		direction_status |= MOVE_DOWN_BIT;
	}
	
	if(command == MOVE_LEFT_COMMAND_END)// if master release move left
	{
		direction_status &= ~MOVE_LEFT_BIT;
	}
	
	if(command == MOVE_LEFT_COMMAND_START)// if master push move left
	{
		direction_status |= MOVE_LEFT_BIT;
	}
	
	if(command == MOVE_RIGHT_COMMAND_END)// if master release move right
	{
		direction_status &= ~MOVE_RIGHT_BIT;
	}
	
	if(command == MOVE_RIGHT_COMMAND_START)// if master release move right
	{
		direction_status |= MOVE_RIGHT_BIT;
	}
	
	if(command == SPEED_UP_END)// if master release speed up
	{
		speed_status &= ~SPEED_UP_BIT;
	}
	
	if(command == SPEED_UP_START)// if master push speed up
	{
		speed_status |= SPEED_UP_BIT;
	}
	
	if(command == SPEED_DOWN_END)// if master release speed down
	{
		speed_status &= ~SPEED_DOWN_BIT;
	}
	
	if(command == SPEED_DOWN_START)// if master push speed down
	{
		speed_status |= SPEED_DOWN_BIT;
	}
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	
}

/**
  * @brief  Master Rx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{

}
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	//uint8_t buf[1];
	aRxBuffer = buf[0];  
	HandleCommand(aRxBuffer);
	if(HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*) buf, 1) != HAL_OK)
	{
		Error_Handler();
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
	while(1)
	{
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		HAL_Delay(200);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
