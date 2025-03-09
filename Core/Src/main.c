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
#include "modbus.h"
#include "vl53l0x.h"
#include "error_codes.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
uint16_t holding_register_database[NUM_HOLDING_REGISTERS] = {
		0x0001,	// MODBUS_ID
		0x0003, // BAUD_RATE
		   100, // MB_TRANSMIT_TIMEOUT
		     2, // MB_TRANSMIT_RETRIES
		0x0000, // MB_ERRORS

		0x0000, // I2C_ERRORS
		0x0000, // I2C_SHUTDOWN

		0xFFFF, // ADC_0
		0xFFFF, // ADC_1
		0xFFFF, // ADC_2

		0xFFFF, // LASER_DISTANCE
		0x0000, // GPIO_READ
		0x0000, // GPIO_WRITE

};

uint16_t prev_gpio_write_register;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{

}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{

}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	int8_t modbus_status = HAL_OK;
	uint8_t modbus_tx_len = 0;
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)(&holding_register_database[ADC_0]), 3) != HAL_OK)
  {
	  Error_Handler();
  }
  // Initialise the VL53L0X
//  	statInfo_t_VL53L0X distanceStr;
//  	initVL53L0X(1, &hi2c1);
//
//  	// Configure the sensor for high accuracy and speed in 20 cm.
//  	setSignalRateLimit(200);
//  	setVcselPulsePeriod(VcselPeriodPreRange, 10);
//  	setVcselPulsePeriod(VcselPeriodFinalRange, 14);
//  	setMeasurementTimingBudget(300 * 1000UL);
//
//  	uint16_t distance;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  	// uint16_t distance is the distance in millimeters.
		// statInfo_t_VL53L0X distanceStr is the statistics read from the sensor.
//		distance = readRangeSingleMillimeters(&distanceStr);

	    // Update the GPIO_READ register
		GPIO_PinState oil_high = HAL_GPIO_ReadPin(Oil_High_GPIO_Port, Oil_High_Pin);
		GPIO_PinState oil_low = HAL_GPIO_ReadPin(Oil_Low_GPIO_Port, Oil_Low_Pin);
		GPIO_PinState oil_estop = HAL_GPIO_ReadPin(Oil_E_Stop_GPIO_Port, Oil_E_Stop_Pin);

		holding_register_database[GPIO_READ] = ((oil_high << OIL_HIGH) | (oil_low << OIL_LOW) | (oil_estop << OIL_ESTOP));

		if(prev_gpio_write_register != holding_register_database[GPIO_WRITE])
		{
			if((prev_gpio_write_register & MCU_DVA_MASK_A) != (holding_register_database[GPIO_WRITE] & MCU_DVA_MASK_A))
			{
				HAL_GPIO_WritePin(MCU_DCV_A_GPIO_Port, MCU_DCV_A_Pin, (holding_register_database[GPIO_WRITE] & MCU_DVA_MASK_A));
			}
			if((prev_gpio_write_register & MCU_DVA_MASK_B) != (holding_register_database[GPIO_WRITE] & MCU_DVA_MASK_B))
			{
				HAL_GPIO_WritePin(MCU_DCV_B_GPIO_Port, MCU_DCV_B_Pin, (holding_register_database[GPIO_WRITE] & MCU_DVA_MASK_B));
			}
			if((prev_gpio_write_register & HPU_GATE_MASK) != (holding_register_database[GPIO_WRITE] & HPU_GATE_MASK))
			{
				HAL_GPIO_WritePin(HPU_Gate_GPIO_Port, HPU_Gate_Pin, (holding_register_database[GPIO_WRITE] & HPU_GATE_MASK));
			}
			if((prev_gpio_write_register & WATER_SOLINOID_MASK) != (holding_register_database[GPIO_WRITE] & WATER_SOLINOID_MASK))
			{
				HAL_GPIO_WritePin(Water_Solinoid_GPIO_Port, Water_Solinoid_Pin, (holding_register_database[GPIO_WRITE] & WATER_SOLINOID_MASK));
			}
			prev_gpio_write_register = holding_register_database[GPIO_WRITE];
		}

		// Handle Modbus Communication
		if(modbus_rx())
		{
			if(get_rx_buffer(0) == holding_register_database[0]) // Check Slave ID
			{
				switch(get_rx_buffer(1))
				{
					case 0x03:
					{
						// Return holding registers
						modbus_status = return_holding_registers(&modbus_tx_len);
						break;
					}
					case 0x10:
					{
						// Write holding registers
						modbus_status = edit_multiple_registers(&modbus_tx_len);
						break;
					}
					default:
					{
						modbus_status = modbus_exception(MB_ILLEGAL_FUNCTION);
						break;
					}
				}
				if(modbus_status != 0)
				{
					holding_register_database[MB_ERRORS] |= 1U << (modbus_status + (MB_FATAL_ERROR - 1));
				}
			}
			// Special case where you retrieve the modbus ID
			else if((get_rx_buffer(0) == 0xFF) && // modbus_id = 0xFF = 255
			(get_rx_buffer(1) == 0x03) && // Function code = read_holding_registers
			(((get_rx_buffer(2) << 8) | get_rx_buffer(3)) == 0x00) && // Address to read = 0
			(((get_rx_buffer(4) << 8) | get_rx_buffer(5)) == 1)) // # of registers to read = 1
			{
				modbus_status = return_holding_registers(&modbus_tx_len);
				if(modbus_status != 0)
				{
					holding_register_database[MB_ERRORS] |= 1U << ((modbus_status - 1) + MB_FATAL_ERROR);
				}
			}
			modbus_status = modbus_set_rx();
			if(modbus_status != 0)
			{
				holding_register_database[MB_ERRORS] |= 1U << ((modbus_status - 1) + MB_FATAL_ERROR);
			}
		}
		modbus_status = monitor_modbus();
		if(modbus_status != HAL_OK && modbus_status != HAL_BUSY)
		{
			switch(modbus_status)
			{
				case MB_TX_TIMEOUT:
				{
					for(uint8_t i = 0; i < holding_register_database[MB_TRANSMIT_RETRIES]; i++)
					{
						modbus_status = modbus_send(modbus_tx_len);
						if(modbus_status != HAL_OK)
						{
							holding_register_database[MB_ERRORS] |= 1U << ((modbus_status - 1) + MB_FATAL_ERROR);
						}
					}
				  break;
				}
				case MB_RX_TIMEOUT:
				{
					// Error only relates to Modbus Master Nodes
					break;
				}
				case MB_UART_ERROR:
				{
					modbus_status = modbus_set_rx();
					if(modbus_status != 0)
					{
						holding_register_database[MB_ERRORS] |= 1U << ((modbus_status - 1) + MB_FATAL_ERROR);
					}
					break;
				}
				case MB_FATAL_ERROR:
				{
					while(modbus_status != HAL_OK)
					{
						modbus_status = modbus_reset();
					}
					break;
				}
				default:
				{
					// Unknown error
				}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_LOW;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMAMUX1_DMA1_CH4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMAMUX1_DMA1_CH4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMAMUX1_DMA1_CH4_5_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MCU_DCV_A_Pin|MCU_DCV_B_Pin|HPU_Gate_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Water_Solinoid_GPIO_Port, Water_Solinoid_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MCU_DCV_A_Pin MCU_DCV_B_Pin HPU_Gate_Pin */
  GPIO_InitStruct.Pin = MCU_DCV_A_Pin|MCU_DCV_B_Pin|HPU_Gate_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Oil_High_Pin */
  GPIO_InitStruct.Pin = Oil_High_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Oil_High_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Oil_Low_Pin Oil_E_Stop_Pin */
  GPIO_InitStruct.Pin = Oil_Low_Pin|Oil_E_Stop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : Water_Solinoid_Pin */
  GPIO_InitStruct.Pin = Water_Solinoid_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Water_Solinoid_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
