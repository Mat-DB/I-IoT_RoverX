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
#include "LTR_329.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SHT40_ADDRESS (0x44 << 1)
#define BLE_ADDRESS (0x55 << 1)
#define UART_DEBUG 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void SHT40_measure(void);
static void LTR_329_setup(void);
static void LTR_329_measure(void);
static void BLE_get(void);

static void drive_rover(void);
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  //LTR_329_setup();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int delay = 5000;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	//SHT40_measure();
	//LTR_329_measure();
	//BLE_get();
	drive_rover();
	HAL_Delay(delay);
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
	HAL_Delay(delay);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  hi2c1.Init.Timing = 0x00707CBB;
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
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5_unused_Pin P6A_unused_Pin */
  GPIO_InitStruct.Pin = PA5_unused_Pin|P6A_unused_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// SHT40 is the themp and humidity sensor
static void SHT40_measure(){
	HAL_StatusTypeDef ret;
	uint8_t data_tx[1] = {0xFD};
	uint8_t data_rx[6];
    #if UART_DEBUG
		int length = 32;
		uint8_t message[32]={'\0'};
    #endif

	ret = HAL_I2C_Master_Transmit(&hi2c1, SHT40_ADDRESS, data_tx, 1, 1000);
	HAL_Delay(10);
	if ( ret != HAL_OK ) {
		#if UART_DEBUG
			snprintf((char*) message, length, "Error Tx SHT40\r\n");
			HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
	    #endif
	}
	else{
		//read bytes
		HAL_Delay(10);
		ret =  HAL_I2C_Master_Receive(&hi2c1, SHT40_ADDRESS, (uint8_t*)&data_rx, 6,1000);
		if ( ret != HAL_OK ) {
			#if UART_DEBUG
				snprintf((char*) message, length, "Error Rx SHT40\r\n");
				HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
			#endif
		}
		else{
			float t_ticks = data_rx[0] * 256 + data_rx[1];
			float rh_ticks = data_rx[3] * 256 + data_rx[4];

			float t_degC = -45 + 175 * t_ticks/65535;
			float rh_pRH = -6 + 125 * rh_ticks/65535;

			#if UART_DEBUG
				for(int i = 0; i < 6 ; i++){
					snprintf((char*) message, length, "data_rx[%i] = %u \r\n",i,data_rx[i]);
					HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
				}
				snprintf((char*) message, length, "t_degC = %.2f \r\n",t_degC);
				HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);

				snprintf((char*) message, length, "rh_pRH = %.2f \r\n",rh_pRH);
				HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
			#endif
		}
		#if UART_DEBUG
			snprintf((char*) message, length, "--------\r\n");
			HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
		#endif
	}
}

//LTR_329 setup
static void LTR_329_setup(){
	HAL_StatusTypeDef ret;
	ltr329_gain_t gain = LTR3XX_GAIN_2;
	ltr329_integrationtime_t intTime = LTR3XX_INTEGTIME_100;
	ltr329_measurerate_t measureRate = LTR3XX_MEASRATE_200;

	#if UART_DEBUG
		int length = 48;
		uint8_t message[48]={'\0'};
	#endif

	// 1. Create ALS_CONTR Register value.
	// 000<gain(3)><Reset(1)><Mode(1)>
	uint8_t contr_value = (0b000 << 5) | (gain << 2) | (0 << 1) | (1);

	#if UART_DEBUG
		snprintf((char*) message, length, "contr_value: 0x%X, gain%d\r\n", contr_value, gain);
		HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
	#endif

	// 2. Set the ALS_CONTR Register
	ret = HAL_I2C_Mem_Write(&hi2c1, LTR329_I2CADDR, LTR329_ALS_CTRL, 1, &contr_value, 1, 1000);

	if ( ret != HAL_OK ) {
		#if UART_DEBUG
		  snprintf((char*) message, length, "Error Tx - set ALS_CONTR reg\r\n");
		  HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
		#endif
	}
	// 3. Create ALS_MEAS_RATE register value
	// 00<integration time(3)><measurement rate(3)>
	uint8_t contr2_value = (0b00 << 6) | (intTime << 3) | (measureRate);
	if (UART_DEBUG) {
		snprintf((char*) message, length, "contr2_value: 0x%X\r\n", contr2_value);
		HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
	}

	// 4. Set ALS_MEAS_RATE Register
	ret = HAL_I2C_Mem_Write(&hi2c1, LTR329_I2CADDR, LTR329_MEAS_RATE, 1, &contr2_value, 1, 1000);
	HAL_Delay(10);
	if ( ret != HAL_OK ) {
		#if UART_DEBUG
			snprintf((char*) message, length, "Error Tx - set ALS_MEAS_RATE reg\r\n");
			HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
		#endif
	}
	else {
		#if UART_DEBUG
			snprintf((char*) message, length, "Succes setting up LTR_329!\r\n");
			HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
		#endif
	}
	#if UART_DEBUG
		snprintf((char*) message, length, "--------\r\n");
		HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
	#endif
}

// LTR_329 is the light sensor
static void LTR_329_measure(){
	HAL_StatusTypeDef ret;
	uint8_t data_rx[4];

	#if UART_DEBUG
		int length = 48;
		uint8_t message[48]={'\0'};
	#endif

	// 5. Get data, 4 register values, 2 times 16 bits
	ret = HAL_I2C_Mem_Read(&hi2c1, LTR329_I2CADDR, LTR329_CH1DATA, 1, (uint8_t*)&data_rx, 4, 1000);
	HAL_Delay(5);
	if ( ret != HAL_OK ) {
		#if UART_DEBUG
			snprintf((char*) message, length, "Error Rx data LTR-329\r\n");
			HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
		#endif
	}
	else{
		#if UART_DEBUG
			for(int i = 0; i < 4 ; i++){
				snprintf((char*) message, length, "data_rx[%i] = %u \r\n",i,data_rx[i]);
			 HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
		 }
		#endif
	}
	#if UART_DEBUG
		snprintf((char*) message, length, "--------\r\n");
		HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
	#endif
}

// BLE module
static void BLE_get(){
	HAL_StatusTypeDef ret;
	uint8_t data_tx[1] = {0xFD};
	uint8_t data_rx;
    #if UART_DEBUG
		int length = 32;
		uint8_t message[32]={'\0'};
    #endif

	ret = HAL_I2C_Master_Transmit(&hi2c1, BLE_ADDRESS, data_tx, 1, 1000);
	HAL_Delay(10);
	if ( ret != HAL_OK ) {
		#if UART_DEBUG
			snprintf((char*) message, length, "Error Tx BLE\r\n");
			HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
	    #endif
	}
	else{
		//read bytes
		HAL_Delay(10);
		ret =  HAL_I2C_Master_Receive(&hi2c1, BLE_ADDRESS, (uint8_t*)&data_rx, 1,1000);
		if ( ret != HAL_OK ) {
			#if UART_DEBUG
				snprintf((char*) message, length, "Error Rx BLE\r\n");
				HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
			#endif
		}
		else{
			#if UART_DEBUG
				snprintf((char*) message, length, "data_rx: 0x%X\r\n", data_rx);
				HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);

				//snprintf((char*) message, length, "rh_pRH = %.2f \r\n",rh_pRH);
				//HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
			#endif
		}
	}
	#if UART_DEBUG
		snprintf((char*) message, length, "--------\r\n");
		HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
	#endif
}

static void drive_rover(){
	uint8_t command[16];  // Join command for LoRaWAN E5
	snprintf((char*) command, 16, "drvstr 200 70\n");
	HAL_UART_Transmit(&huart1, command, strlen((char*) command), 100);


	//uint8_t message[16];
	uint8_t temp[32];
	//HAL_UART_Receive(&huart1, message, 7, 10);
	snprintf((char*) temp, 32, "Driving started\r\n");
	HAL_UART_Transmit(&huart2, temp, strlen((char*) temp), 10);
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
