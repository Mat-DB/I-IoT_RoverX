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
#include "uart_lora.h"
#include "uart_rover.h"
#include "BLE.h"
#include "stm_power.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Set to 0 for master and 1 for slave!
#define SLAVE 1
#define SHT40_ADDRESS (0x44 << 1)

#if SLAVE
	#define BLE_I2C_ADDRESS 0x08 // Slave rover
	uint32_t master_timer = 0;
	uint32_t slave_timer = 0;
	uint32_t wakeStartTime = 0;
#else
	#define BLE_I2C_ADDRESS 0x09 // Master rover
#endif


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;

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
static void MX_TIM1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
static void global_init_Done(void);
static void custom_init_Done(void);

static void SHT40_measure(void);
static void BLE_get(void);

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
  MX_TIM1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  // Wake BLE by pulling PB4 low then high
  HAL_GPIO_WritePin(BLE_wake_up_GPIO_Port, BLE_wake_up_Pin, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(BLE_wake_up_GPIO_Port, BLE_wake_up_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(BLE_wake_up_GPIO_Port, BLE_wake_up_Pin, GPIO_PIN_SET);
  HAL_Delay(1);

  // Check if we woke up from Standby
  if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET) {
      // Clear Standby and Wakeup Flags
      __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
      __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF3);
      HAL_I2C_DeInit(&hi2c1);
      HAL_Delay(10);
      MX_I2C1_Init();
      #if UART_DEBUG
          printf("Woken up from Standby mode!\r\n");
      #endif
  } else {
      #if UART_DEBUG
        printf("Normal startup\r\n");
      #endif
  }

  #if SLAVE
  	  LTR_329_setup();
  	  SHT40_measure();
  	  LTR_329_measure();

	  #if UART_DEBUG
  	  	  uint32_t i2cStartTime = HAL_GetTick();
	  #endif

  	  // Make string packet to send!!

  	  while(!send_data_to_BLE(data)) {
  		  HAL_Delay(10);
  	  }
	  #if UART_DEBUG
  	  	  uint32_t i2cEndTime = HAL_GetTick();
	  #endif
  	  HAL_TIM_Base_Start(&htim1);
  	  HAL_Delay(1); // Wait for UART to flush
	#else
	  int counter = 1;
	  while(!receive_data_from_BLE_master()) {
		  HAL_Delay(1);
	  }
	  while(!send_data_to_BLE_master()) {
		  HAL_Delay(1);
	  }
	  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0);
	  enterStandbyModeMaster();
	#endif

  custom_init_Done();

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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer(&hrtc, 30720, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15624;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|BLE_wake_up_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5_unused_Pin P6A_unused_Pin */
  GPIO_InitStruct.Pin = PA5_unused_Pin|P6A_unused_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin BLE_wake_up_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|BLE_wake_up_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Global init done
static void global_init_Done(){
	#if UART_DEBUG
		const uint8_t length=24;
		uint8_t message[length];
		snprintf((char*) message, length, "Global init done!\r\n");
		HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
	#endif
}

// Custom init done
static void custom_init_Done(){
	#if UART_DEBUG
		const uint8_t length=24;
		uint8_t message[length];
		snprintf((char*) message, length, "Custom init done!\r\n");
		HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
	#endif
}

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
	} else {
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
			uint16_t temp_int = (uint16_t) (t_degC*10);
			float rh_pRH = -6 + 125 * rh_ticks/65535;
			uint16_t rh_int = (uint16_t) (rh_pRH*10);

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
