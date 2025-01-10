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
#include "ADXL345.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
//#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SHT40_ADDRESS (0x44 << 1)

// Timer variables
uint32_t master_timer = 0;
uint32_t slave_timer = 0;
uint32_t wakeStartTime = 0;

// Data variables
float temp_degC;
float rh_pRH;
uint16_t ch0_both;
uint16_t ch1_IR;
float z_g;

#define format_data_length 50
char formatted_data[format_data_length];

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
void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void global_init_Done(void);
void custom_init_Done(void);
void SHT40_measure(void);

void format_data(uint8_t power_percentage, uint8_t sleep_mode, uint8_t data_flags, uint8_t rover_moved);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 2000);
  return len;
}
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

  /*
   *
   */

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

  custom_init_Done();

  #if SLAVE
  	  // Timer is to measure time awake -> Debug
	  #if UART_DEBUG
  	  	  HAL_TIM_Base_Start(&htim1);
	  #endif
  	  LTR_329_setup();
  	  ADXL345_WakeUp();
  	  ADXL345_Init();

      uint32_t startTime = HAL_GetTick();
      bool gotACK = false;

      // Send data to BLE
      SHT40_measure();
      LTR_329_measure();
      ADXL345_Measure();

      #if UART_DEBUG
  	  	  uint32_t i2cStartTime = HAL_GetTick();
	  #endif
  	  // 3 bits bat % - 1 bit to sleep - 3 bits select data - 1 bit moved
  	  // Select data: 1 bit temp&humidity - 1 bit light - 1 bit vibrations
  	  format_data(0x0, 0x1, 0b00001110, 0x0);
  	  while(!slave_send_data_to_BLE(formatted_data)) {
  		  HAL_Delay(5); // Set to 5 instead of 10
  	  }
	  #if UART_DEBUG
  	  	  uint32_t i2cEndTime = HAL_GetTick();
	  #endif

      // Wait for ACK with timeout
      while((HAL_GetTick() - startTime) < 10000) { // 10000ms = 10s
          if(slave_receive_data_from_BLE()) {
              gotACK = true;
              uint32_t i2cDuration = i2cEndTime - i2cStartTime;
			  #if UART_DEBUG
              	  printf("Tussentijd (I2C transmit): %lu ms\r\n", i2cDuration);
			  #endif
              // Go to standby a little lower in the code
          }
          HAL_Delay(10);  // Small delay to prevent tight polling
      }

      // If no ACK received within 10s
      if(!gotACK) {
          enterStandbyMode10s();  // Enter 10s standby and signal BLE
      }
      else {
    	  enterStandbyMode();  // Normal standby with calculated delay
      }
	#else
	  int counter = 1;
	  while(!master_receive_data_from_BLE()) {
		  HAL_Delay(10);
		  printf("Waiting...\r\n");
	  }
	  while(!master_send_data_to_BLE()) {
		  HAL_Delay(1);
	  }
	  enterStandbyModeMaster();
	#endif

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
void MX_I2C1_Init(void)
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
void global_init_Done(){
	#if UART_DEBUG
		const uint8_t length=24;
		uint8_t message[length];
		snprintf((char*) message, length, "Global init done!\r\n");
		HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
	#endif
}

// Custom init done
void custom_init_Done(){
	#if UART_DEBUG
		const uint8_t length=24;
		uint8_t message[length];
		snprintf((char*) message, length, "Custom init done!\r\n");
		HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
	#endif
}

// SHT40 is the themp and humidity sensor
void SHT40_measure(){
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

			temp_degC = -45 + 175 * t_ticks/65535;
			//uint16_t temp_int = (uint16_t) (temp_degC*10);
			rh_pRH = -6 + 125 * rh_ticks/65535;
			//uint16_t rh_int = (uint16_t) (rh_pRH*10);

			#if UART_DEBUG
				for(int i = 0; i < 6 ; i++){
					snprintf((char*) message, length, "data_rx[%i] = %u \r\n",i,data_rx[i]);
					HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
				}
				snprintf((char*) message, length, "t_degC = %.2f \r\n",temp_degC);
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

void format_data(uint8_t power_percentage, uint8_t sleep_mode, uint8_t data_flags, uint8_t rover_moved){
    uint8_t input = ((power_percentage & 0x07) << 5) | // Mask to 3 bits (bits 7-5)
                    ((sleep_mode & 0x01) << 4) |      // Mask to 1 bit (bit 4)
                    ((data_flags & 0x07) << 1) |     // Mask to 3 bits (bits 3-1)
                    (rover_moved & 0x01);            // Mask to 1 bit (bit 0)
    uint8_t message[32];  // Buffer for binary output message
    int pos = 0;          // Position tracker for the message buffer

    // Append the input byte directly to the message
    message[pos++] = input;

    // Append selected data based on flags in binary format, scaled by 100
    if (data_flags & 0x04) {
        short int temperature = (short int)((temp_degC) * 100);  // Placeholder temperature scaled by 100
        short int humidity = (short int)(rh_pRH * 100); // Placeholder humidity scaled by 100
        memcpy(&message[pos], &temperature, sizeof(temperature));
        pos += sizeof(temperature);
        memcpy(&message[pos], &humidity, sizeof(humidity));
        pos += sizeof(humidity);
    }

    if (data_flags & 0x02) {
        short int light_data = ch0_both;  // Placeholder light data
        short int IR_data = ch1_IR;
        memcpy(&message[pos], &light_data, sizeof(light_data));
        pos += sizeof(light_data);
        memcpy(&message[pos], &IR_data, sizeof(IR_data));
        pos += sizeof(IR_data);
    }

    if (data_flags & 0x01) {
        short int vibration = (short int)(z_g * 100);  // Placeholder vibration data scaled by 100
        memcpy(&message[pos], &vibration, sizeof(vibration));
        pos += sizeof(vibration);
    }

    snprintf(formatted_data, format_data_length, "");
    for (int i = 0; i < pos; i++) {
    	// Add a space after each byte except the last one
//    	if (i < pos - 1) {
//    		snprintf(formatted_data, format_data_length, "%s %2X ",formatted_data, message[i]);
//    	}
//    	else {
//    		snprintf(formatted_data, format_data_length, "%s %2X",formatted_data, message[i]);
//    	}
    	snprintf(formatted_data, format_data_length, "%s%2X",formatted_data, message[i]);
    }

    // Transmit the buffer
    #if UART_DEBUG
    	printf("FORMATTED Data:\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*) formatted_data, strlen(formatted_data), 100);
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
