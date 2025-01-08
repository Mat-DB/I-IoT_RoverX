/*
 * LTR_329.c
 *
 *  Created on: Dec 18, 2024
 *      Author: matthias
 */

#include "LTR_329.h"

//extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;

extern uint16_t ch0_both;
extern uint16_t ch1_IR;

//LTR_329 setup
void LTR_329_setup(){
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
void LTR_329_measure(){
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
		ch0_both = data_rx[1] * 256 + data_rx[0]; // Visible + IR light
		ch1_IR = data_rx[3] * 256 + data_rx[2]; // IR light only

		#if UART_DEBUG
			for(int i = 0; i < 4 ; i++){
				snprintf((char*) message, length, "data_rx[%i] = %u \r\n",i,data_rx[i]);
				HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
			}
			snprintf((char*) message, length, "CH0 Visible + IR: %d\r\n", ch0_both);
			HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
			snprintf((char*) message, length, "CH1 Infrared: %d\r\n", ch1_IR);
			HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
		#endif
	}
	#if UART_DEBUG
		snprintf((char*) message, length, "--------\r\n");
		HAL_UART_Transmit(&huart2, message, strlen((char*) message), 100);
	#endif
}
