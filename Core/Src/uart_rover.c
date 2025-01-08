/*
 * uart_rover.c
 *
 *  Created on: Dec 18, 2024
 *      Author: matthias
 */

#include "uart_rover.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;


int Rover_SendCommand_1000(char *command, char *expected) {
	return Rover_SendCommand(command, expected, 1000);
}


int Rover_SendCommand(char *command, char *expected, uint32_t timeout) {
	int retVal=0;
	#if UART_DEBUG
		const uint8_t temp_length=120;
		char temp[temp_length];
	#endif

	char r_buf[100]={0};
    HAL_UART_Transmit(&huart1, (uint8_t*) command, strlen(command), 100);

    HAL_UART_Receive(&huart1, (uint8_t*) r_buf, sizeof(r_buf), timeout);

	if (strstr(r_buf, expected) != NULL) {
		#if UART_DEBUG
        	snprintf((char*) temp, temp_length, "command: %s success!\r\n", command);
        	HAL_Delay(UART_DELAY);
        	HAL_UART_Transmit(&huart2, (uint8_t*) temp, strlen(temp), 100);
		#endif
        retVal=1;
	} else {
		#if UART_DEBUG
			snprintf(temp, temp_length, "\r\nOutput:\r\n");
			HAL_Delay(UART_DELAY);
			HAL_UART_Transmit(&huart2, (uint8_t*) temp, strlen(temp), 100);
			HAL_Delay(UART_DELAY);
			HAL_UART_Transmit(&huart2, (uint8_t*) r_buf, strlen(r_buf), 100);
		#endif
	}
    return retVal;
}

void drive_rover(){
	uint8_t command[16];  // Join command for LoRaWAN E5
	snprintf((char*) command, 16, "drvstr 200 70\n");
	HAL_UART_Transmit(&huart1, command, strlen((char*) command), 100);


	//uint8_t message[16];
	uint8_t temp[32];
	//HAL_UART_Receive(&huart1, message, 7, 10);
	snprintf((char*) temp, 32, "Driving started\r\n");
	HAL_UART_Transmit(&huart2, temp, strlen((char*) temp), 10);
}
