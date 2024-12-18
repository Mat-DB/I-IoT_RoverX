/*
 * uart_lora.c
 *
 *  Created on: Dec 17, 2024
 *      Author: matthias
 */

#include "uart_lora.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

int LoRa_SendCommand_1000(const char *command, const char *expected) {
	return LoRa_SendCommand(command, expected, 1000);
}


int LoRa_SendCommand(const char *command, const char *expected, uint32_t timeout) {
	int retVal=0;
	#if UART_DEBUG
		const uint8_t temp_length=120;
		char temp[temp_length];
	#endif
    HAL_UART_Transmit(&huart1, (uint8_t*) command, strlen(command), 100);

    char r_buf[100]={0};
    HAL_UART_Receive(&huart1, (uint8_t*) r_buf, sizeof(r_buf), timeout);

	if (strstr(r_buf, expected) != NULL) {
		#if UART_DEBUG
        	snprintf((char*) temp, temp_length, "command: %s success!\r\n", command);
        	HAL_Delay(UART_DELAY);
        	HAL_UART_Transmit(&huart2, (uint8_t*) temp, strlen(temp), 100);
		#endif
        retVal=1;
	}
    return retVal;
}

