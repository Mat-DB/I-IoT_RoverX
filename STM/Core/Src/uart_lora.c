/*
 * uart_lora.c
 *
 *  Created on: Dec 17, 2024
 *      Author: matthias
 */

#include "uart_lora.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;


int LoRa_SendCommand_1000(char *command, char *expected) {
	return LoRa_SendCommand(command, expected, 1000);
}


int LoRa_SendCommand(char *command, char *expected, uint32_t timeout) {
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


void LoRaWAN_Startup(void) {
	#if UART_DEBUG
		const uint8_t temp_length=120;
		uint8_t temp[temp_length];
	#endif
	const uint8_t length=64;
	char command[length];
	char expect_rx[length];
	int prev_succes=1;

	// AT test command
	if (prev_succes) {
		prev_succes = 0;
		snprintf(command, length, "AT\r\n");
		snprintf(expect_rx, length, "+AT: OK");
		if (LoRa_SendCommand_1000(command, expect_rx)) {
			prev_succes = 1;
			#if UART_DEBUG
				HAL_Delay(UART_DELAY);
				snprintf((char*) temp, temp_length, "AT successful!\r\n");
				HAL_UART_Transmit(&huart2, temp, strlen((char*) temp), 100);
			#endif
		} else {
			#if UART_DEBUG
				HAL_Delay(UART_DELAY);
				snprintf((char*) temp, temp_length, "AT failed!\r\n");
				HAL_UART_Transmit(&huart2, temp, strlen((char*) temp), 100);
			#endif
		}
	}

	// Set DevEUI
	if (prev_succes) {
		prev_succes = 0;
		snprintf(command, length, "AT+ID=DevEui\"70B3D57ED006C326\"\r\n");
		snprintf(expect_rx, length, "+ID: DevEui,");
		if (LoRa_SendCommand_1000(command, expect_rx)) {
			prev_succes = 1;
			#if UART_DEBUG
				HAL_Delay(UART_DELAY);
				snprintf((char*) temp, temp_length, "DevEUI successful!\r\n");
				HAL_UART_Transmit(&huart2, temp, strlen((char*) temp), 100);
			#endif
		} else {
			#if UART_DEBUG
				HAL_Delay(UART_DELAY);
				snprintf((char*) temp, temp_length, "DevEUI failed!\r\n");
				HAL_UART_Transmit(&huart2, temp, strlen((char*) temp), 100);
			#endif
		}
	}

	// Set AppEUI
	if (prev_succes) {
		prev_succes = 0;
		snprintf(command, length, "AT+ID=AppEui,\"BB0B3CCAC02A0000\"\r\n");
		snprintf(expect_rx, length, "+ID: AppEui,");
		if (LoRa_SendCommand_1000(command, expect_rx)) {
			prev_succes = 1;
			#if UART_DEBUG
				HAL_Delay(UART_DELAY);
				snprintf((char*) temp, temp_length, "AppEUI successful!\r\n");
				HAL_UART_Transmit(&huart2, temp, strlen((char*) temp), 100);
			#endif
		} else {
			#if UART_DEBUG
				HAL_Delay(UART_DELAY);
				snprintf((char*) temp, temp_length, "AppEUI failed!\r\n");
				HAL_UART_Transmit(&huart2, temp, strlen((char*) temp), 100);
			#endif
		}
	}

	// Set AppKey
	if (prev_succes) {
		prev_succes = 0;
		snprintf(command, length, "AT+KEY=APPKEY,\"6A5B74B4864C3EE02C1176C8B5C670BB\"\r\n");
		snprintf(expect_rx, length, "+KEY: APPKEY");
		if (LoRa_SendCommand_1000(command, expect_rx)) {
			prev_succes = 1;
			#if UART_DEBUG
				HAL_Delay(UART_DELAY);
				snprintf((char*) temp, temp_length, "AppKey successful!\r\n");
				HAL_UART_Transmit(&huart2, temp, strlen((char*) temp), 100);
			#endif
		} else {
			#if UART_DEBUG
				HAL_Delay(UART_DELAY);
				snprintf((char*) temp, temp_length, "AppKey failed!\r\n");
				HAL_UART_Transmit(&huart2, temp, strlen((char*) temp), 100);
			#endif
		}
	}

	// Set Data Rate, DR
	if (prev_succes) {
		prev_succes = 0;
		snprintf(command, length, "AT+DR=EU868\r\n");
		snprintf(expect_rx, length, "+DR: EU868");
		if (LoRa_SendCommand_1000(command, expect_rx)) {
			prev_succes = 1;
			#if UART_DEBUG
				HAL_Delay(UART_DELAY);
				snprintf((char*) temp, temp_length, "DR=EU868 successful!\r\n");
				HAL_UART_Transmit(&huart2, temp, strlen((char*) temp), 100);
			#endif
		} else {
			#if UART_DEBUG
				HAL_Delay(UART_DELAY);
				snprintf((char*) temp, temp_length, "DR=EU868 failed!\r\n");
				HAL_UART_Transmit(&huart2, temp, strlen((char*) temp), 100);
			#endif
		}
	}

	// Set CHannel frequency, CH
	if (prev_succes) {
		prev_succes = 0;
		snprintf(command, length, "AT+CH=NUM,0-2\r\n");
		snprintf(expect_rx, length, "+CH: NUM");
		if (LoRa_SendCommand_1000(command, expect_rx)) {
			prev_succes = 1;
			#if UART_DEBUG
				HAL_Delay(UART_DELAY);
				snprintf((char*) temp, temp_length, "Channel successful!\r\n");
				HAL_UART_Transmit(&huart2, temp, strlen((char*) temp), 100);
			#endif
		} else {
			#if UART_DEBUG
				HAL_Delay(UART_DELAY);
				snprintf((char*) temp, temp_length, "Channel failed!\r\n");
				HAL_UART_Transmit(&huart2, temp, strlen((char*) temp), 100);
			#endif
		}
	}

	// Set mode to LWOTAA
	if (prev_succes) {
		prev_succes = 0;
		snprintf(command, length, "AT+MODE=LWOTAA\r\n");
		snprintf(expect_rx, length, "+MODE: LWOTAA");

		if (LoRa_SendCommand_1000(command, expect_rx)) {
			prev_succes = 1;
			#if UART_DEBUG
				HAL_Delay(UART_DELAY);
				snprintf((char*) temp, temp_length, "MODE successful!\r\n");
				HAL_UART_Transmit(&huart2, temp, strlen((char*) temp), 100);
			#endif
		} else {
			#if UART_DEBUG
				HAL_Delay(UART_DELAY);
				snprintf((char*) temp, temp_length, "MODE failed!\r\n");
				HAL_UART_Transmit(&huart2, temp, strlen((char*) temp), 100);
			#endif
		}
	}

	// Join the network
	if (prev_succes) {
		prev_succes = 0;
		snprintf(command, length, "AT+JOIN\r\n");
		snprintf(expect_rx, length, "Network joined");

		if (LoRa_SendCommand(command, expect_rx, 20000)) {
			prev_succes = 1;
			#if UART_DEBUG
				HAL_Delay(UART_DELAY);
				snprintf((char*) temp, temp_length, "JOIN successful!\r\n");
				HAL_UART_Transmit(&huart2, temp, strlen((char*) temp), 100);
			#endif
		} else {
			#if UART_DEBUG
				HAL_Delay(UART_DELAY);
				snprintf((char*) temp, temp_length, "JOIN failed!\r\n");
				HAL_UART_Transmit(&huart2, temp, strlen((char*) temp), 100);
			#endif
		}
	}
}


void LoRaWAN_Sleep(void){
	LoRa_SendCommand_1000("AT+LOWPOWER\r\n", "+LOWPOWER: SLEEP");
}


void LoRaWAN_Send_msg(char *data, int hex_1){
	#if UART_DEBUG
		const uint8_t temp_length=120;
		uint8_t temp[temp_length];
	#endif
	const uint8_t length=64;
	char command[length];
	char expect_rx[length];

	if (hex_1) {
		snprintf(command, length, "AT+MSGHEX=\"%02X\"\r\n", (unsigned int) data);
		sniprintf(expect_rx, length, "+MSG: Done");
		if (LoRa_SendCommand(command, expect_rx, 20000)) {
			#if UART_DEBUG
				HAL_Delay(UART_DELAY);
				snprintf((char*) temp, temp_length, "Send data successful!\r\n");
				HAL_UART_Transmit(&huart2, temp, strlen((char*) temp), 100);
			#endif
		} else {
			#if UART_DEBUG
				HAL_Delay(UART_DELAY);
				snprintf((char*) temp, temp_length, "Data send failed!\r\n");
				HAL_UART_Transmit(&huart2, temp, strlen((char*) temp), 100);
			#endif
		}
	} else {
		snprintf(command, length, "AT+MSG=\"%s\"\r\n", data);
		sniprintf(expect_rx, length, "+MSG: Done");
		if (LoRa_SendCommand(command, expect_rx, 20000)) {
			#if UART_DEBUG
				HAL_Delay(UART_DELAY);
				snprintf((char*) temp, temp_length, "Send data successful!\r\n");
				HAL_UART_Transmit(&huart2, temp, strlen((char*) temp), 100);
			#endif
		} else {
			#if UART_DEBUG
				HAL_Delay(UART_DELAY);
				snprintf((char*) temp, temp_length, "Data send failed!\r\n");
				HAL_UART_Transmit(&huart2, temp, strlen((char*) temp), 100);
			#endif
		}
	}

}

