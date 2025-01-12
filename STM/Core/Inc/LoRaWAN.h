/*
 * uart_lora.h
 *
 *  Created on: Dec 17, 2024
 *      Author: matthias
 */

#ifndef SRC_UART_LORA_H_
#define SRC_UART_LORA_H_


#include <string.h>
#include <stdio.h>
#include "main.h"

// Function definitions
int LoRa_SendCommand_1000(char *command, char *expected);
int LoRa_SendCommand(char *command, char *expected, uint32_t timeout);
void LoRaWAN_Startup(char send_EUIs);
void LoRaWAN_Sleep(void);
void LoRaWAN_WakeUp(void);
void LoRaWAN_Send_msg(char *data, uint8_t hex_1);

#endif /* SRC_UART_LORA_H_ */
