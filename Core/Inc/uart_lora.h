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

// Function Prototype
int LoRa_SendCommand_1000(const char *command, const char *expected);
int LoRa_SendCommand(const char *command, const char *expected, uint32_t timeout);

#endif /* SRC_UART_LORA_H_ */
