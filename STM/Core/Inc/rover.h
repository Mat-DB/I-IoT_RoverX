/*
 * uart_rover.h
 *
 *  Created on: Dec 18, 2024
 *      Author: matthias
 */

#ifndef INC_ROVER_H_
#define INC_ROVER_H_

#include <string.h>
#include <stdio.h>
#include "main.h"

// Function definitions
int Rover_SendCommand_1000(char *command, char *expected);
int Rover_SendCommand(char *command, char *expected, uint32_t timeout);

void drive_rover(void);
void drive_sequence_master(void);

#endif /* INC_ROVER_H_ */
