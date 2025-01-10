/*
 * BLE.h
 *
 *  Created on: Jan 8, 2025
 *      Author: matthias
 */

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "main.h"

#ifndef INC_BLE_H_
#define INC_BLE_H_

bool slave_send_data_to_BLE(const char* data_string);
bool slave_receive_data_from_BLE(void);
bool master_receive_data_from_BLE(void);
bool master_send_data_to_BLE(void);


#endif /* INC_BLE_H_ */
