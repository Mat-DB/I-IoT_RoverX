/*
 * BLE.c
 *
 *  Created on: Jan 8, 2025
 *      Author: matthias
 */

#include "BLE.h"
//#include <ctype.h>

extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;

extern uint32_t master_timer;
extern uint32_t slave_timer;
extern uint32_t wakeStartTime;

extern char received_data_BLE[lora_data_length];

bool slave_send_data_to_BLE(const char* data_string) {
    HAL_StatusTypeDef status;

    // Transmit the string over I2C
    status = HAL_I2C_Master_Transmit(&hi2c1, BLE_I2C_ADDRESS << 1, (uint8_t *)data_string, strlen(data_string), 500);

    // Check transmission status
    if (status == HAL_OK) {
        #if UART_DEBUG
        	printf("Data sent to BLE: %s\r\n", data_string);
        #endif
        return true;
    } else {
        #if UART_DEBUG
        	printf("Error in transmission: 0x%02X\r\n", status);
        #endif
//        // Attempt I2C Recovery
//        HAL_I2C_DeInit(&hi2c1);
//        HAL_Delay(1);
//        MX_I2C1_Init();
        return false;
    }
}

bool mastertest_receive_data_from_BLE(void) {
    HAL_StatusTypeDef status;
    uint8_t buffer[64];

    // First, make sure I2C bus is clear
    reset_i2c();
    HAL_Delay(10);  // Give BLE time to setup after wake

    // Try to receive data
    for(int retry = 0; retry < 3; retry++) {  // Add retries
        status = HAL_I2C_Master_Receive(&hi2c1, BLE_I2C_ADDRESS << 1, buffer, sizeof(buffer)-1, 100);

        if (status == HAL_OK) {
            buffer[sizeof(buffer)-1] = '\0';
            printf("Received I2C data: %s\r\n", buffer);
            return true;
        }
        HAL_Delay(10);  // Wait before retry
    }

    return false;
}


//bool slave_receive_data_from_BLE(void) {
//   HAL_StatusTypeDef status;
//   uint8_t received_data[64] = {0};
//
//   status = HAL_I2C_Master_Receive(&hi2c1, BLE_I2C_ADDRESS << 1, received_data, sizeof(received_data) - 1, 2000);
//   if (status == HAL_OK) {
//       received_data[sizeof(received_data) - 1] = '\0';
//
//       if (isspace(received_data[0])) { // Check if first character is a space
//           #if UART_DEBUG
//           printf("Received valid data from BLE: %s\r\n", received_data);
//           #endif
//           master_send_data_to_BLE();
//           return true;
//       }
//       #if UART_DEBUG
//       printf("Invalid data format (no leading space)\r\n");
//       #endif
//       return false;
//   }
//   return false;
//}


bool master_receive_data_from_BLE(void) {
    HAL_StatusTypeDef status;
    uint8_t buffer[64];

    // First, make sure I2C bus is clear
    reset_i2c();
    HAL_Delay(10); // Give BLE time to setup after wake

    // Try to receive data with retries
    for(int retry = 0; retry < 3; retry++) {
        status = HAL_I2C_Master_Receive(&hi2c1, BLE_I2C_ADDRESS << 1, buffer, sizeof(buffer)-1, 100);

        if (status == HAL_OK) {
            buffer[sizeof(buffer)-1] = '\0';
            #if UART_DEBUG
            printf("Received from BLE: %s\r\n", buffer);
            #endif

            char dataOnly[lora_data_length];
            uint32_t time1, time2;
            // Parse data with format "data_timer1timer2"
            if (sscanf((char*)buffer, "%[^_]_%lu_%lu", dataOnly, &time1, &time2) == 3) {
            	master_timer = time1;
                slave_timer = time2;
                #if UART_DEBUG
                printf("Data: %s\nTime1: %lu ms, Time2: %lu ms\n",
                       dataOnly, master_timer, slave_timer);
                #endif
                snprintf(received_data_BLE, lora_data_length, "%s", dataOnly);
                return true;
            } else {
                #if UART_DEBUG
                	printf("Failed to parse data format\r\n");
                	printf("TEST: data: %s\r\n", dataOnly);
                #endif
            }
        }
        #if UART_DEBUG
        printf("Error receiving from BLE (attempt %d): %d\r\n", retry + 1, status);
        #endif
        HAL_Delay(10); // Wait before retry
    }

    return false;
}


bool master_send_data_to_BLE() {
    HAL_StatusTypeDef status;
    // Define the "ACK" message
    const char *ack_message = "ACK";

    // Transmit the "ACK" message over I2C
    status = HAL_I2C_Master_Transmit(&hi2c1, BLE_I2C_ADDRESS << 1, (uint8_t *)ack_message, strlen(ack_message), 500);

    // Check transmission status
    if (status == HAL_OK) {
        #if UART_DEBUG
            printf("ACK sent to BLE: %s\r\n", ack_message);
        #endif
            return 1;
    } else {
        #if UART_DEBUG
            printf("Error in transmission while sending ACK\r\n");
        #endif
        return 0;
    }
}


