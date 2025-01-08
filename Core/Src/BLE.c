/*
 * BLE.c
 *
 *  Created on: Jan 8, 2025
 *      Author: matthias
 */

#include "BLE.h"

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
        // Attempt I2C Recovery
        HAL_I2C_DeInit(&hi2c1);
        HAL_Delay(1);
        MX_I2C1_Init();
        return false;
    }
}

bool slave_receive_data_from_BLE(void) {
    HAL_StatusTypeDef status;
    uint8_t buffer[64]; // Increased buffer size for timer values

    status = HAL_I2C_Master_Receive(&hi2c1, BLE_I2C_ADDRESS << 1, buffer, sizeof(buffer)-1, 100);

    if (status == HAL_OK) {
        buffer[sizeof(buffer)-1] = '\0';
        printf("Received from BLE: %s\r\n", buffer);

        // Parse ACK and timer values
        if (strncmp((char*)buffer, "ACK_", 4) == 0) {
            uint32_t m_timer, s_timer;
            if (sscanf((char*)buffer, "ACK_%lu_%lu", &m_timer, &s_timer) == 2) {
                master_timer = m_timer;
                slave_timer = s_timer;
                printf("Parsed timers - Master: %lu, Slave: %lu\r\n", master_timer, slave_timer);
                return true;
            }
        }
    } else {
        printf("Error receiving from BLE: %d\r\n", status);
        HAL_I2C_DeInit(&hi2c1);
        HAL_Delay(1);
        MX_I2C1_Init();
    }

    return false;
}


bool master_receive_data_from_BLE(void) {
    HAL_StatusTypeDef status;
    uint8_t received_data[64] = {0};

    status = HAL_I2C_Master_Receive(&hi2c1, BLE_I2C_ADDRESS << 1, received_data, sizeof(received_data) - 1, 500);

    if (status == HAL_OK) {
        received_data[sizeof(received_data) - 1] = '\0';

        // Validate that we received actual sensor data (check for "T=" and "H=")
        if (strstr((char*)received_data, "T=") && strstr((char*)received_data, "H=")) {
            printf("Received valid sensor data from BLE: %s\r\n", received_data);
            send_data_to_BLE_master();  // Only send ACK for valid data
            return true;
        }
        return false;
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
        // Attempt I2C Recovery
        HAL_I2C_DeInit(&hi2c1);
        HAL_Delay(1);
        MX_I2C1_Init();
        return 0;
    }
}





