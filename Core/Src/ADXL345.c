/*
 * ADXL345.c
 *
 *  Created on: Jan 8, 2025
 *      Author: matthias
 */

#include "ADXL345.h"

extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;

extern float z_g;

void ADXL345_Init() {
    uint8_t data;

    // Set Power Control register (0x2D) to Measure Mode (bit D3 = 1)
    data = 0x08; // Measure mode
    HAL_I2C_Mem_Write(&hi2c1, ADXL345_I2C_ADDR, 0x2D, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

    // Set Data Format register (0x31) for full resolution and ±16g range
    data = 0x08; // Full resolution, ±16g
    HAL_I2C_Mem_Write(&hi2c1, ADXL345_I2C_ADDR, 0x31, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

    // Set Data Rate (0x2C) to 100 Hz
    data = 0x03; // 100 Hz
    HAL_I2C_Mem_Write(&hi2c1, ADXL345_I2C_ADDR, 0x2C, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
}

void ADXL345_WakeUp() {
    HAL_StatusTypeDef ret;
    uint8_t power_ctl;

    // Step 2: Clear the Sleep bit (Sleep = 0, Measure = 0)
    power_ctl = 0x00; // Ensure Sleep = 0
    ret = HAL_I2C_Mem_Write(&hi2c1, ADXL345_I2C_ADDR, 0x2D, I2C_MEMADD_SIZE_8BIT, &power_ctl, 1, 1000);
    if (ret != HAL_OK) {
        #if UART_DEBUG
            HAL_UART_Transmit(&huart2, (uint8_t *)"Error: Failed to clear Sleep bit\r\n", 34, HAL_MAX_DELAY);
        #endif
        return;
    }

    HAL_Delay(10); // Allow time for the device to stabilize

    // Step 3: Switch back to measurement mode (Measure = 1)
    power_ctl = 0x08; // Measure = 1, Sleep = 0
    ret = HAL_I2C_Mem_Write(&hi2c1, ADXL345_I2C_ADDR, 0x2D, I2C_MEMADD_SIZE_8BIT, &power_ctl, 1, 1000);
    if (ret != HAL_OK) {
        #if UART_DEBUG
        HAL_UART_Transmit(&huart2, (uint8_t *)"Error: Failed to enter measurement mode\r\n", 42, HAL_MAX_DELAY);
        #endif
    } else {
        #if UART_DEBUG
        HAL_UART_Transmit(&huart2, (uint8_t *)"ADXL345 is now in measurement mode\r\n", 37, HAL_MAX_DELAY);
        #endif
    }
}

void ADXL345_Measure() {
    HAL_StatusTypeDef ret;
    uint8_t data_tx[1] = {0x32}; // Starting register for data (DATAX0)
    uint8_t data_rx[2];          // Buffer for receiving Z-axis data (2 bytes)
    int16_t z;
    float scale_factor = 0.004;  // 4 mg/LSB = 0.004 g/LSB

    // Check ADXL345 Device ID (optional for debugging)
    uint8_t device_id;
    ret = HAL_I2C_Mem_Read(&hi2c1, ADXL345_I2C_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, &device_id, 1, 1000);
    if (ret == HAL_OK && device_id == 0xE5) {
        HAL_UART_Transmit(&huart2, (uint8_t *)"ADXL345 detected!\r\n", 20, HAL_MAX_DELAY);
    } else {
        HAL_UART_Transmit(&huart2, (uint8_t *)"ADXL345 not detected!\r\n", 24, HAL_MAX_DELAY);
        return;
    }

    // Request Z-axis data (DATAX2, DATAX3 for Z-axis)
    ret = HAL_I2C_Master_Transmit(&hi2c1, ADXL345_I2C_ADDR, &data_tx[0], 1, 1000);
    if (ret != HAL_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t *)"Error Tx Z-axis\r\n", 18, HAL_MAX_DELAY);
        return;
    }

    HAL_Delay(10);

    // Read Z-axis data (2 bytes)
    ret = HAL_I2C_Master_Receive(&hi2c1, ADXL345_I2C_ADDR, data_rx, 2, 1000);
    if (ret != HAL_OK) {
        HAL_UART_Transmit(&huart2, (uint8_t *)"Error Rx Z-axis\r\n", 18, HAL_MAX_DELAY);
        return;
    }

    // Combine the MSB and LSB for Z-axis
    z = (int16_t)((data_rx[1] << 8) | data_rx[0]);
    z_g = z * scale_factor;

    // Transmit Z-axis data
	#if UART_DEBUG
        char message[64];
        snprintf(message, sizeof(message), "Z: %.2fg\r\n", z_g);
        HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
	#endif
}

void ADXL345_Sleep() {
    HAL_StatusTypeDef ret;
    uint8_t power_ctl;
    power_ctl = 0x05;
    ret = HAL_I2C_Mem_Write(&hi2c1, ADXL345_I2C_ADDR, 0x2D, I2C_MEMADD_SIZE_8BIT, &power_ctl, 1, 1000);

    if (ret != HAL_OK) {
        #if UART_DEBUG
        HAL_UART_Transmit(&huart2, (uint8_t *)"Error: Failed to set sleep mode\r\n", 33, HAL_MAX_DELAY);
        #endif
    } else {
        #if UART_DEBUG
        HAL_UART_Transmit(&huart2, (uint8_t *)"ADXL345 is now in sleep mode\r\n", 31, HAL_MAX_DELAY);
        #endif
    }
}



