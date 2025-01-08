/*
 * stm_power.c
 *
 *  Created on: Jan 8, 2025
 *      Author: matthias
 */

#include "stm_power.h"

extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;

extern uint32_t master_timer;
extern uint32_t slave_timer;
extern uint32_t wakeStartTime;

void enterStandbyMode() {
    uint32_t totalWakeTime = HAL_GetTick() - wakeStartTime;
	#if UART_DEBUG
        printf("Total wake time: %lu ms\r\n", totalWakeTime);
	#endif
    HAL_I2C_DeInit(&hi2c1);

    // Total delay in milliseconds
    uint32_t total_delay_ms = master_timer + slave_timer;

    // Convert milliseconds to RTC ticks
    // 2.048 RTC ticks per millisecond (30720/15000)
    uint32_t delay_in_rtc_ticks = (total_delay_ms * 2048) / 1000;

    // Calculate adjusted wake-up value
    uint32_t rtc_wakeup_value;
    if (delay_in_rtc_ticks < 30720) {
        rtc_wakeup_value = 30720 - delay_in_rtc_ticks;
    } else {
        rtc_wakeup_value = 1;  // Minimum value
    }

	#if UART_DEBUG
    	printf("Total delay: %lu ms, RTC ticks: %lu, Wakeup value: %lu\r\n",
           total_delay_ms, delay_in_rtc_ticks, rtc_wakeup_value);
	#endif

    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, rtc_wakeup_value, RTC_WAKEUPCLOCK_RTCCLK_DIV16, 0) != HAL_OK) {
        Error_Handler();
    }

    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF3);

    HAL_PWR_EnterSTANDBYMode();
}

void enterStandbyModeMaster() {
    // Properly disable I2C before going to standby
    HAL_I2C_DeInit(&hi2c1);

    // Set RTC wake-up timer
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 30720, RTC_WAKEUPCLOCK_RTCCLK_DIV16, 0) != HAL_OK) {
        Error_Handler();
    }

    /* Clear Wakeup and Standby Flags */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF3);
	#if UART_DEBUG
    	printf("Going to sleep\r\n");
	#endif
    // Enter Standby Mode
    HAL_PWR_EnterSTANDBYMode();
}

void enterStandbyMode10s(void) {
    // Signal BLE to sleep via falling edge on D3
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_Delay(10);  // Give BLE time to process and enter sleep

    // Properly disable I2C before going to standby
    HAL_I2C_DeInit(&hi2c1);

    // Set RTC wake-up timer to 10 seconds
    uint32_t ticks_10s = (10 * 1000 * 2048) / 1000;

    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, ticks_10s, RTC_WAKEUPCLOCK_RTCCLK_DIV16, 0) != HAL_OK) {
        Error_Handler();
    }

    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF3);

	#if UART_DEBUG
    	printf("No ACK received in 10 seconds, entering standby mode\r\n");
	#endif

    // Enter Standby Mode
    HAL_PWR_EnterSTANDBYMode();
}


