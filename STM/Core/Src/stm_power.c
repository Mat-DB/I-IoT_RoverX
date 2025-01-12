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
extern uint32_t start_LoRa;

#define timer_value 61440

void enterStandbyModeMaster() {
	#if UART_DEBUG
    	uint32_t totalWakeTime = HAL_GetTick() - wakeStartTime;
        printf("Total wake time: %lu ms\r\n", totalWakeTime);
	#endif
    HAL_I2C_DeInit(&hi2c1);

    uint32_t LoRaTime = HAL_GetTick() - start_LoRa;
    // Total delay in milliseconds
    uint32_t total_delay_ms = master_timer + slave_timer + LoRaTime + 20; // offset van 20ms zodat master wakker wordt voor de slave

    // Convert milliseconds to RTC ticks
    // 2.048 RTC ticks per millisecond (30720/15000)
    uint32_t delay_in_rtc_ticks = (total_delay_ms * 2048) / 1000;

    // Calculate adjusted wake-up value
    uint32_t rtc_wakeup_value;
    if (delay_in_rtc_ticks < timer_value) {
        rtc_wakeup_value = timer_value - delay_in_rtc_ticks;
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

void enterStandbyMode() {
    // Properly disable I2C before going to standby
    HAL_I2C_DeInit(&hi2c1);

    // Set RTC wake-up timer
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, timer_value, RTC_WAKEUPCLOCK_RTCCLK_DIV16, 0) != HAL_OK) {
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

void enterStandbyModeShort(void) {
    // Properly disable I2C before going to standby
    HAL_I2C_DeInit(&hi2c1);

    // Set RTC wake-up timer to 1 seconds
    uint32_t ticks_10s = (1 * 1000 * 2048) / 1000;

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


void reset_i2c(void) {
    // Deinit I2C
    HAL_I2C_DeInit(&hi2c1);
    // Brief delay
    HAL_Delay(10);
    // Reinit I2C
    HAL_I2C_Init(&hi2c1);
}

//void reset_UART1(void){
//	//MX_USART1_UART_DeInit();
//	HAL_Delay(10);
//	MX_USART1_UART_Init();
//	HAL_Delay(10);
//}

