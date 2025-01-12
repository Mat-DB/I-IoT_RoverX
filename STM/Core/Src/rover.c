/*
 * uart_rover.c
 *
 *  Created on: Dec 18, 2024
 *      Author: matthias
 */

#include <rover.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;


int Rover_SendCommand_1000(char *command, char *expected) {
	return Rover_SendCommand(command, expected, 1000);
}


int Rover_SendCommand(char *command, char *expected, uint32_t timeout) {
	int retVal=0;
	#if UART_DEBUG
		const uint8_t temp_length=120;
		char temp[temp_length];
	#endif

	char r_buf[100]={0};
    HAL_UART_Transmit(&huart1, (uint8_t*) command, strlen(command), 100);

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


void drive_rover(){
	uint8_t command[16];  // Join command for LoRaWAN E5
	snprintf((char*) command, 16, "drvstr 200 100\n");
	HAL_UART_Transmit(&huart1, command, strlen((char*) command), 100);


	#if UART_DEBUG
	  printf("Driving started\r\n");
	#endif
}


void drive_sequence_master(void) {
//	short int var;
//	short int var1;

	#if UART_DEBUG
	  printf("Driving started\r\n");
	#endif

	 // START FORWARD 3sec
    HAL_GPIO_WritePin(Rover_Button_GPIO_Port, Rover_Button_Pin, GPIO_PIN_RESET);
    HAL_Delay(150); // Delay for  seconds
    HAL_GPIO_WritePin(Rover_Button_GPIO_Port, Rover_Button_Pin, GPIO_PIN_SET);

    HAL_Delay(800);

    // Start/stop command
    HAL_GPIO_WritePin(Rover_Button_GPIO_Port, Rover_Button_Pin, GPIO_PIN_RESET);
    HAL_Delay(150); // Delay for  seconds
    HAL_GPIO_WritePin(Rover_Button_GPIO_Port, Rover_Button_Pin, GPIO_PIN_SET);

    // END FORWARD 3sec


    // START LEFT 3sec
    //left
    /*
	for (var = 0; var < 6; ++var) {
		HAL_GPIO_WritePin(Rover_Left_GPIO_Port, Rover_Left_Pin,GPIO_PIN_RESET );
		HAL_Delay(500); // Delay for  seconds
		HAL_GPIO_WritePin(Rover_Left_GPIO_Port, Rover_Left_Pin,GPIO_PIN_SET );
		HAL_Delay(500);
	}

    // Start/stop command
    HAL_GPIO_WritePin(Rover_Button_GPIO_Port, Rover_Button_Pin, GPIO_PIN_RESET);
    HAL_Delay(500); // Delay for  seconds
    HAL_GPIO_WritePin(Rover_Button_GPIO_Port, Rover_Button_Pin, GPIO_PIN_SET);

    HAL_Delay(3000);
    // Start/stop command
    HAL_GPIO_WritePin(Rover_Button_GPIO_Port, Rover_Button_Pin,GPIO_PIN_RESET);
    HAL_Delay(500); // Delay for  seconds
    HAL_GPIO_WritePin(Rover_Button_GPIO_Port, Rover_Button_Pin,GPIO_PIN_SET);
	*/
    // END LEFT 3sec

    // START RIGHT 3sec
    // right
    /*
    for (var1 = 0; var1 < 6; ++var1) {
    		HAL_GPIO_WritePin(Rover_Right_GPIO_Port, Rover_Right_Pin,GPIO_PIN_RESET );
    		HAL_Delay(500); // Delay for  seconds
    		HAL_GPIO_WritePin(Rover_Right_GPIO_Port, Rover_Right_Pin, GPIO_PIN_SET);
    		HAL_Delay(500);
    }

    // Start/stop command
    HAL_GPIO_WritePin(Rover_Button_GPIO_Port, Rover_Button_Pin, GPIO_PIN_RESET);
    HAL_Delay(500); // Delay for  seconds
    HAL_GPIO_WritePin(Rover_Button_GPIO_Port, Rover_Button_Pin, GPIO_PIN_SET);

    HAL_Delay(3000);
    // Start/stop command
    HAL_GPIO_WritePin(Rover_Button_GPIO_Port, Rover_Button_Pin,GPIO_PIN_RESET);
    HAL_Delay(500); // Delay for  seconds
    HAL_GPIO_WritePin(Rover_Button_GPIO_Port, Rover_Button_Pin,GPIO_PIN_SET);
	*/
    // END RIGHT 3sec

	#if UART_DEBUG
	  printf("Driving stopped\r\n");
	#endif
}



