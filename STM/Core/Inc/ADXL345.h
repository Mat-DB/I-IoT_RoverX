/*
 * ADXL345.h
 *
 *  Created on: Jan 8, 2025
 *      Author: matthias
 */

#include <string.h>
#include <stdio.h>
#include "main.h"

#ifndef INC_ADXL345_H_
#define INC_ADXL345_H_

#define ADXL345_I2C_ADDR (0x53 << 1)

void ADXL345_Init(void);
void ADXL345_WakeUp(void);
void ADXL345_Measure(void);
void ADXL345_Sleep(void);

#endif /* INC_ADXL345_H_ */
