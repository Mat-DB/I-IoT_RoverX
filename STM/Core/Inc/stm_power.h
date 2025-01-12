/*
 * stm_power.h
 *
 *  Created on: Jan 8, 2025
 *      Author: matthias
 */

#include <string.h>
#include <stdio.h>
#include "main.h"

#ifndef INC_STM_POWER_H_
#define INC_STM_POWER_H_

void enterStandbyMode(void);
void enterStandbyModeMaster();
void enterStandbyModeShort(void);

void reset_i2c(void);

#endif /* INC_STM_POWER_H_ */
