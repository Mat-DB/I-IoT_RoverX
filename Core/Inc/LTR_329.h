/*
 * LTR_329.h
 *
 *  Created on: Nov 19, 2024
 *      Author: matthias
 */

#include <string.h>
#include <stdio.h>
#include "main.h"

#ifndef SRC_LTR_329_H_
#define SRC_LTR_329_H_

#define LTR329_I2CADDR (0x29 << 1) ///< I2C address
#define LTR329_PART_ID 0x86         ///< Part id/revision register
#define LTR329_MANU_ID 0x87         ///< Manufacturer ID register
#define LTR329_ALS_CTRL 0x80        ///< ALS control register
#define LTR329_STATUS 0x8C          ///< Status register
#define LTR329_CH1DATA 0x88         ///< Data for channel 1 (read all 4 bytes!)
#define LTR329_MEAS_RATE 0x85       ///< Integration time and data rate

/*!    @brief  Sensor gain for ALS  */
typedef enum {
  LTR3XX_GAIN_1 = 0,
  LTR3XX_GAIN_2 = 1,
  LTR3XX_GAIN_4 = 2,
  LTR3XX_GAIN_8 = 3,
  // 4 & 5 unused!
  LTR3XX_GAIN_48 = 6,
  LTR3XX_GAIN_96 = 7,
} ltr329_gain_t;

/*!    @brief Integration times, in milliseconds */
typedef enum {
  LTR3XX_INTEGTIME_100,
  LTR3XX_INTEGTIME_50,
  LTR3XX_INTEGTIME_200,
  LTR3XX_INTEGTIME_400,
  LTR3XX_INTEGTIME_150,
  LTR3XX_INTEGTIME_250,
  LTR3XX_INTEGTIME_300,
  LTR3XX_INTEGTIME_350,
} ltr329_integrationtime_t;

/*!    @brief Measurement rates, in milliseconds */
typedef enum {
  LTR3XX_MEASRATE_50,
  LTR3XX_MEASRATE_100,
  LTR3XX_MEASRATE_200,
  LTR3XX_MEASRATE_500,
  LTR3XX_MEASRATE_1000,
  LTR3XX_MEASRATE_2000,
} ltr329_measurerate_t;


void LTR_329_setup(void);
void LTR_329_measure(void);


#endif /* SRC_LTR_329_H_ */
