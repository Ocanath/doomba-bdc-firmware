/*
 * settings.h
 *
 *  Created on: Aug 13, 2018
 *      Author: Ocanath
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_
#include "stm32f0xx_hal.h"

//#define GET_INIT_OFFSET
//#define GET_ENCODER_MIDPOINT
#define TEST_FOC

uint32_t i2c_address;
const float iq_gain;
const float id_gain;
float align_offset;				//offset angle IN RADIANS
const float elec_conv_ratio;


#endif /* SETTINGS_H_ */
