/*
 * settings.h
 *
 *  Created on: Aug 13, 2018
 *      Author: Ocanath
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_
#include "stm32f3xx_hal.h"

#define EXTERNAL_OSCILLATOR

#ifdef EXTERNAL_OSCILLATOR
#define COMM_PWM_PERIOD 1000//1125?
#else
#define COMM_PWM_PERIOD 1000
#endif

#define FLIP_TAU

//#define GET_INIT_OFFSET
//#define GET_ENCODER_MIDPOINT
//#define TEST_FOC

//#define GIMBAL_MOTOR

float enc_mid_voltage_amp;

uint32_t i2c_address;
const float iq_gain;
const float id_gain;
float align_offset;				//offset angle IN RADIANS
const float elec_conv_ratio;


#endif /* SETTINGS_H_ */
