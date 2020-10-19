/*
 * settings.h
 *
 *  Created on: Aug 13, 2018
 *      Author: Ocanath
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_
#include "stm32f3xx_hal.h"

#define USE_UART_ENCODER

#define EXTERNAL_OSCILLATOR

#ifdef EXTERNAL_OSCILLATOR
#define COMM_PWM_PERIOD 1000//1125?
#else
#define COMM_PWM_PERIOD 1000
#endif

//#define FLIP_TAU

//#define GET_INIT_OFFSET
//#define GET_ENCODER_MIDPOINT
//#define TEST_FOC

//#define GIMBAL_MOTOR

extern float enc_mid_voltage_amp;

//uint32_t i2c_address;
extern uint32_t ser_num;
extern const float iq_gain;
extern const float id_gain;
extern float align_offset;				//offset angle IN RADIANS
extern const float elec_conv_ratio;

#endif /* SETTINGS_H_ */
