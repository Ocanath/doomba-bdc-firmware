/*
 * settings.h
 *
 *  Created on: Aug 13, 2018
 *      Author: Ocanath
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_
#include "stm32f3xx_hal.h"
/***********************Begin define hardware precomp settings*********************/

/*Motor serial number, for the #define lookup list in settings.c . Also the CAN Node ID*/
#define MODULE_NUMBER 32

/*Is a KMZ60 or TLE encoder populated?
 * 	Note: elec_conv_ratio needs to be defined based on this number. elec_conv_ratio is
 * 	equal to the number of pole-pairs of the motor for TLE encoder, and it is equal to the
 * 	number of pole-pairs / 2 for the KMZ60 (including fractions).
 * */
//#define USE_KMZ60_ENCODER

/*Is the external 16Mhz oscillator populated? (Note: all hw should have this populated...)*/
#define EXTERNAL_OSCILLATOR

/*******************************************END************************************/

/**/
#ifdef EXTERNAL_OSCILLATOR
#define COMM_PWM_PERIOD 1125//1125?
#else
#define COMM_PWM_PERIOD 1000
#endif

#define MAX_VQ 3546	// (sqrt(3)/2) * 4096

//#define FLIP_TAU

//#define GET_INIT_OFFSET
//#define GET_ENCODER_MIDPOINT
//#define TEST_FOC

//#define GIMBAL_MOTOR

typedef struct fixed_PI_params_t
{
	int32_t kp_m;
	int32_t kp_div;
	int32_t ki_m;
	int32_t ki_div;
	int32_t x;
	int32_t x_sat;
}fixed_PI_params_t;

extern float enc_mid_voltage_amp;
//uint32_t i2c_address;
extern uint32_t ser_num;
extern const float iq_gain;
extern const float id_gain;
extern float align_offset;				//offset angle IN RADIANS
extern int32_t align_offset_fixed;
extern const float elec_conv_ratio;
extern const int32_t elec_conv_ratio_fixed;
extern const int32_t gl_prop_delay_const_12b;
extern const int32_t gl_prop_delayloop_interval;
extern fixed_PI_params_t iq_pi;
extern fixed_PI_params_t id_pi;

#endif /* SETTINGS_H_ */
