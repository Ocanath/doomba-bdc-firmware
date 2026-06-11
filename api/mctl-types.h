/*
 * profiles.h
 *
 *  Created on: Nov 14, 2021
 *      Author: Ocanath Robotman
 */

#ifndef INC_MCTL_TYPES_H_
#define INC_MCTL_TYPES_H_
#include "stdint.h"
#include "pctl.h"

/**/
#define COMM_PWM_PERIOD 1000	//nets us about 85khz, with around 12bit voltage resolution. speed beats reso.

#define MAX_VQ 3546	// (sqrt(3)/2) * 4096



typedef struct fixed_PI_params_t
{
	i32_t kp;
	i32_t ki;
	int32_t x_integral_div;	//no x in this structure. replace X from previous implementation with xdiv, and use external x parameter
	int32_t x_sat;
	uint8_t out_rshift;
}fixed_PI_params_t;

typedef union {
	int8_t d8[sizeof(uint32_t)/sizeof(int8_t)];
	uint8_t u8[sizeof(uint32_t)/sizeof(uint8_t)];
	uint16_t u16[sizeof(uint32_t)/sizeof(uint16_t)];
	int16_t i16[sizeof(uint32_t)/sizeof(int16_t)];
	uint32_t u32;
	int32_t i32;
	float f32;	//sizeof(float) == sizeof(uint32_t) on this system
}u32_fmt_t;


typedef struct fds_motor_params_t
{
	/*CAN id. THIS. MUST. BE. FIRST.*/
	uint32_t module_number;

	/*Properties of the motor which have to be measured and differ for each instance*/
	int32_t align_offset_fixed;
	int32_t is_flipped;	//misc 1-5 are encoder related, and obsolete. they do not apply when AMS encoder is used
	uint8_t startup_control_mode;
	uint8_t unused1;
	uint8_t unused2;
	uint8_t unused3;
	uint32_t baudrate;
	u32_fmt_t misc4;
	u32_fmt_t misc5;


	/*Properties of the motor which are either inherent or do not change between instances
	 * These must be written by the CAN master*/
	int32_t elec_conv_ratio_fixed;
	int32_t gl_prop_delayloop_interval;
	int32_t gl_prop_delay_const_12b;
	fixed_PI_params_t iq_pi;
	fixed_PI_params_t id_pi;

	int32_t phase_a_mid;	//current sensor mid, phase A (0A adc value)
	int32_t phase_b_mid;	//current sensor mid, phase B
	int32_t phase_c_mid;	//current sensor mid, phase C

	int32_t phase_a_5Aval;	//raw adc value at 5A, phase A
	int32_t phase_b_5Aval;	//raw adc value at 5A, phase B
	int32_t phase_c_5Aval;	//raw adc value at 5A, phase C

}fds_motor_params_t;

#endif /* INC_PROFILES_H_ */
