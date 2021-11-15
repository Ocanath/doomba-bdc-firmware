/*
 * profiles.h
 *
 *  Created on: Nov 14, 2021
 *      Author: Ocanath Robotman
 */

#ifndef INC_PROFILES_H_
#define INC_PROFILES_H_
#include "stdint.h"

#define AEROSKY_MC4225 		0
#define MULTISTAR_MS4822 	1
#define	MULTISTAR_MS4225	2


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

typedef struct fixed_PI_params_t
{
	int32_t kp_m;
	int32_t kp_div;
	int32_t ki_m;
	int32_t ki_div;
	int32_t x;
	int32_t x_sat;
}fixed_PI_params_t;
extern const int32_t elec_conv_ratio_fixed;
extern const int32_t gl_prop_delay_const_12b;
extern const int32_t gl_prop_delayloop_interval;
extern fixed_PI_params_t iq_pi;
extern fixed_PI_params_t id_pi;
extern int32_t align_offset_fixed;


extern const float elec_conv_ratio;
extern const float iq_gain;
extern const float id_gain;
extern float align_offset;				//offset angle IN RADIANS
extern const float gl_prop_delay_const;

#endif /* INC_PROFILES_H_ */
