/*
 * dartt_params.h
 *
 *  Created on: Sep 13, 2025
 *      Author: Ocanath Robotman
 */

#ifndef INC_DARTT_PARAMS_H_
#define INC_DARTT_PARAMS_H_
#include "profiles.h"
#include "pctl.h"

typedef enum {FOC_MODE, SINUSOIDAL_MODE, PCTL_IQ, PCTL_VQ, OPEN_LOOP_MODE} control_mode_t;	//foc with velocity?

typedef struct dartt_params_t
{

	int32_t command_word;

	pctl_params_t mctl_iq;
	pctl_params_t mctl_vq;


	int32_t cal_openloop_voltage;
	int32_t cal_voltage;

	uint8_t en_blink_led;
	uint8_t use_uart_encoder;
	uint8_t control_mode;
	uint8_t led_state;

	fds_motor_params_t fds_mp;
	uint32_t update_fs;	//use uint32_t because it'll get padded anyway
}dartt_params_t;

extern dartt_params_t gl_dp;

#endif /* INC_DARTT_PARAMS_H_ */
