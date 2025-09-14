/*
 * dartt_params.c
 *
 *  Created on: Sep 13, 2025
 *      Author: Ocanath Robotman
 */
#include "dartt_params.h"


dartt_params_t gl_dp =
{


		/*Initialize params for position control using FOC as 0*/
		.mctl_iq = {
		               .kpki =
		               {
		                               .kp = {.i32 = 0, .radix = 0},
		                               .ki = {.i32 = 0, .radix = 0},
		                               .x = 0,
		                               .x_sat = 0,
		                               .out_rshift = 0
		               },
		               .kd = {.i32 = 0, .radix = 0},
		               .out_sat = 0,
		},
		/*Initialize params for position control using sinusoidal as 0*/
		.mctl_vq = {
		               .kpki = {
		                               .kp = {.i32 = 50, .radix = 8},
		                               .ki = {.i32 = 3, .radix = 10},
		                               .x_integral_div = 10,
		                               .x = 0, //no reason to init this to another value (unless you want the motor to jump when you start pctl... like a psycho...)
		                               .x_sat = 1000,  //no integral efforts for now
		                               .out_rshift = 0
		               },
		              .kd = { //filter this will help for pos control. Need to do fixed point IIR
		                               .i32 = 5,       //no deriv effort either, for now
		                               .radix = 5
		               },
		               .out_sat = 0,
		},


		.cal_openloop_voltage = 100,
		.cal_voltage = 1000,

		.en_blink_led = 0,
		.use_uart_encoder = 0,
		.control_mode = PCTL_VQ,
		.led_state = 0,

		.fds_mp =
		{
				.module_number = 0,

				/*The all important align offset*/
				.align_offset_fixed = 0,

				/*These are not hand obtained ever, so no commands for them*/
				.is_flipped = 1,
				.misc2 = {{0}},
				.misc3 = {{0}},
				.misc4 = {{0}},
				.misc5 = {{0}},

				/*every single one of these values needs a command. i might run out of room with an 8bit header...*/
				.elec_conv_ratio_fixed = 11,
				.gl_prop_delayloop_interval = 100,
				.gl_prop_delay_const_12b = 0x7FFFFFFF,
				.iq_pi = {
					.kp = {
							.i32 = 0,
							.radix = 0
					},
					.ki = {
							.i32 = 0,
							.radix = 0
					},
					.x_integral_div = 1,	//this does NOT need a command tho, helper variable. KEEP INITIALIZATION TO 0 (or it'll clamp to x_sat before the controller wrangles it as the worst case scenario)
					.x_sat = 0,
					.out_rshift = 31
				},
				.id_pi = {
					.kp = {
							.i32 = 0,
							.radix = 31
					},
					.ki = {
							.i32 = 0,
							.radix = 31
					},
					.x_integral_div = 1,
					.x_sat = 0,
					.out_rshift = 31
				}
		},

		.update_fs = 0
};
