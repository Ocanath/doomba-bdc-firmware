/*
 * dartt_params.c
 *
 *  Created on: Sep 13, 2025
 *      Author: Ocanath Robotman
 */
#include "dartt_mctl_params.h"


dartt_mctl_params_t gl_dp =
{
		.command_word = 0,

		.theta_rem_m = 0,	//position 32bit in 14bit ticks
		.iq = 0,	//q axis current
		.dtheta_fixedpoint_rad_p_sec = 0,	//velocity in fixedpoint rad/sec
		.ovfl_cnt = 0,
		.id = 0,	//d axis current
		.ia = 0,	//phase current a
		.ib = 0,	//phase current b
		.ic = 0, //phase current c
		.theta_offset = 0,

		/*Initialize params for position control using FOC as 0*/
		.mctl_iq = {
	               .kpki = {
	                               .kp = {.i32 = 300, .radix = 8},
	                               .ki = {.i32 = 3, .radix = 11},
	                               .x_integral_div = 15,
	                               .x = 0, //no reason to init this to another value (unless you want the motor to jump when you start pctl... like a psycho...)
	                               .x_sat = 1000,  //no integral efforts for now
	                               .out_rshift = 0
	               },
	              .kd = { //filter this will help for pos control. Need to do fixed point IIR
	                               .i32 = 50,       //no deriv effort either, for now
	                               .radix = 5
	               },
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

		.open_loop_vq = 0,
		.open_loop_vd = 100,

		.unused_1 = 0,
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
				.startup_control_mode = PCTL_VQ,
				.unused1 = 0,
				.unused2 = 0,
				.unused3 = 0,
				.baudrate = 921600,
				.misc4 = {{0}},
				.misc5 = {{0}},

				/*every single one of these values needs a command. i might run out of room with an 8bit header...*/
				.elec_conv_ratio_fixed = 11,
				.gl_prop_delayloop_interval = 100,
				.gl_prop_delay_const_12b = 0x7FFFFFFF,
				.iq_pi = {
					.kp = {
							.i32 = 50,
							.radix = 8
					},
					.ki = {
							.i32 = 3,
							.radix = 10
					},
					.x_integral_div = 10,	//this does NOT need a command tho, helper variable. KEEP INITIALIZATION TO 0 (or it'll clamp to x_sat before the controller wrangles it as the worst case scenario)
					.x_sat = 1000,
					.out_rshift = 0
				},
				.id_pi = {
					.kp = {
							.i32 = 50,
							.radix = 8
					},
					.ki = {
							.i32 = 3,
							.radix = 10
					},
					.x_integral_div = 10,
					.x_sat = 1000,
					.out_rshift = 0
				},
				.phase_a_mid = 0,
				.phase_b_mid = 0,
				.phase_c_mid = 0,
				.phase_a_5Aval = 0,
				.phase_b_mid = 0,
				.phase_c_mid = 0
		},
		.autocalibration_voltage = 500,
		.load_action = 0,
		.action_flag = 0
};
