/*
 * settings.c
 *
 *  Created on: Sep 9, 2018
 *      Author: Ocanath
 */
#include <FDCAN.h>
#include "settings.h"
#include "trig_fixed.h"
#include "dartt_mctl_params.h"

/*
 * NOTE:
 *There are some dead serial numbers here. Calibration is so quick... try and be descriptive in your annotations,
 *and delete serial nums you know are dead, but if you get a bad serial number just do a new one. Generally higher numbers are
 *more likely to be ok.
 *
 *
 *
 * EXTREMELY IMPORTANT NOTE!!!!!!!!!!!!!
 * MEGA CRAZY IMPORTANT!
 * IDK WHAT THE FUCK!
 *
 *
 *
 * Positive align offsets... cause control instability
 * idk why
 * it's fucking weird
 * but it's true
 *
 */


#ifdef GIMBAL_MOTOR
float enc_mid_voltage_amp = .6;
#else
float enc_mid_voltage_amp = .09;
#endif

/*default settings, corresponding to i think motor number 38.
 * Oh shit... how do i manage that....
 *
 * Think i gotta do it manually
 * so it'll be the first member variable of the struct
 *
 *
 *
 *
 * TO VIEW THIS MEMORY:
 * use STM32CubeProgrammer with size set to the page size (0x800)
 * and the page address set to page 31 (or the last page), i.e. 0x80000F800
 * */
fds_motor_params_t * fds_mp = &gl_dp.fds_mp;





/*Delete these stragglers...*/
const float iq_gain = 0.03f;
const float id_gain = 0.01f;
const float gl_prop_delay_const = 900.f;




/*
 * Helper function to load all the fds_mp params
 * Must go before CAN init for can ID to work properly
 * */
void load_flash_params(void)
{
	//loading
	if(is_page_empty(sizeof(fds_mp)/sizeof(uint32_t)) == 0)
	{
		m_read_flash((uint32_t*)fds_mp,sizeof(fds_motor_params_t)/sizeof(uint32_t));
	}
	else
	{
		m_write_flash((uint64_t*)fds_mp,sizeof(fds_motor_params_t)/sizeof(uint64_t));
	}

	//sanitize variables and flags
	gl_ext_cmd_id = (0x7FF - fds_mp->module_number);	//load this ONCE on startup
	if(fds_mp->is_flipped != 0)
	{
		fds_mp->is_flipped = 1;	//sanitize flag
	}
	if(gl_dp.fds_mp.baudrate == 0xFFFFFFFF || gl_dp.fds_mp.baudrate == 0)	//sanitize baudrate
	{
		gl_dp.fds_mp.baudrate = 921600;	//proper default - handle for uninitialized
	}
}


/*
 * Initialization helper function to sanitize the x_integral_div parameter, in order to ensure that it does not cause a divide by zero error when transferring from one representation to another.
 * */
void check_idpi_xdiv_param(void)
{
	if(fds_mp->id_pi.x_integral_div == 0)
		fds_mp->id_pi.x_integral_div = 1;	//pick the value with no effect on calculation between original and new
	if(fds_mp->iq_pi.x_integral_div == 0)
		fds_mp->iq_pi.x_integral_div = 1;	//pick the value with no effect on calculation between original and new
}


void write_flash_params(void)
{
	fds_mp->elec_conv_ratio_fixed = 7;
	fds_mp->align_offset_fixed = 1222;
	fds_mp->iq_pi = (fixed_PI_params_t){
		.kp = {
				.i32 = 10000,
				.radix = 3
		},
				.ki = {
						.i32 = 10000,
						.radix = 7
		},
		.x_integral_div = 1,	//this does NOT need a command tho, helper variable. KEEP INITIALIZATION TO 0 (or it'll clamp to x_sat before the controller wrangles it as the worst case scenario)
		.x_sat = MAX_VQ,
		.out_rshift = 9
	};
	fds_mp->id_pi = (fixed_PI_params_t){
		.kp = {
				.i32 = 5000,
				.radix = 5
		},
				.ki = {
						.i32 = 10000,
						.radix = 7
		},
		.x_integral_div = 1,	//this does NOT need a command tho, helper variable. KEEP INITIALIZATION TO 0 (or it'll clamp to x_sat before the controller wrangles it as the worst case scenario)
		.x_sat = MAX_VQ,
		.out_rshift = 9
	};
	m_write_flash((uint64_t*)fds_mp,sizeof(fds_motor_params_t)/sizeof(uint64_t));
}

