/*
 * settings.c
 *
 *  Created on: Sep 9, 2018
 *      Author: Ocanath
 */
#include "settings.h"
#include "trig_fixed.h"
#include "CAN.h"

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
fds_motor_params_t fds_mp =
{
	.module_number = 0,

	/*The all important align offset*/
	.align_offset_fixed = 902,

	/*These are not hand obtained ever, so no commands for them*/
	.gl_cos_min = 276,
	.gl_cos_max = 3737,
	.gl_sin_min = 274,
	.gl_sin_max = 3704,
	.gl_oerr_total = 74,

	/*every single one of these values needs a command. i might run out of room with an 8bit header...*/
	.elec_conv_ratio_fixed = 7,
	.gl_prop_delayloop_interval = 60,
	.gl_prop_delay_const_12b = 3,
	.iq_pi = {
		.kp = {
				.i32 = 5218,
				.radix = 5
		},
		.ki = {
				.i32 = 6498,
				.radix = 9
		},
		.x = 0,	//this does NOT need a command tho, helper variable. KEEP INITIALIZATION TO 0 (or it'll clamp to x_sat before the controller wrangles it as the worst case scenario)
		.x_sat = MAX_VQ,
		.out_rshift = 10
	},
	.id_pi = {
		.kp = {
				.i32 = 0,
				.radix = 12
		},
		.ki = {
				.i32 = 30,
				.radix = 12
		},
		.x = 0,
		.x_sat = 0,
		.out_rshift = 12
	}
};





/*Delete these stragglers...*/
float align_offset;
const float iq_gain = 0.03f;
const float id_gain = 0.01f;
const float gl_prop_delay_const = 900.f;




/*
 * Helper function to load all the fds_mp params
 * Must go before CAN init for can ID to work properly
 * */
void load_flash_params(void)
{
	if(is_page_empty(sizeof(fds_mp)/sizeof(uint32_t)) == 0)
		m_read_flash((uint32_t*)&fds_mp,sizeof(fds_motor_params_t)/sizeof(uint32_t));
	else
		m_write_flash((uint32_t*)&fds_mp,sizeof(fds_motor_params_t)/sizeof(uint32_t));
	gl_ext_cmd_id = (0x7FF - fds_mp.module_number);	//load this ONCE on startup
}


