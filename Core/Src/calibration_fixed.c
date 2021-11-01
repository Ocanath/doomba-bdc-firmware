/*
 * calibration_fixed.c
 *
 *  Created on: Oct 26, 2021
 *      Author: Ocanath Robotman
 */
#include "foc_fixed.h"
#include "init.h"
#include "foc_commutation.h"

int64_t abs64(int64_t in)
{
	if(in > 0)
		return in;
	else
		return -in;
}

int32_t run_v_fixed(int32_t v, uint32_t run_time, int64_t * dist)
{
	for(uint32_t stop_ts = HAL_GetTick() + 100; HAL_GetTick()<stop_ts;)	//initial run for SS
		sinusoidal_fixed(v, 0);	//todo: replace this with a voltage control mode call
	int64_t run_start = gl_theta_rotations*OVERFLOW_WRAP_VAL + gl_theta_rem_m;
	for(uint32_t stop_ts = HAL_GetTick() + run_time; HAL_GetTick()<stop_ts;)	//the run
		sinusoidal_fixed(v, 0);
	int64_t run_end = gl_theta_rotations*OVERFLOW_WRAP_VAL + gl_theta_rem_m;
	TIMER_UPDATE_DUTY(0,0,0);
	HAL_Delay(50);
	*dist = run_end-run_start;
	int64_t run_speed = (( (*dist) << 16)/run_time);
	if(run_speed >= 0)
		return run_speed;
	else
		return -run_speed;
}


//void auto_calibrate_fixed(void)
//{
//	int16_t passcount = 0;
//	while(1)
//	{
//		int32_t v = MAX_VQ/2;
//		int64_t dist1,dist2;
//		int64_t run1_speed = run_v_fixed(v, 400, &dist1);
//		int64_t run2_speed = run_v_fixed(-v, 400, &dist2);
//		int8_t sign = 0;
//		if(dist1 > 0 && dist2 < 0)
//			sign = 1;
//		else if (dist1 < 0 && dist2 > 0)
//			sign = -1;
//		if(sign < 0 && abs64(dist1) > 24576 && abs64(dist2) > 24576 )
//			align_offset_fixed = -align_offset_fixed;
//
//		int64_t err = run1_speed-run2_speed;
//		int64_t u = (err* (1 << 16) )/7;	//apply a 12 bit leftshift and apply control effort reduction
//		if(sign < 0)
//			u = -u;
//		align_offset_fixed = (u + align_offset_fixed) >> 32;	//remove bitshifts applied in run_v and in the control effort calculation
//
//		if(abs64(u) < 6)	//pass criteria .0001
//		{
//			passcount++;
//			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);	//count the number of passes
//		}
//		else
//			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
//		if(passcount > 10)
//			break;	//if you've passed over 10 times, you're done!
//	}
//}

/*
 *Automatically find the encoder offset
 */
void auto_calibrate_fixed(void)
{
	TIMER_UPDATE_DUTY(0,0,0);
//	align_offset = -PI/elec_conv_ratio;
	int passcount = 0;
	while(1)
	{
		int32_t v = MAX_VQ/2;
		int64_t dist1,dist2;
		int64_t r1_sp = run_v_fixed(v, 400, &dist1);
		int64_t r2_sp = run_v_fixed(-v, 400, &dist2);

		double run1_speed = ((double)r1_sp)/((double)(1 << 16));
		double run2_speed = ((double)r2_sp)/((double)(1 << 16));
		float run_dist_1 = ((float)dist1)/4096.f;
		float run_dist_2 = ((float)dist2)/4096.f;

		float sign = 0.f;
		if(run_dist_1 > 0.f && run_dist_2 < 0.f)
			sign = 1.f;		//the direction is how it should be. positive tau means positive encoder direction
		else if (run_dist_1 < 0 && run_dist_2 > 0)
			sign = -1.f;	//the direction is flipped. positive tau means negative encoder direction

		if(sign < 0 && abs_f(run_dist_1) > 6.f && abs_f(run_dist_2) > 6.f)
			align_offset = -align_offset;		//if you're flipped and you have moved appreciably, flip the align offset and the sign should flip

		double err = run1_speed-run2_speed;	//compute base term for control effort
		double u = err/7.0;	//scale control effort down (k gain)

		align_offset += u*sign;	//adjust the align offset to converge on a value that matches the run speeds (zeros the difference)
		align_offset_fixed = (int32_t)(align_offset * 4096.f);
		if(abs_f(u) < 0.0001f)	//if the run speed difference is very small, it's a passable align offset
		{
			passcount++;
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);	//count the number of passes
		}
		else
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
		if(passcount > 10)
			break;	//if you've passed over 10 times, you're done!
	}
}
