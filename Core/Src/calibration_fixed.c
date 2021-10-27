/*
 * calibration_fixed.c
 *
 *  Created on: Oct 26, 2021
 *      Author: Ocanath Robotman
 */
#include "foc_fixed.h"
#include "init.h"

/*
 * Ramp up until steady state speed is high
 * Record the beginning and end of the run after SS speed is hit
 * return the average speed
*/
int32_t run_v_fixed(int32_t v, uint32_t run_time, float * run_distance)
{
//	for(uint32_t stop_ts = HAL_GetTick() + 100; HAL_GetTick()<stop_ts;)	//initial run for SS
//		sinusoidal_fixed(v, 0);	//todo: replace this with a voltage control mode call
//
//	float run_start = gl_theta_m;
//	for(uint32_t stop_ts = HAL_GetTick() + run_time; HAL_GetTick()<stop_ts;)	//the run
//		sinusoidal_fixed(v, 0);
//	float run_end = gl_theta_m;
//	TIMER_UPDATE_DUTY(0,0,0);
//	HAL_Delay(50);
//	*run_distance = run_end - run_start;
//	float run_time_f = ((float)run_time);
//	float run_speed = *run_distance/run_time_f;
//	return abs_f( run_speed ); //proportional to percent error when you replace TAU with V as in voltage
	return 0;
}

/*
 *Automatically find the encoder offset
 */
void auto_calibrate_fixed(void)
{
//	TIMER_UPDATE_DUTY(0,0,0);
////	align_offset = -PI/elec_conv_ratio;
//	int passcount = 0;
//	while(1)
//	{
//		float v = .5f;	//voltage percent out of sqrt(3)/2 (applied to supply voltage)
//		float run_dist_1,run_dist_2;
//		float run1_speed = run_v(v,400, &run_dist_1);	//returns absolute value of the run speed, with the total distance of the run output to the dist var
//		float run2_speed = run_v(-v,400, &run_dist_2);
//
//		float sign = 0.f;
//		if(run_dist_1 > 0.f && run_dist_2 < 0.f)
//			sign = 1.f;		//the direction is how it should be. positive tau means positive encoder direction
//		else if (run_dist_1 < 0 && run_dist_2 > 0)
//			sign = -1.f;	//the direction is flipped. positive tau means negative encoder direction
//
//		if(sign < 0 && abs_f(run_dist_1) > 6.f && abs_f(run_dist_2) > 6.f)
//			align_offset = -align_offset;		//if you're flipped and you have moved appreciably, flip the align offset and the sign should flip
//
//		double err = run1_speed-run2_speed;	//compute base term for control effort
//		double u = err/7.0;	//scale control effort down (k gain)
//
//		align_offset += u*sign;	//adjust the align offset to converge on a value that matches the run speeds (zeros the difference)
//		if(abs_f(u) < 0.0001f)	//if the run speed difference is very small, it's a passable align offset
//		{
//			passcount++;
//			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);	//count the number of passes
//		}
//		else
//			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
//		if(passcount > 10)
//			break;	//if you've passed over 10 times, you're done!
//	}
}
