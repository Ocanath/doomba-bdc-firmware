/*
 * calibration.c
 *
 *  Created on: Apr 29, 2019
 *      Author: Ocanath
 */
#include "calibration.h"
#include "mag-encoder.h"


void print_float(float v)
{
	uint8_t * ptr = (uint8_t * )(&v);
	HAL_UART_Transmit(&huart2, ptr, 4, 10);
}
void print_str(const char * str)
{
	int strlen;
	for(strlen=0;str[strlen] != 0; strlen++);
	HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen, 10);
}


/*
 * check and correct the encoder region for KMZ60 encoder. Necessary because encoder returns
 * twice the actual angle, meaning that 1/2 of the time, sin and cosine will be the SAME value
 * for 2 DIFFERENT motor angles. This means that FOC will think positive is positive 1/2 the time,
 * and positive is negative the other half of the time.
 *
 * This function operates by performing a local position control loop. The setpoint is the current
 * motor position with a slight positive offset. This is so that the initial 'jerk' from the motor is not
 * great. It will try position control. If the error exceeds some threshold (greater than the guaranteed
 * inital error for a still motor) it knows that it's spinning the wrong way. It will then reverse the
 * motor direction by compensating for the INCORRECT initial error, and guard direction reversals for
 * a period of time. If the motor reaches it's setpoint (to within an acceptable margin) for some time,
 * this function will return. If not, it will try reversing again, and allow for a longer period of time
 * before to try and settle.
 *
 * I have tested this on a gearbox motor. It is surprisingly reliable, and usually settles in less than half
 * a second unless a significant load is applied.
 *
 * The driver light will stay on until completion and blink during normal operation; additionally, it will not
 * parse i2c commands. If both lights are on, the driver is almost certainly calling this function. If it is calling
 * it indefinitely, you should try and guide the motor by hand to 'help it out'
 *
 *
 */

/*Note: original values are 0.4f, 50, .1f, .6f*/
float check_encoder_region(float start_step, uint32_t settle_time, float settle_error, float reverse_error, uint32_t timeout, uint8_t * status)
{
	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
	uint32_t exit_ts = HAL_GetTick()+timeout;
	uint32_t ts = HAL_GetTick();
	float theta_m_prev = 0;
	float theta_set = (unwrap(theta_abs_rad(), &theta_m_prev)*.5)+start_step;	//get the initial position and use it to set the motor setpoint.
	float err = 0;
	uint32_t try_ts = 100;
	uint32_t off_ts = 0;
	while(1)
	{
		float theta_m = unwrap(theta_abs_rad(), &theta_m_prev)*.5;	//get current motor position
		err = theta_set-theta_m;		// and error
		float tau = 30*err;				//position control
		if(tau > 50)
			tau = 50;
		if(tau < -50)
			tau = -50;
		foc(tau,0);

		if(err < 0)
			err = -err;		//get the absolute value of the error

		if(err > reverse_error && HAL_GetTick() > ts)	//if the error is great and you haven't tried for some time, reverse the direction
		{
			theta_enc_prev -= TWO_PI;	//used by inline foc loop
			ts = HAL_GetTick()+try_ts;
			try_ts+=50;	//if you failed on the last attempt, try for just a liiitle bit longer. This improves stability (i.e. no infinite oscillations if you're significantly out of bounds)
		}

		if(err > settle_error)	//if the error is great,
			off_ts = HAL_GetTick();	//set the timestamp
		else
		{
			if(HAL_GetTick() - off_ts > settle_time)	//if you've settled (low error for an acceptable time) break
			{
				HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
				*status =  CAL_SUCCESS;
				break;
			}
		}
		if(HAL_GetTick() > exit_ts)
			*status = CAL_ERR_TIMEOUT;
	}
	return theta_enc_prev;
}


static float abs_f(float in)
{
	if(in < 0)
		return -in;
	else
		return in;
}

//static float abs_err_prev = 0.0f;
//static uint32_t check_encoder_region_2_timestamp = 0;
//uint8_t check_encoder_region_2(float start_offset, float diff_err_thresh, uint32_t timeout)
//{
////	diff_err_thresh = abs_f(diff_err_thresh);//user input guard
//	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
//	check_encoder_region_2_timestamp = HAL_GetTick() + timeout;
//
//	float inital_position = unwrap(theta_abs_rad(), &theta_m_prev)*.5;
//	float setpoint = inital_position + start_offset;
//	uint32_t err_check_ts = HAL_GetTick() + 100;
//	uint32_t consec_pos_err_cnt = 0;
//	uint32_t consec_low_diff_err_cnt = 0;
//	while(1)
//	{
//		uint32_t period = 5;
//		float theta_m = unwrap(theta_abs_rad(), &theta_m_prev)*.5;	//get current motor position
//		float err = setpoint - theta_m;
//		float tau = 30.0f*err;				//position control
//		if(tau > 50)
//			tau = 50;
//		if(tau < -50)
//			tau = -50;
//		foc(tau,0);
//
//		if(HAL_GetTick() > err_check_ts)
//		{
//			float abs_err = abs_f(err);
//			float diff_err = abs_err - abs_err_prev;	//i.e. the change in the absolute value of the error
//
//			if(diff_err > diff_err_thresh)
//			{
//				consec_low_diff_err_cnt = 0;
//				consec_pos_err_cnt++;
//				if(consec_pos_err_cnt > 5)
//				{
//					theta_enc_prev -= TWO_PI; //used by inline foc loop, will reverse the foc direction
//					period = 50;
//				}
//			}
//			else if (diff_err < -diff_err_thresh)
//			{
//				consec_pos_err_cnt = 0;
//				consec_low_diff_err_cnt++;
//				if(consec_low_diff_err_cnt >= 5)//300ms of low error
//					return CAL_SUCCESS;
//			}
//			err_check_ts = HAL_GetTick() + period;
//			abs_err_prev = abs_err;
//		}
//
//		if(HAL_GetTick() > check_encoder_region_2_timestamp)
//			return CAL_ERR_TIMEOUT;
//	}
//	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
//}

static float time_seconds()
{
	return ((float)HAL_GetTick())*.001f;
}

void obtain_encoder_midpoints()
{
	TIMER_UPDATE_DUTY(500,500,500);
	float t_start = time_seconds();
	uint16_t s_max = 0;
	uint16_t s_min = 4096;
	uint16_t c_max = 0;
	uint16_t c_min = 4096;
	while(time_seconds() - t_start < 15)
	{
		if(dma_adc_raw[ADC_SIN_CHAN] > s_max)
			s_max = dma_adc_raw[ADC_SIN_CHAN];
		if(dma_adc_raw[ADC_SIN_CHAN] < s_min)
			s_min = dma_adc_raw[ADC_SIN_CHAN];

		if(dma_adc_raw[ADC_COS_CHAN] > c_max)
			c_max = dma_adc_raw[ADC_COS_CHAN];
		if(dma_adc_raw[ADC_COS_CHAN] < c_min)
			c_min = dma_adc_raw[ADC_COS_CHAN];


		float theta = 420*sin_fast( (fmod_2pi(.2*time_seconds() + PI) - PI) );
		theta = fmod_2pi(theta + PI) - PI;		//re-modulate theta_m. ensure that the angle is constrained from -pi to pi!!
		float sin_theta = sin_fast(theta);				//calculate the sin of the electrical (magnetic flux) angle
		float cos_theta = cos_fast(theta);				//and the cosine for park and inverse park domains
		float i_alpha,i_beta;
		uint32_t tA,tB,tC;
		inverse_park_transform(enc_mid_voltage_amp, 0, sin_theta, cos_theta, &i_alpha, &i_beta);	//maybe call theta rel again?
		svm(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);
		TIMER_UPDATE_DUTY(tA,tB,tC);
	}
	TIMER_UPDATE_DUTY(0,0,0);
	sin_mid = (s_max + s_min)/2;
	cos_mid = (c_max + c_min)/2;
}

/*
 * Checks a candidate align_offset (global variable, used by the encoder)
 * by producing a triangle speed waveform using constant positive and negative
 * torques; i.e. positive tau_ref for run_time ms, negative tau_ref  for run_time ms.
 * The function measures the position at the start of the waveform, the peak/tip of the
 * triangle, and the endpoint. Since the torque is constant, the speed is determined by physical
 * system (same conditions for both directions in an unloaded motor) and the quality of the align
 * offset; therefore, for a good offset, the error between the start and endpoints should be minimal.
 *
 * However, some align offsets produce ZERO speed since they're so terrible. To factor this in, the
 * midpoint is compared to the starting point; if the motor has not moved appreciably (a generous 1 radian)
 * then the align offset is no good, and an arbitrary large error is returned.
 *
 *In general, the align offset which produces the smallest error that is LESS than a given threshold (1 rad or less)
 *should be accepted.
 *
 *For motors with fixed encoder mounts, this function should be used in tandem with sweep_align.
 *for motors with variable encoder mounts (and fixed align offset values) this function should
 *relay visual feedback to a human operator, through the light and potentially a matlab plot.
 */
#define OUT_OF_BOUNDS_ERROR 1000000
volatile float gl_error_print = 0;
uint32_t print_ts=0;
float check_align_offset(uint32_t run_time, float tau_ref)
{
	uint32_t run_ts = HAL_GetTick() + run_time;
	float init_pos = unwrap(theta_abs_rad(), &theta_m_prev)*.5;	//get the initial motor position
	float prev_theta = 0;//need to track a second time for speed. unwrap smashes it
	uint8_t dir_correct_flag = 1;
	while(HAL_GetTick() < run_ts)
	{
		float theta_m = unwrap(theta_abs_rad(), &theta_m_prev)*.5;
		if(theta_m - prev_theta < 0)	//if speed is negative and direction is positive
			dir_correct_flag = 0;
		prev_theta = theta_m;

		foc(tau_ref,0);


		if(HAL_GetTick()>print_ts)
		{
			print_float(align_offset);
			print_float(gl_error_print);
			print_float(theta_m);
			print_ts = HAL_GetTick()+10;
		}

	}

	/*make sure we've picked an align offset that's not completely out of bounds, by checking that we've actually traveled an appreciable distance during phase 1*/
	float mid_check_err = init_pos - prev_theta;
	//	mid_check_err &= ~0x80000000;
	if(mid_check_err < 0)
		mid_check_err = -mid_check_err;
	if(mid_check_err < 1)	//if the motor didn't move during this sweep
	{
		gl_error_print = OUT_OF_BOUNDS_ERROR;
		return gl_error_print;
	}

	run_ts = HAL_GetTick() + run_time;
	while(HAL_GetTick() < run_ts)
	{
		float theta_m = unwrap(theta_abs_rad(), &theta_m_prev)*.5;
		if(theta_m - prev_theta > 0)	//if speed is positive and direction is negative
			dir_correct_flag = 0;
		prev_theta = theta_m;

		foc(-tau_ref,0);

		if(HAL_GetTick()>print_ts)
		{
			print_float(align_offset);
			print_float(gl_error_print);
			print_float(theta_m);
			//should be able to throw in position too
			print_ts = HAL_GetTick()+10;
		}
	}

	/*First, check to make sure you passed all edge condition flags*/
	//	if(dir_correct_flag == 0)
	//	{
	//		gl_error_print = OUT_OF_BOUNDS_ERROR;
	//		return gl_error_print;	//if direction mismatch, return an out of bounds error.
	//	}

	float final_pos = unwrap(theta_abs_rad(), &theta_m_prev)*.5;
	float err = final_pos - init_pos;
	if(err < 0)
		err = -err;
	gl_error_print = err;
	return err;
}


/*
 * New idea:
 * To eliminate the initial jerk behaviour and improve stability, sweep
 */

//static uint32_t check_encoder_region_2_timestamp = 0;
uint8_t check_encoder_region_2(uint32_t track_time, uint32_t timeout)
{
	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
	uint32_t check_encoder_region_2_timestamp = HAL_GetTick() + timeout;

	uint32_t err_check_ts = HAL_GetTick() + 100;
	float abs_err_prev = 0.0f;
	float diff_err = 0.0f;
	float setpoint = unwrap(theta_abs_rad(), &theta_m_prev)*.5;
	float dq = 0;
	float q_prev = 0;
	float dq_prev = 0;

	//float sum_abs_err = 0.0f;
	float setpoint_change_rate = 0.005f;
	const float speed_thresh = setpoint_change_rate*1000.0f + 20.0f;

	uint32_t try_ts = 0;	//block execution of the fix block for now.
	uint32_t print_ts = 0;
	uint32_t fail_ts = HAL_GetTick();
	while(1)
	{
		float theta_m = unwrap(theta_abs_rad(), &theta_m_prev)*.5;	//get current motor position

		/*Position Control*/
		float err = setpoint - theta_m;
		float tau = 30.0f*err;				//position control
		if(tau > 50)
			tau = 50;
		if(tau < -50)
			tau = -50;
		foc(tau,0);
		/*END*/

		float abs_err = abs_f(err);
		//calculate the change in error, speed, and update the setpoint in a periodic wrapper
		if(HAL_GetTick() > err_check_ts)
		{
			diff_err = abs_err - abs_err_prev;	//i.e. the change in the absolute value of the error
			abs_err_prev = abs_err;

			dq = (theta_m - q_prev)*1000.0f;	//speed
			dq = (dq+dq_prev)*.5;
			dq_prev=dq;
			q_prev = theta_m;

			setpoint+=setpoint_change_rate;	//update setpoint

			err_check_ts = HAL_GetTick();
		}


		float abs_dq = abs_f(dq); //precompute the abs of the motor speed
		if(HAL_GetTick() >= try_ts)
		{
			if(abs_err > 0.6f && abs_dq > speed_thresh && diff_err > .03f)	//if you are rapidly diverging
			{
				theta_enc_prev -= TWO_PI;	//put the encoder in the correct region
				setpoint = theta_m;	//reset tracking so there are no violent pops
				try_ts = HAL_GetTick()+5;	//wait a couple ms
				fail_ts = HAL_GetTick();
			}

			if (abs_err > 2.0f && abs_dq <= speed_thresh) //if your speed is very low but your error is high, you got stuck on something.
			{
				setpoint = theta_m;
				setpoint_change_rate = -setpoint_change_rate;
				try_ts = HAL_GetTick()+1000;	//wait a couple ms
				fail_ts = HAL_GetTick();
			}
		}

		if(HAL_GetTick()-fail_ts > track_time)
		{
			if(abs_err < 0.4f)
				return CAL_SUCCESS;

		}


		if(HAL_GetTick() >= print_ts)
		{
			print_float(abs_err);
			print_float(abs_f(dq));
			print_float(diff_err);

			print_ts = HAL_GetTick() + 9;
		}

		if(HAL_GetTick() > check_encoder_region_2_timestamp)
			return CAL_ERR_TIMEOUT;
	}
	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
}



/*
 * sweeps from the neg to pos candidate limits with check_align_offset. finds
 * the align offset which produces the smallest absolute value error within bounds,
 *  error being difference in position endpoints of a triangle waveform. i.e. you select
 *  the offset which produces the same torque for positive and negative iq references.
 *
 *  The negative and positive limits can be obtained from obtain_align_offset (misnomer, and should
 *  be named). This function is not guaranteed to produce valid limits, so if you have an uncooperative
 *  motor, try from -PI/elec_conv_ratio to ZERO. this method also should be faster for motors with high
 *  pole pair counts, such as the MS-4822
 *
 *
 */
float sweep_align(float align_neg, float align_pos)
{
	theta_m_prev = 0;
	float max_err = 0;
	float min_err = 1000;
	float align_offset_of_min_error = align_offset;
	for(align_offset = align_neg; align_offset < align_pos; align_offset += .005)
	{
		float err = check_align_offset(400, 15);
		if(err < .3)
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
		else
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
		if(err < min_err)
		{
			min_err = err;
			if(err < .5)	//if you're within range, new candidate for best align offset
				align_offset_of_min_error = align_offset;
		}
		if(err > max_err)
			err = max_err;
	}
	return align_offset_of_min_error;
}



void manual_align_calib()
{
	while(1)
	{
		float err = check_align_offset(500, 10);
		if(err < .25)
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
		else
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
	}
}


void test_foc()
{
	uint32_t reverse_ts = 0;
	float iq_ref = 15;
	uint32_t print_ts = 0;
	float th_m_prev = 0;
	float th_tmp = 0;	//unwrap smashes unwrap.
	while(1)
	{

		float theta_m = unwrap(theta_abs_rad(), &th_m_prev)*.5;

		if( (theta_m - th_tmp > 0 && iq_ref > 0) || (theta_m - th_tmp < 0 && iq_ref < 0))
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
		else
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
		th_tmp = theta_m;


		if(HAL_GetTick()>print_ts)
		{
			float t = (float)HAL_GetTick()*.001f;
			//			theta_m_previous = theta_m;
			print_float(theta_m);
			//			print_float(v_m);
			print_float(t);
			//			print_float(theta_m-PI);
			//			print_float(align_offset);
			print_ts = HAL_GetTick()+10;
		}

		if(HAL_GetTick() > reverse_ts)
		{
			reverse_ts = HAL_GetTick()+2000;
			iq_ref = -iq_ref;
		}
		foc(iq_ref,0);
	}
}

enum {VERIFY_FAILED, VERIFY_PASSED};
void force_encoder_region()
{
	uint8_t retc = VERIFY_FAILED;
	while(retc != VERIFY_PASSED)
	{
		retc = verify_encoder_region(30.0f, 200.0f, 2, 500);
		if(retc == VERIFY_FAILED)
			theta_enc_prev -= TWO_PI;	//put the encoder in the correct region
	}
}

uint8_t verify_encoder_region(float tau, float diff_thresh, int pass_count_thresh, uint32_t timeout)
{
	float theta_m = unwrap(theta_abs_rad(), &theta_m_prev)*.5f;	//get current motor position
	uint32_t tau_switch_ts = HAL_GetTick()+200;
	float pos_prev = theta_m;
	int pass_count = 0;
	while(1)
	{
		theta_m = unwrap(theta_abs_rad(), &theta_m_prev)*.5f;	//get current motor position
		foc(tau,0);

		float diff = theta_m - pos_prev;
		if(abs_f(diff) >= diff_thresh)
		{
			pos_prev = theta_m;
			if((tau > 0 && diff > diff_thresh) || (tau < 0 && diff < -diff_thresh))
			{
				pass_count++;
				HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
			}
			else
			{
				pass_count--;
				HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
			}
			tau = -tau;
			tau_switch_ts = HAL_GetTick();	//TODO: instead of time, move the rotor so the change in position is at least diff_thresh
		}
		if((HAL_GetTick() - tau_switch_ts) > timeout)
		{
			tau=-tau;
			pos_prev = theta_m;
			tau_switch_ts = HAL_GetTick();
		}

		if(pass_count >= pass_count_thresh)
			return VERIFY_PASSED;
		else if (pass_count <= -pass_count_thresh)
			return VERIFY_FAILED;

	}

}
