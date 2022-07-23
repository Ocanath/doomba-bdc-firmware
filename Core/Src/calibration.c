/*
 * calibration.c
 *
 *  Created on: Apr 29, 2019
 *      Author: Ocanath
 */
#include "calibration.h"
#include "mag-encoder.h"
#include "foc_commutation.h"
#include "foc_fixed.h"
#include "CAN.h"
#include "calibration_fixed.h"
#include "m_uart.h"

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
 * Ramp up until steady state speed is high
 * Record the beginning and end of the run after SS speed is hit
 * return the average speed
 */
float run_v(int32_t vq, uint32_t run_time, float * run_distance)
{
	/*Interesting discovery:
	 * Phase advancement prevents this from working basically entirely. It causes
	 * the controller to converge on weird local minima.
	 *
	 * Removing it leads to very improved stability of this technique. This should not come as a much of a surprise,
	 * considering the fact that it is adding a first-order bias (since the advancement term is a function of
	 * the motor speed) to the controller, which is likely big enough that the simple 'P'
	 * dynamics is not sufficient to stabilize.
	 * */
	int32_t bkp_interval = gl_prop_delayloop_interval;	//backup before clobber
	int32_t bkp_div = gl_prop_delay_const_12b;
	gl_prop_delayloop_interval = 0x7FFFFFFF;	//turn off phase advancement by clobbering these constants
	gl_prop_delay_const_12b = 0x7FFFFFFF;


	for(uint32_t stop_ts = HAL_GetTick() + 100; HAL_GetTick()<stop_ts;) //spin up to reach steady state velocity
		sinusoidal_fixed(vq);
	float run_start = ((float)gl_theta_rem_m)/4096.f;
	for(uint32_t stop_ts = HAL_GetTick() + run_time; HAL_GetTick()<stop_ts;)
		sinusoidal_fixed(vq);
	float run_end = ((float)gl_theta_rem_m)/4096.f;
	TIMER_UPDATE_DUTY(0,0,0);
	HAL_Delay(50);
	*run_distance = run_end-run_start;
	float run_time_f = (float)run_time;//stay in ms, to keep our gain small and avoid floating point errors

	gl_prop_delayloop_interval = bkp_interval;
	gl_prop_delay_const_12b = bkp_div;	//restore phase advancement before and exit}

	return abs_f(*run_distance/run_time_f);
}

/*
 *Automatically find the encoder offset
 */
void auto_calibrate_motor(void)
{

	TIMER_UPDATE_DUTY(0,0,0);
	//	align_offset = -PI/elec_conv_ratio;
	int passcount = 0;
	while(1)
	{
		int32_t vq = 2048;	//voltage percent out of sqrt(3)/2*4096 (applied to supply voltage)
		float run_dist_1,run_dist_2;
		float run1_speed = run_v(vq,400, &run_dist_1);	//returns absolute value of the run speed, with the total distance of the run output to the dist var
		float run2_speed = run_v(-vq,400, &run_dist_2);
		TIMER_UPDATE_DUTY(500,500,500);

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
		align_offset_fixed = (int32_t)(align_offset*4096.f);


		if(abs_f(u) < 0.00005f && ( (abs_f(run_dist_1) + abs_f(run_dist_2)) > 1.f))	//if the run speed difference is very small, it's a passable align offset
		{
			passcount++;
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);	//count the number of passes
		}
		else
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
		if(passcount > 10)
			break;	//if you've passed over 10 times, you're done!



		{

			floatsend_t fmt;
			int bidx = 0;
			uint8_t buf[8*sizeof(float)] = {0};

			fmt.v = (float)u;
			buffer_data(fmt.d, sizeof(float), buf, &bidx);
			fmt.v = (float)align_offset_fixed;
			buffer_data(fmt.d, sizeof(float), buf, &bidx);

			m_uart_tx_start(&m_huart2, buf, 2*sizeof(float));	//HAL_UART_Transmit_IT(&huart2,buf,4*sizeof(float));
		}

		TIMER_UPDATE_DUTY(0,0,0);
		HAL_Delay(1);
	}
	align_offset_fixed = (int32_t)(align_offset * 4096.f);

}



void test_foc()
{

	//	uint32_t reverse_ts = 0;
	//	float iq_ref = 70.f;
	//	float vref = .8f;
	int32_t vref = MAX_VQ;
	for(int i = 0; i < 20; i ++)
	{
		HAL_GPIO_TogglePin(STAT_PORT,STAT_PIN);
		HAL_Delay(100);
	}
	while(1)
	{
		for(uint32_t run_ts = HAL_GetTick()+2000; HAL_GetTick()< run_ts;)
			sinusoidal_fixed(vref);
		//foc(iq_ref,0);//sinusoidal(.8f,0);

		TIMER_UPDATE_DUTY(0,0,0);
		HAL_Delay(100);

		for(uint32_t run_ts = HAL_GetTick()+2000; HAL_GetTick()< run_ts;)
			sinusoidal_fixed(-vref);//foc(iq_ref,0);
		//sinusoidal(-.8f,0);

		TIMER_UPDATE_DUTY(0,0,0);
		HAL_Delay(100);

	}
}

enum {VERIFY_FAILED, VERIFY_PASSED, VERIFY_TIMEOUT};

uint8_t verify_encoder_region(float tau, float diff_thresh, int pass_count_thresh, uint32_t timeout)
{
	float theta_m = unwrap(theta_abs_rad(), &theta_m_prev)*.5f;	//get current motor position
	uint32_t tau_switch_ts = HAL_GetTick();
	float pos_prev = theta_m;
	int pass_count = 0;
	while(1)
	{
		theta_m = unwrap(theta_abs_rad(), &theta_m_prev)*.5f;	//get current motor position
		//		i2c_tx_data.v = theta_m;
		foc(tau,0);

		float diff = theta_m - pos_prev;
		if(abs_f(diff) >= diff_thresh)
		{
			pos_prev = theta_m;
			if((tau > 0 && diff > diff_thresh) || (tau < 0 && diff < -diff_thresh))
			{
				pass_count++;
				HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
			}
			else
			{
				pass_count--;
				HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
			}
			tau = -tau;
			tau_switch_ts = HAL_GetTick();	//TODO: instead of time, move the rotor so the change in position is at least diff_thresh
		}
		if((HAL_GetTick() - tau_switch_ts) > timeout)
		{
			//			tau=-tau;
			//			pos_prev = theta_m;
			//			tau_switch_ts = HAL_GetTick();
			return VERIFY_TIMEOUT;
		}

		if(pass_count >= pass_count_thresh)
			return VERIFY_PASSED;
		else if (pass_count <= -pass_count_thresh)
			return VERIFY_FAILED;

		//		int rc = handle_i2c(&hi2c1);
		//		if(hi2c1.ErrorCode != 0 || rc != 0)
		//		{
		//			HAL_NVIC_ClearPendingIRQ(I2C1_EV_IRQn);				//and maybe doing this are critical for i2c_IT error recovery
		//			while(HAL_NVIC_GetPendingIRQ(I2C1_EV_IRQn)==1);
		//			I2C1_Reset();
		//			gl_i2c_mode = I2C_MODE_WAIT;
		//		}
	}

}

void force_encoder_region(void)
{
	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
	uint8_t retc = VERIFY_FAILED;
	float tau = 5.0f;
	while(retc != VERIFY_PASSED)
	{
		retc = verify_encoder_region(tau, 0.33f, 3, 500);
		if(retc == VERIFY_FAILED)
		{
			theta_enc_prev -= TWO_PI;	//put the encoder in the correct region
		}
		if(retc == VERIFY_TIMEOUT)
		{
			if(tau > 0)
			{
				tau+=5.0f;
				if(tau > 50.0f)
					tau = 50.0f;
			}
			if(tau < 0)
			{
				tau -= 5.0f;
				if(tau < -50.0f)
					tau = -50.0f;
			}
			tau = -tau;
		}
	}
	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
}


