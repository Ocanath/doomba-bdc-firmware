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
	m_uart_tx_start(&m_huart2, ptr, sizeof(float));
	while(m_huart2.bytes_to_send != 0);
}

/*cheap helper function to avoid needing to use sprintf...*/
int32_t int_to_str(int32_t arg, uint8_t* buf, int16_t buf_len)
{
	uint8_t flag = 0;
	if (arg < 0)
	{
		flag = 1;
		arg = -arg;
	}

	int32_t argcpy = arg;
	int32_t val_len = (flag == 0) ? 0 : 1;
	while (argcpy != 0)
	{
		argcpy /= 10;
		val_len++;
	}
	if (val_len > buf_len)
		return -1;

	int16_t i = val_len - 1;
	while (arg != 0)
	{
		if (i < 0 || i >= buf_len)
		{
			return -1;
		}
		buf[i--] = (arg % 10) + '0';
		arg /= 10;
	}
	if (flag && i >= 0 && i < buf_len)
	{
		buf[i--] = '-';
	}
	if (i == -1)		//gets subtracted on last character load
		return val_len;
	else
		return -1;
}

void print_str(const char * str)
{
	int strlen;
	for(strlen=0;str[strlen] != 0; strlen++);
	m_uart_tx_start(&m_huart2, (uint8_t*)str, strlen);
	while(m_huart2.bytes_to_send != 0);
}

void print_int_as_str(int32_t in)
{
	uint8_t istr[12] = {0};	//only 11 characters are needed actually
	int_to_str(in, istr, 12);
	print_str((const char *)istr);
}

void print_calibration_parameters(void)
{
	print_str("\033[2J");

	print_str("MODULE_NUMBER is: ");
	print_int_as_str((int32_t)MODULE_NUMBER);
	print_str("\r\n\r\n");

	print_str("#define ALIGN_OFFSET ");
	print_int_as_str(align_offset_fixed);
	print_str("\r\n");

	print_str("int32_t gl_cos_min = ");
	print_int_as_str(gl_cos_min);
	print_str(";\r\n");

	print_str("int32_t gl_cos_max = ");
	print_int_as_str(gl_cos_max);
	print_str(";\r\n");

	print_str("int32_t gl_sin_min = ");
	print_int_as_str(gl_sin_min);
	print_str(";\r\n");

	print_str("int32_t gl_sin_max = ");
	print_int_as_str(gl_sin_max);
	print_str(";\r\n");

	print_str("int32_t gl_oerr_total = ");
	print_int_as_str(gl_oerr_total);
	print_str(";\r\n");
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
	uint32_t uart_tx_ts = 0;
	while(time_seconds() - t_start < 15)
	{
		uint16_t adc_sin = dma_adc_raw[ADC_SIN_CHAN];
		uint16_t adc_cos = dma_adc_raw[ADC_COS_CHAN];

		if(adc_sin > s_max)
			s_max = adc_sin;
		if(adc_sin < s_min)
			s_min = adc_sin;

		if(adc_cos > c_max)
			c_max = adc_cos;
		if(adc_cos < c_min)
			c_min = adc_cos;


		float theta = 420*sin_fast( (fmod_2pi(.2*time_seconds() + PI) - PI) );
		theta = fmod_2pi(theta + PI) - PI;		//re-modulate theta_m. ensure that the angle is constrained from -pi to pi!!
		float sin_theta = sin_fast(theta);				//calculate the sin of the electrical (magnetic flux) angle
		float cos_theta = cos_fast(theta);				//and the cosine for park and inverse park domains
		float i_alpha,i_beta;
		uint32_t tA,tB,tC;
		inverse_park_transform(enc_mid_voltage_amp, 0, sin_theta, cos_theta, &i_alpha, &i_beta);	//maybe call theta rel again?
		svm(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);
		TIMER_UPDATE_DUTY(tA,tB,tC);



		uint32_t tick = HAL_GetTick();
		if(tick >= uart_tx_ts)
		{
			uart_tx_ts = tick + 10;

			int16_fmt_t fmt;
			int bidx = 0;
			uint8_t buf[8*sizeof(int16_t)] = {0};

			fmt.v = adc_sin;
			buffer_data(fmt.d, sizeof(int16_t), buf, &bidx);
			fmt.v = adc_cos;
			buffer_data(fmt.d, sizeof(int16_t), buf, &bidx);
			fmt.v = s_max;
			buffer_data(fmt.d, sizeof(int16_t), buf, &bidx);
			fmt.v = s_min;
			buffer_data(fmt.d, sizeof(int16_t), buf, &bidx);
			fmt.v = c_max;
			buffer_data(fmt.d, sizeof(int16_t), buf, &bidx);
			fmt.v = c_min;
			buffer_data(fmt.d, sizeof(int16_t), buf, &bidx);


			m_uart_tx_start(&m_huart2, buf, 6*sizeof(int16_t));	//HAL_UART_Transmit_IT(&huart2,buf,4*sizeof(float));
		}
	}
	TIMER_UPDATE_DUTY(0,0,0);
	//sin_mid = (s_max + s_min)/2;
	//cos_mid = (c_max + c_min)/2;
	gl_sin_max = s_max;
	gl_sin_min = s_min;
	gl_cos_max = c_max;
	gl_cos_min = c_min;
	eval_encoder_params();
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
	fixed_PI_params_t bkp_idctlp = id_pi;
	id_pi.kp.i32 = 0;
	id_pi.ki.i32 = 0;

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
	id_pi = bkp_idctlp;

	return abs_f(*run_distance/run_time_f);
}


/*Open loop commute/lock motor to a specific electrical angle
 * with specified vq intensity.
 *
 * Returns the amplitude, offset, and orthogonality error compensated angle.
 *
 * Helper function for computing encoder parameters (ignoring return value), or performing
 * coarse estimate of comm. offset for calibration and quick calibration
 * validation
 * */
int32_t elock_angle(int32_t vq, int32_t theta)
{
	theta = wrap_2pi_12b(theta);
	int32_t sth = sin_12b(theta);
	int32_t cth = cos_12b(theta);
	int32_t alpha,beta, ta,tb,tc;
	inverse_park_fixed(vq, 0, sth, cth, &alpha,&beta);
	svm_fixed(alpha,beta,TIM1->ARR,&ta,&tb,&tc);
	TIMER_UPDATE_DUTY(ta,tb,tc);
	return wrap_2pi_12b(theta_rel_fixed() * elec_conv_ratio_fixed);
}

/*
 * Lock rotor to a specified electrical position, wait for it to settle,
 *  and return the measured encoder angle
 * */
int32_t get_rotorlock_offsetdeviation(int32_t vq, int32_t theta, uint32_t init_settling_time_ms, int num_samples)
{
	elock_angle(vq, theta);	//initial angle lock
	HAL_Delay(init_settling_time_ms);	//settling time to prevent rotor oscillations

	int64_t sum = 0;
	for(int i = 0; i < num_samples; i++)
	{
		sum += (int64_t)(elock_angle(vq,theta));
		HAL_Delay(1);
	}
	sum = sum/((int64_t)num_samples);
	return (int32_t)sum;
}

/*open loop track an electrical angle. Helper/prog on a robust coarse align offset
 * check
 *
 * Why this works:
 *
 * When the motor open loop settles on a position, it SETTLES ON MAX ID position (ie
 * locks to the rotor position which is pi/2 radians away from the position which causes
 * the rotor to commute with maximum torque).
 *
 * This function tracks a desired ELECTRICAL angle given an align offset.
 *
 *
 * */
int32_t openloop_eang_pctl(int32_t ang, uint32_t exp_ts)
{
	float theta_e = 0;
	uint32_t uart_tx_ts = 0;
	int16_t passcount = 0;
	while(HAL_GetTick() < exp_ts)
	{
		elock_angle(400, (int32_t)(theta_e*4096.f));
		int32_t theta_m = wrap_2pi_12b(theta_rel_fixed()*elec_conv_ratio);

		float error = ((float)(wrap_2pi_12b(ang - theta_m)))/4096.f;
		float effort = error/500.f;
		if(effort > .001f)
			effort = .001f;
		if(effort < -.001f)
			effort = -.001f;
		theta_e += effort;

		int32_t aoff_error_coarse = wrap_2pi_12b((int32_t)(theta_e*4096.f));
		aoff_error_coarse = wrap_2pi_12b(aoff_error_coarse + HALF_PI_12B);	//settles on (+/-)pi/2 away, so gotta comp

		uint32_t tick = HAL_GetTick();
		if(tick >= uart_tx_ts)
		{
			uart_tx_ts = tick + 10;

			if(effort > -4e-5f && effort < 4e-5f)
			{
				passcount++;
			}
			else
			{
				passcount = 0;
			}
			if(passcount > 100)
			{
				return aoff_error_coarse;
			}
		}
	}
	return 0;//never happens
}

/*
 *Automatically find the encoder offset
 */
void auto_calibrate_motor(void)
{

	TIMER_UPDATE_DUTY(0,0,0);
	//	align_offset = -PI/elec_conv_ratio;
	int passcount = 0;

//	/*open loop lock angle for a coarse estimate of the align offset*/
//	align_offset = 0;	//first, zero out align offset for relative accuracy of this method
//	align_offset_fixed = 0;	//important: must clear for rotorlock to return correct value
//	int32_t lock_angle = 0;	//experiment with nonzero lock angles?
//	int32_t res = get_rotorlock_offsetdeviation(500, lock_angle, 500, 300);
//	res = wrap_2pi_12b(res - lock_angle);
//	res = wrap_2pi_12b(res + HALF_PI_12B);
//	align_offset = ((float)res)/4096.f;
//	align_offset_fixed = (int32_t)(align_offset*4096.f);

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


