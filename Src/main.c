
#include "main.h"
#include "init.h"
#include "adc.h"
#include "foc_commutation.h"
#include "mag-encoder.h"
#include "i2c.h"
#include <string.h>
#include "settings.h"

/*
 * V3 R1 uses STM32F302C8T6
 * This version has, but currently does not use, an external oscillator. Future versions will likely use the STM32F301
 */

#define BRAKE 0
#define STOP 1
#define FORWARD_OPEN 2
#define FORWARD_CLOSED 3
#define BACKWARD_OPEN 4
#define BACKWARD_CLOSED 5


#define MAX_CUR 60.0


/*TODO:
 *
 *I2C gauntlet!!!
 *
 */

/*
 * Since I don't have a .README yet... i'll just put the revision notes up here
 *
 *Version change. This is now v3 hardware.
 *since we are using F302, uart is now compatible with SPI pins. We're taking advantage of this.
 *I also wrote a brushed control, for shits and goggles.
 */


void brushed_tau(float tau)
{
	if(tau > 1)
		tau = 1;
	if(tau < -1)
		tau = -1;
	uint32_t offset = (int32_t)(tau*500.0f);
	TIMER_UPDATE_DUTY(500+offset,500,500+offset);
}

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
 * obtain the approximate location of the alignment offset
 */
float obtain_align_offset(float * min_err, float * max_err)
{

	*max_err = 0;
	*min_err = 100000;
	float avg_err = 0;


	float i_alpha,i_beta;
	uint32_t tA,tB,tC;
	float sin_theta = 0;
	float cos_theta = 1;
	inverse_park_transform(.15, 0, sin_theta, cos_theta, &i_alpha, &i_beta);	//maybe call theta rel again?
	svm(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);
	TIMER_UPDATE_DUTY(tA,tB,tC);
	HAL_Delay(1000);	//lock and settle
	float theta_elec=0;
	float theta_enc = unwrap( theta_abs_rad(), &theta_enc_prev)*elec_conv_ratio;
	while(!(theta_enc < .05 && theta_enc > -.05))
	{
		theta_enc = unwrap( theta_abs_rad(), &theta_enc_prev)*elec_conv_ratio;

		float theta = fmod_2pi(theta_elec + PI) - PI;		//re-modulate theta_m. ensure that the angle is constrained from -pi to pi!!
		sin_theta = sin_fast(theta);				//calculate the sin of the electrical (magnetic flux) angle
		cos_theta = cos_fast(theta);				//and the cosine for park and inverse park domains
		inverse_park_transform(enc_mid_voltage_amp, 0, sin_theta, cos_theta, &i_alpha, &i_beta);	//maybe call theta rel again?
		svm(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);
		TIMER_UPDATE_DUTY(tA,tB,tC);

		theta_elec += .003;
	}

	uint32_t num_samples = 0;
	uint32_t print_ts = 0;
	for(theta_elec = 0; theta_elec > -TWO_PI*3*elec_conv_ratio; theta_elec-=0.003)
	{
		num_samples++;
		float theta = fmod_2pi(theta_elec + PI) - PI;		//re-modulate theta_m. ensure that the angle is constrained from -pi to pi!!
		sin_theta = sin_fast(theta);				//calculate the sin of the electrical (magnetic flux) angle
		cos_theta = cos_fast(theta);				//and the cosine for park and inverse park domains
		inverse_park_transform(enc_mid_voltage_amp, 0, sin_theta, cos_theta, &i_alpha, &i_beta);	//maybe call theta rel again?
		svm(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);
		TIMER_UPDATE_DUTY(tA,tB,tC);

		theta_enc = unwrap( theta_abs_rad(), &theta_enc_prev)*elec_conv_ratio;

		float err = theta_elec-theta_enc;
		avg_err += err;

		if(err < 0)
			err = -err;
		if(err > *max_err)
			*max_err = err;
		if(err < *min_err)
			*min_err = err;


		if(HAL_GetTick()>print_ts)
		{
			print_float(theta_elec);
			print_float(theta_enc);
			print_ts = HAL_GetTick()+1;
		}

		//HAL_Delay(5);
	}
	HAL_Delay(500);
	for(theta_elec = -TWO_PI*3*elec_conv_ratio; theta_elec < 0; theta_elec+=0.003)
	{
		num_samples++;
		float theta = fmod_2pi(theta_elec + PI) - PI;		//re-modulate theta_m. ensure that the angle is constrained from -pi to pi!!
		sin_theta = sin_fast(theta);				//calculate the sin of the electrical (magnetic flux) angle
		cos_theta = cos_fast(theta);				//and the cosine for park and inverse park domains
		inverse_park_transform(enc_mid_voltage_amp, 0, sin_theta, cos_theta, &i_alpha, &i_beta);	//maybe call theta rel again?
		svm(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);
		TIMER_UPDATE_DUTY(tA,tB,tC);

		theta_enc = unwrap( theta_abs_rad(), &theta_enc_prev)*elec_conv_ratio;

		float err = theta_elec-theta_enc;
		avg_err += err;


		if(err < 0)
			err = -err;
		if(err > *max_err)
			*max_err = err;
		if(err < *min_err)
			*min_err = err;

		if(HAL_GetTick()>print_ts)
		{
			print_float(theta_elec);
			print_float(theta_enc);
			print_ts = HAL_GetTick()+1;
		}
	}
	avg_err = avg_err / ((float)(num_samples));
	avg_err = avg_err / elec_conv_ratio;
	*max_err = *max_err / elec_conv_ratio;
	*min_err = *min_err / elec_conv_ratio;
	return avg_err;
	//average error is in ELECTRICAL units!!!
}

float check_encoder_region();
void align_offset_test();


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
			//			print_float(theta_m);
			//			print_float(theta_m-PI);
			print_float(align_offset);
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

int main(void)
{
	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_USART2_UART_Init();

	if(sizeof(float) != 4)
	{
		while(1)
		{
			print_str("ERROR! INVALID FLOAT SIZE\r\n");
			HAL_Delay(1000);
		}
	}
	//	print_str("\r\nHELLO WORLD I AM FOC\r\n");
	//	start_pwm();
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	TIMER_UPDATE_DUTY(0,0,0);
	TIM1->CCR4 = 650;	//for 7_5, you have about 8uS of sampling.you want to catch the current waveform right at the middle

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)dma_adc_raw, NUM_ADC);
	//	HAL_SPI_TransmitReceive_DMA(&hspi1, t_data, r_data,2);	//think need to change DMA settings to word from byte or half word

	HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);

	HAL_Delay(100);
	HAL_GPIO_WritePin(CAL_PORT, CAL_PIN, 1);
	HAL_Delay(100);
	HAL_GPIO_WritePin(CAL_PORT, CAL_PIN, 0);
	HAL_Delay(100);

	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
	get_current_cal_offsets();
	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);

	int led_ts = HAL_GetTick()+100;
	int led_state = 1;

	//	align_offset = obtain_align_offset();
	//	TIMER_UPDATE_DUTY(500,500,500);
	//	while(1);

#ifdef GET_ENCODER_MIDPOINT
	obtain_encoder_midpoints();
#endif
#ifdef GET_INIT_OFFSET
	/*	Depricated rubbish*/
	//	obtain_encoder_offset();
	//	align_offset_test();

	/*Method one*/
//	float min_align_offset, max_align_offset;
//	obtain_align_offset(&min_align_offset, &max_align_offset );
//	align_offset = sweep_align(min_align_offset, max_align_offset);	//might need to obtain range from obtain_align_offset

	/*Method two*/
	align_offset = sweep_align(-PI/elec_conv_ratio,0);

#endif

//	manual_align_calib();//-0.236198962

	TIMER_UPDATE_DUTY(0,0,0);

	float x_iq_PI = 0;	//torque pi state
	float x_id_PI = 0;	//flux pi state
	float uq = 0;
	float ud = 0;

	float iqref = 0;
	float idref = 0;

	/*
	 * NOTE:
	 * The new encoder needs to be unwrapped, divided by two, and re-wrapped
	 * in order to have correct FOC commutation.
	 *
	 * Because the encoder returns TWICE the mechanical angle, not the actual mechanical
	 * angle, HALF of the mechanical angle has the potential to be incorrect. I.e.
	 * if mechanically the motor is advanced by PI, the ENCODER returns the SAME value.
	 *
	 * This is accounted for by unwrapping when the motor is on, but when the motor is OFF
	 * this goes away. THEREFORE either the motor needs to be mechanically restricted so that
	 * it cannot be easily initialized in the wrong region (direct drive only), OR a test
	 * procedure needs to be developed so that the angle is corrected for if it is intialized
	 * in the wrong region.
	 */
	//	TIMER_UPDATE_DUTY(0,0,0);
	//	while(1)
	//	{
	//		print_float(1);
	//		print_float(-1);
	//		HAL_Delay(10);
	//	}
	theta_m_prev = 0;

#ifdef TEST_FOC
	test_foc();
#endif

	check_encoder_region();
	theta_m_prev = theta_enc_prev;


	while(1)
	{
		handle_i2c(&hi2c1);


		//		uint32_t data_in = ((i2c_rx_data[0] << 24) | (i2c_rx_data[1] << 16) | (i2c_rx_data[2] << 8) | (i2c_rx_data[3]));
		//		float * f_addr = (float *)(&data_in);
		//		iqref = *f_addr;
		float * f_addr = (float *)(i2c_rx_data);
		iqref = *f_addr;

		if(iqref > MAX_CUR)
			iqref = MAX_CUR;
		else if (iqref < -MAX_CUR)
			iqref = -MAX_CUR;
		if(HAL_GetTick() > recieved_data_ts + 100)
			iqref = 0;

		/****************************************FOC functional block*************************************/
		float i_a,i_b,i_c;
		conv_raw_current(&i_a,&i_b, &i_c);

		float i_alpha,i_beta;
		clarke_transform(i_a,i_b,i_c,&i_alpha, &i_beta);

		//		float theta_enc = unwrap( theta_rel_rad(), &theta_enc_prev);
		float theta_enc = unwrap(theta_rel_rad(), &theta_enc_prev);
		float theta_elec = theta_enc*elec_conv_ratio;		//test this! MS motor has 22 pole pairs, which means multiply by 11. ENCODER RETURNS 2*theta so we absorb .5 into this term
		theta_elec = fmod_2pi(theta_elec + PI) - PI;		//re-modulate theta_m. ensure that the angle is constrained from -pi to pi!!
		float sin_theta = sin_fast(theta_elec);				//calculate the sin of the electrical (magnetic flux) angle
		float cos_theta = cos_fast(theta_elec);				//and the cosine for park and inverse park domains

		float i_q, i_d;
		park_transform(i_alpha, i_beta, sin_theta,cos_theta, &i_q, &i_d);

		controller_PI(iqref, i_q, iq_gain, 0.000000001, &x_iq_PI, &uq);		//this sort of works
		controller_PI(idref, i_d, id_gain, 0.0000000001, &x_id_PI, &ud);		//high current


		uint32_t tA,tB,tC;
		inverse_park_transform(uq, ud, sin_theta,cos_theta, &i_alpha, &i_beta);	//maybe call theta rel again?sin_theta
		svm(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);
		TIMER_UPDATE_DUTY(tA,tB,tC);
		/**********************************************END**********************************************/

		float theta_m = unwrap(theta_abs_rad(), &theta_m_prev)*.5;
		//		uint32_t * d_addr = (uint32_t *)(&theta_m);
		//		uint32_t tx_data = *d_addr;
		//		i2c_tx_data[0] = (tx_data & 0xFF000000) >> 24;
		//		i2c_tx_data[1] = (tx_data & 0x00FF0000) >> 16;
		//		i2c_tx_data[2] = (tx_data & 0x0000FF00) >> 8;
		//		i2c_tx_data[3] = (tx_data & 0x000000FF);
		uint8_t * ptr = (uint8_t *)(&theta_m);
		i2c_tx_data[0] = ptr[0];	//i did this to keep i2c_tx_data in the same place
		i2c_tx_data[1] = ptr[1];	//however, just changing it's base pointer
		i2c_tx_data[2] = ptr[2];	//is probably totally fine
		i2c_tx_data[3] = ptr[3];

		//		HAL_UART_Transmit(&huart2, i2c_tx_data, 4, 10);

		if(HAL_GetTick()>=led_ts)
		{
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,led_state);
			led_state = !led_state & 1;
			led_ts = HAL_GetTick() + 200;
		}
	}

}

/*
 * use the light to dowse for an angle.
 */
void find_raw_encoder_angle(float angle, uint32_t timeout)
{
	uint32_t manual_calib_ts = HAL_GetTick();
	while(1)
	{
		if(HAL_GetTick() > manual_calib_ts + timeout)
			return;
		float theta_enc = theta_abs_rad();
		if(theta_enc < angle + .025 && theta_enc > angle-.025)
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
		else
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
	}
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
float check_encoder_region()
{
	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
	uint32_t ts = HAL_GetTick();
	float theta_m_prev = 0;
	float theta_set = (unwrap(theta_abs_rad(), &theta_m_prev)*.5)+.4;	//get the initial position and use it to set the motor setpoint.
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

		if(err > .6 && HAL_GetTick() > ts)	//if the error is great and you haven't tried for some time, reverse the direction
		{
			theta_enc_prev -= TWO_PI;
			ts = HAL_GetTick()+try_ts;
			try_ts+=50;	//if you failed on the last attempt, try for just a liiitle bit longer. This improves stability (i.e. no infinite oscillations if you're significantly out of bounds)
		}

		if(err > .1)	//if the error is great,
			off_ts = HAL_GetTick();	//set the timestamp
		else
		{
			if(HAL_GetTick() - off_ts > 50)	//if you've settled (low error for an acceptable time) break
			{
				HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
				break;
			}
		}
	}
	return theta_enc_prev;
}




float ao_pos = 0;
float ao_neg = 0;
void align_offset_test()
{
	float lower_limit = -PI/elec_conv_ratio;
	float upper_limit = -lower_limit;
	uint32_t period = 0;
	float theta_m_prev = 0;
	float theta_unwrapped_prev = 0;
	float theta_m;
	int prev_rotation_num = 0;
	uint32_t rotation_start_ts = HAL_GetTick();
	uint32_t min_pos_period = 0xFFFFFFFF;
	uint32_t min_neg_period = 0xFFFFFFFF;
	for(align_offset = lower_limit; align_offset < upper_limit; align_offset+=.00002)
	{
		theta_m = unwrap(theta_abs_rad(), &theta_m_prev)*.5;
		float d_theta = theta_m - theta_unwrapped_prev;
		float wrapped_theta = theta_m * ONE_BY_TWO_PI;//= fmod_2pi(theta_m);
		int rotation_num = (int)wrapped_theta;
		if(rotation_num != prev_rotation_num)
		{
			period = HAL_GetTick()-rotation_start_ts;
			if(period < min_pos_period && d_theta > 0)
			{
				min_pos_period = period;
				ao_pos = align_offset;
			}
			if(period < min_neg_period && d_theta < 0)
			{
				min_neg_period = period;
				ao_neg = align_offset;
			}
		}
		if(d_theta > 0.003)
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
		else if(d_theta < -.003)
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);

		foc(10,0);
		prev_rotation_num = rotation_num;
		theta_unwrapped_prev = theta_m;
	}
	//	align_offset = ao_pos;
	align_offset = (ao_pos - ao_neg)*.5;
	TIMER_UPDATE_DUTY(0,0,0);
}

/*
 * 		old 'discrete' position controller (on driver)
 * 		//		theta_ref = .4*sin(time_seconds()*6+1.45)+HALF_PI;
		//		theta_ref = M_PI*.5 + sin(time_seconds()*6);
		//		theta_ref = 5*TWO_PI*sin(time_seconds()*3);

		//		theta_ref = .5*HALF_PI*sin((time_seconds()-tstart)*TWO_PI*2+1.57);
		//		theta_ref = .6*sin(time_seconds()*12);
		//		theta_ref = 0;
		//		float  err = theta_ref - theta_m;
		//		float dtheta = theta_m - p_theta;
		//		p_theta = theta_m;
		//		float dt = time_seconds()-tprev;
		//		tprev = time_seconds();
		//		iqref = 30*err - dtheta/dt*0.2;
		//		if(iqref > 40.0)
		//			iqref = 40.0;
		//		if(iqref < -40.0)
		//			iqref = -40.0;

		//		iqref = 5.0;
 *
 */
