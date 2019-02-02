
#include "main.h"
#include "init.h"
#include "adc.h"
#include "foc_commutation.h"
#include "mag-encoder.h"
#include "i2c.h"
#include <string.h>
#include "settings.h"



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
 *New align offset procedure:
 *	-plot UART for a V/hz sinusoidal sweep, to obtain the dc offset (function exists).
 *	-obtain the min and max with the above method. this is the range across which you align offset will probably sit
 *	-sweep the min thru the max align offset, with FOC, back and forth. Plot iq over UART!!! Whichever align offset produces
 *		the closest match for iq, is the selected offset.
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


volatile float max_err;
volatile float min_err;
volatile float avg_err;

float obtain_align_offset()
{
	max_err = 0;
	min_err = 100000;
	avg_err = 0;


	float i_alpha,i_beta;
	uint32_t tA,tB,tC;
	float sin_theta = 0;
	float cos_theta = 1;
	inverse_park_transform(.6, 0, sin_theta, cos_theta, &i_alpha, &i_beta);	//maybe call theta rel again?
	svm(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);
	TIMER_UPDATE_DUTY(tA,tB,tC);
	HAL_Delay(1000);	//lock and settle

	float theta_elec;
	uint32_t num_samples = 0;
	uint32_t print_ts = 0;
	for(theta_elec = 0; theta_elec > -TWO_PI*2*elec_conv_ratio; theta_elec-=0.002)
	{
		num_samples++;
		float theta = fmod_2pi(theta_elec + PI) - PI;		//re-modulate theta_m. ensure that the angle is constrained from -pi to pi!!
		sin_theta = sin_fast(theta);				//calculate the sin of the electrical (magnetic flux) angle
		cos_theta = cos_fast(theta);				//and the cosine for park and inverse park domains
		inverse_park_transform(.6, 0, sin_theta, cos_theta, &i_alpha, &i_beta);	//maybe call theta rel again?
		svm(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);
		TIMER_UPDATE_DUTY(tA,tB,tC);

		float theta_enc = unwrap( theta_abs_rad(), &theta_enc_prev)*elec_conv_ratio;

		float err = theta_elec-theta_enc;
		avg_err += err;

		if(err < 0)
			err = -err;
		if(err > max_err)
			max_err = err;
		if(err < min_err)
			min_err = err;


		if(HAL_GetTick()>print_ts)
		{
			print_float(theta_elec);
			print_float(theta_enc);
			print_ts = HAL_GetTick()+1;
		}

		//HAL_Delay(5);
	}
	HAL_Delay(500);
	for(theta_elec = -TWO_PI*2*elec_conv_ratio; theta_elec < 0; theta_elec+=0.002)
	{
		num_samples++;
		float theta = fmod_2pi(theta_elec + PI) - PI;		//re-modulate theta_m. ensure that the angle is constrained from -pi to pi!!
		sin_theta = sin_fast(theta);				//calculate the sin of the electrical (magnetic flux) angle
		cos_theta = cos_fast(theta);				//and the cosine for park and inverse park domains
		inverse_park_transform(.6, 0, sin_theta, cos_theta, &i_alpha, &i_beta);	//maybe call theta rel again?
		svm(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);
		TIMER_UPDATE_DUTY(tA,tB,tC);

		float theta_enc = unwrap( theta_abs_rad(), &theta_enc_prev)*elec_conv_ratio;

		float err = theta_elec-theta_enc;
		avg_err += err;


		if(err < 0)
			err = -err;
		if(err > max_err)
			max_err = err;
		if(err < min_err)
			min_err = err;

		if(HAL_GetTick()>print_ts)
		{
			print_float(theta_elec);
			print_float(theta_enc);
			print_ts = HAL_GetTick()+1;
		}
	}
	avg_err = avg_err / ((float)(num_samples));
	return avg_err;
	//average error is in ELECTRICAL units!!!
}

void check_encoder_region();
void align_offset_test();


void test_foc()
{
	uint32_t reverse_ts = 0;
	float iq = 15;
	uint32_t print_ts = 0;
	float th_m_prev = 0;
	while(1)
	{
		float theta_m = unwrap(theta_abs_rad(), &th_m_prev)*.5;
		if(HAL_GetTick()>print_ts)
		{
			print_float(theta_m);
			print_float(theta_m-PI);
			print_ts = HAL_GetTick()+1;
		}

		if(HAL_GetTick() > reverse_ts)
		{
			reverse_ts = HAL_GetTick()+2000;
			iq = -iq;
		}
		foc(iq,0);
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
	TIM1->CCR4 = 950;	//for 7_5, you have about 8uS of sampling.you want to catch the current waveform right at the middle

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
	//	obtain_encoder_offset();
	align_offset_test();
#endif

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
void check_encoder_region()
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
