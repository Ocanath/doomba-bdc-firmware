#include "main.h"
#include "stm32f0xx_hal.h"
#include "adc.h"
#include "delay_uS.h"
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


/*
 * Since I don't have a .README yet... i'll just put the revision notes up here
 *
 * CHanges:
 * added some preprocessor directives that make it easier to switch between motor drivers
 * changed to i2c fast mode
 * used pointer logic instead of memcpy
 *
 * NOTES:
 * there is a serious i2c bug which occurs when more than 1 device is on the bus at once.
 * THIS HAS BEEN VERY CHALLENGING TO DEBUG.
 * if there is only 1 device, no problems. as soon as 2 are juggled at once (on the nrf middleman) it has this 'popping' effect where the motors... pop and jitter
 *
 *
 * THINGS TO TRY:
 * try on F3, since blocking doesn't work on the f0 middleman and it did work on the f3
 * try using floating point divide instead of copying and reformatting a whole float.
 * if all else fails, try SPI
 *
 */


/*
char msg_buf[32];
void print_string(char * c)
{
	int i;
	for(i=0; c[i] != 0; i++);
	HAL_UART_Transmit(&huart1, (uint8_t*)c, i, 100);
}
void print_int16(int16_t val)
{
	msg_buf[0] = (val & 0x00FF);
	msg_buf[1] = (val & 0xFF00)>>8;
	HAL_UART_Transmit(&huart1, (uint8_t*)msg_buf, 2, 100);
}
void print_int32(int32_t val)
{
	msg_buf[0] = (val & 0x00FF);
	msg_buf[1] = (val & 0xFF00)>>8;
	msg_buf[2] = (val & 0xFF0000)>>16;
	msg_buf[3] = (val & 0xFF000000)>>24;
	HAL_UART_Transmit(&huart1, (uint8_t*)msg_buf, 4, 100);
}
 */
void check_encoder_region();

typedef enum {ADVANCE_TO_ZERO = 0, REST_AT_ZERO = 1, ADVANCE_TO_ANGLE = 2, REST_AT_ANGLE = 3} bang_test_state;

int main(void)
{
	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC_Init();
	MX_I2C1_Init();
	//	MX_SPI1_Init();
	//	MX_USART1_UART_Init();
	MX_TIM1_Init();
	MX_TIM14_Init();

	HAL_TIM_Base_Start(&htim14);
	HAL_TIM_PWM_Start_IT(&htim14, TIM_CHANNEL_1);

//	start_pwm();
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	TIMER_UPDATE_DUTY(0,0,0);
	TIM1->CCR4 = 650;	//for 7_5, you have about 8uS of sampling.you want to catch the current waveform right at the middle

	HAL_ADC_Start_DMA(&hadc, (uint32_t *)dma_adc_raw, NUM_ADC);
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


#ifdef GET_ENCODER_MIDPOINT
	obtain_encoder_midpoints();
#endif
#ifdef GET_INIT_OFFSET
	obtain_encoder_offset();
#endif

	TIMER_UPDATE_DUTY(0,0,0);

	float x_iq_PI = 0;	//torque pi state
	float x_id_PI = 0;	//flux pi state
	float uq = 0;
	float ud = 0;

	float iqref = 0;
	float idref = 0;

	float theta_m_prev = 0;
	theta_enc_prev = theta_abs_rad();

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
	check_encoder_region();

	while(1)
	{
		handle_i2c();
		uint32_t data_in = ((i2c_rx_data[0] << 24) | (i2c_rx_data[1] << 16) | (i2c_rx_data[2] << 8) | (i2c_rx_data[3]));

		float * f_addr = (float *)(&data_in);
		iqref = *f_addr;

		if(iqref > MAX_CUR)
			iqref = MAX_CUR;
		else if (iqref < -MAX_CUR)
			iqref = -MAX_CUR;
		if(HAL_GetTick() > recieved_data_ts + 100)
			iqref = 0;

//		iqref = 10;

		/****************************************FOC functional block*************************************/
		float i_a,i_b,i_c;
		conv_raw_current(&i_a,&i_b, &i_c);

		float i_alpha,i_beta;
		clarke_transform(i_a,i_b,i_c,&i_alpha, &i_beta);

		float theta_enc = unwrap( theta_rel_rad(), &theta_enc_prev);
		float theta_elec = theta_enc*num_pole_pairs;		//test this! MS motor has 22 pole pairs, which means multiply by 11. ENCODER RETURNS 2*theta so we absorb .5 into this term
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
		uint32_t * d_addr = (uint32_t *)(&theta_m);
		uint32_t tx_data = *d_addr;
		i2c_tx_data[0] = (tx_data & 0xFF000000) >> 24;
		i2c_tx_data[1] = (tx_data & 0x00FF0000) >> 16;
		i2c_tx_data[2] = (tx_data & 0x0000FF00) >> 8;
		i2c_tx_data[3] = (tx_data & 0x000000FF);


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
 * twice the actual angle
 */
void check_encoder_region()
{
	uint32_t ts = HAL_GetTick();
	theta_enc_prev = theta_abs_rad();
	float theta_enc_start = unwrap( theta_rel_rad(), &theta_enc_prev)*.5;	//on your marks...
	while(HAL_GetTick() < ts + 10)
	{
		foc(12,0);
	}
	float theta_enc_end = unwrap( theta_rel_rad(), &theta_enc_prev)*.5;
	if(theta_enc_end - theta_enc_start < 0)
		align_offset = -align_offset;	//same as add pi. this value relates the electrical angle to the encoder angle
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


