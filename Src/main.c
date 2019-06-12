
#include "main.h"
#include "init.h"
#include "adc.h"
#include "foc_commutation.h"
#include "mag-encoder.h"
#include "i2c.h"
#include <string.h>
#include "settings.h"
#include "calibration.h"



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

#define BRUSHED_TAU_EBRAKE_VAL -2.0f
/*
 * Since I don't have a .README yet... i'll just put the revision notes up here
 *
 *Version change. This is now v3 hardware.
 *since we are using F302, uart is now compatible with SPI pins. We're taking advantage of this.
 *I also wrote a brushed control, for shits and goggles.
 */
void brushed_tau(float tau)
{
	if(tau <= 1.0f && tau >= -1.0f)
	{
		uint32_t offset = (int32_t)(tau*500.0f);
		TIMER_UPDATE_DUTY(500-offset,500+offset,500-offset);
	}
	else	//I want to add an e-brake feature. Easy way is to just make out of range values stop the motor.
	{
		TIMER_UPDATE_DUTY(0,0,0);
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

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	TIMER_UPDATE_DUTY(0,0,0);
	TIM1->CCR4 = 650;	//for 7_5, you have about 8uS of sampling.you want to catch the current waveform right at the middle

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)dma_adc_raw, NUM_ADC);

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

#ifndef BRUSHED_MODE

#ifdef GET_ENCODER_MIDPOINT
	obtain_encoder_midpoints();
#endif
#ifdef GET_INIT_OFFSET
	/*Best Method*/
	align_offset = sweep_align(-PI/elec_conv_ratio,0);
#endif
//	manual_align_calib();//-0.236198962

	TIMER_UPDATE_DUTY(0,0,0);

#ifdef TEST_FOC
	test_foc();
#endif

	float x_iq_PI = 0;	//torque pi state
	float x_id_PI = 0;	//flux pi state
	float uq = 0;
	float ud = 0;
	float iqref = 0;
	float idref = 0;

	theta_m_prev = 0;

//	uint8_t status = CAL_ERR_TIMEOUT;
//	while(status != CAL_SUCCESS)
//	{
//		check_encoder_region(0.5f, 50, 0.15f, 0.7f, 3000, &status);
//		if(status == CAL_ERR_TIMEOUT)
//			check_encoder_region(-0.5f, 50, 0.15f, 0.7f, 3000, &status);
//	}
//	theta_m_prev = theta_enc_prev;
	check_encoder_region_2(1000, 100000);	//less violent. For more robust operation, make the first number larger. If the distance sweep is too big, make it smaller.
	theta_m_prev=theta_enc_prev;

	while(1)
	{
		handle_i2c(&hi2c1);

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


#else
	float iqref = 0;
	while(1)
	{
		handle_i2c(&hi2c1);

		float * f_addr = (float *)(i2c_rx_data);
		iqref = *f_addr;

		if(iqref > MAX_CUR)
			iqref = MAX_CUR;
		else if (iqref < -MAX_CUR)
			iqref = -MAX_CUR;
		if(HAL_GetTick() > recieved_data_ts + 100)
			iqref = BRUSHED_TAU_EBRAKE_VAL;

		brushed_tau(iqref);

		float theta_m = unwrap(theta_abs_rad(), &theta_m_prev)*.5;

		uint8_t * ptr = (uint8_t *)(&theta_m);
		i2c_tx_data[0] = ptr[0];	//i did this to keep i2c_tx_data in the same place
		i2c_tx_data[1] = ptr[1];	//however, just changing it's base pointer
		i2c_tx_data[2] = ptr[2];	//is probably totally fine
		i2c_tx_data[3] = ptr[3];

		if(HAL_GetTick()>=led_ts)
		{
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,led_state);
			led_state = !led_state & 1;
			led_ts = HAL_GetTick() + 200;
		}
	}
#endif

}

