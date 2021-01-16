#include "main.h"
#include "init.h"
#include "adc.h"
#include "foc_commutation.h"
#include "mag-encoder.h"
#include "settings.h"
#include "calibration.h"
#include "CAN.h"

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


#define MAX_CUR 90.0

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


/**
 * @brief Rx Transfer completed callback.
 * @param huart UART handle.
 * @retval None
 */
uint8_t gl_uart_cplt_flag = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	gl_uart_cplt_flag = 1;
}

typedef union floatsend_chk_t
{
	float v;
	uint8_t d[sizeof(float) + 1];
}floatsend_chk_t;

int main(void)
{
	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_DMA_Init();
	CAN_Init();
	MX_ADC1_Init();
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
	TIM1->CCR4 = TIM1->ARR-1;	//for 7_5, you have about 8uS of sampling.you want to catch the current waveform right at the middle

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

	uint32_t led_ts = HAL_GetTick()+100;

#ifdef GET_ENCODER_MIDPOINT
	obtain_encoder_midpoints();
#endif
#ifdef GET_INIT_OFFSET
	/*Best Method*/
	auto_calibrate_motor();
#endif
	//	manual_align_calib();//-0.236198962

	//	align_offset = sweep_align(-PI/elec_conv_ratio,0, 0.1f, 15.0f, 300);
	//	align_offset = sweep_align(align_offset-.3f,align_offset+0.3f, 0.001f, 30.0f, 400);
	//	align_offset = sweep_align(-0.671f, -0.668f, 0.0001f, 40.0f, 500);

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

	floatsend_chk_t uart_encoder_data;
	for(int i = 0; i < 5; i++)
		uart_encoder_data.d[i] = 0;

	floatsend_t can_rx_data;
	floatsend_t can_tx_data;

#ifdef USE_KMZ60_ENCODER
	force_encoder_region();
	if(theta_enc_prev < -PI)
		theta_enc_prev += TWO_PI*2.0f;
	//	theta_m_prev=theta_enc_prev;
#endif

	uint32_t dq_ts = 0;
	float prev_theta_m_dq = 0;
	float phase_advance_angle = 0;
	uint32_t can_cmd_exp_ts = 0;
	while(1)
	{
		/*CAN protocol. */
		if(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) >= 1)
		{
			if(HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &can_rx_header, can_rx_data.d) == HAL_OK)
			{
				if(can_rx_header.StdId == MODULE_NUMBER)
				{
					iqref = can_rx_data.v;
					can_cmd_exp_ts = HAL_GetTick() + 500;
				}
				else if (can_rx_header.StdId == ext_cmd_id)
					handle_ext_cmd(&gl_ext_cmd_flags, can_rx_data.d);
				HAL_CAN_AddTxMessage(&hcan, &can_tx_header, can_tx_data.d, &can_tx_mailbox);
			}	//NOTE the rx header will contain the message ID
		}
		if(HAL_GetTick() > can_cmd_exp_ts)
			iqref = 0.f;

		//		parse_master_cmd(&led_ts);

		/****************************************FOC functional block*************************************/
		float i_a,i_b,i_c;
		conv_raw_current(&i_a,&i_b, &i_c);

		float i_alpha,i_beta;
		clarke_transform(i_a,i_b,i_c,&i_alpha, &i_beta);

		float theta_enc = unwrap(theta_rel_rad(), &theta_enc_prev);
		float theta_elec = theta_enc*elec_conv_ratio + phase_advance_angle;		//test this! MS motor has 22 pole pairs, which means multiply by 11. ENCODER RETURNS 2*theta so we absorb .5 into this term
		//		float theta_elec = theta_enc*elec_conv_ratio;		//test this! MS motor has 22 pole pairs, which means multiply by 11. ENCODER RETURNS 2*theta so we absorb .5 into this term
		theta_elec = fmod_2pi(theta_elec + PI) - PI;		//re-modulate theta_m. ensure that the angle is constrained from -pi to pi!!
		float sin_theta = sin_fast(theta_elec);				//calculate the sin of the electrical (magnetic flux) angle
		float cos_theta = cos_fast(theta_elec);				//and the cosine for park and inverse park domains

		float i_q, i_d;
		park_transform(i_alpha, i_beta, sin_theta,cos_theta, &i_q, &i_d);
#ifdef FLIP_TAU
		controller_PI(-iqref, i_q, iq_gain, 0.000000001, &x_iq_PI, &uq);		//this sort of works
#else
		controller_PI(iqref, i_q, iq_gain, 0.0f, &x_iq_PI, &uq);		//this sort of works
#endif
//		controller_PI(idref, i_d, id_gain, 0.0f, &x_id_PI, &ud);		//high current
		ud = 0.f;

		uint32_t tA,tB,tC;
		if(uq > 0.8f)
			uq = 0.8f;
		if(uq < -0.8f)
			uq = -0.8f;
		inverse_park_transform(uq, ud, sin_theta,cos_theta, &i_alpha, &i_beta);	//maybe call theta rel again?sin_theta
		svm(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);
		TIMER_UPDATE_DUTY(tA,tB,tC);
		/**********************************************END**********************************************/

		/*phase advancement. necessary for high bw commutation*/
		if(HAL_GetTick() > dq_ts) //in this implementation, mechanical theta is computed outside the loop and shared with foc.
		{
			float dq = (theta_enc-prev_theta_m_dq)*25.0f;
			prev_theta_m_dq = theta_enc;
			phase_advance_angle = dq/gl_prop_delay_const;//division yields more accurate advance angle for better performance.
			dq_ts = HAL_GetTick()+40;

//			clear_latched_nfault();	//TODO: add this in and TEST IT
		}

		/*Load CAN message with motor position*/
		if(gl_ext_cmd_flags.use_uart_encoder == 0)
			can_tx_data.v = theta_enc;
		else
		{
			HAL_UART_Receive_IT(&huart2, uart_encoder_data.d, 5);
			if(gl_uart_cplt_flag == 1)	//should work
			{
				//TODO: add checksum calculation and check
				can_tx_data.v = uart_encoder_data.v;
				gl_uart_cplt_flag=0;
			}
		}


		if(HAL_GetTick()>=led_ts && gl_ext_cmd_flags.en_blink_led)
		{
			HAL_GPIO_TogglePin(STAT_PORT,STAT_PIN);
			led_ts = HAL_GetTick() + 200;
		}
	}

}
