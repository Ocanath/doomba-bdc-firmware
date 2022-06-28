#include "main.h"
#include "init.h"
#include "adc.h"
#include "foc_commutation.h"
#include "mag-encoder.h"
#include "settings.h"
#include "calibration.h"
#include "calibration_fixed.h"
#include "CAN.h"
#include "foc_fixed.h"
#include "m_uart.h"

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


static volatile int16_fmt_t uart_encoder_data = {0};

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

int32_t input_val = 0;
static volatile int32_t vq_pctl = 0;
static volatile int32_t iq_pctl = 0;
/**
  */
void m_uart_rx_cplt_callback(uart_it_t * h)
{
	int8_t sum = 0;
	for(int i = 0; i < h->bytes_received; i++)
		sum += (int8_t)h->rx_buf[i];
	if(sum == 0)
	{
		for(int i = 0; i < sizeof(int16_t); i++)	//load unchecked data, in case we want to use that
			uart_encoder_data.d[i] = h->rx_buf[i];
		if(gl_ext_cmd_flags.control_mode == POS_IQ)
		{
			int32_t q = (int32_t)uart_encoder_data.v;

			int32_t iq = ctl_PI_fixed( (input_val - q), &mctl_iq.kpki);
			iq -= (gl_dtheta_fixedpoint_rad_p_sec*mctl_iq.kd.i32) >> mctl_iq.kd.radix;
			iq = iq * (1 << gl_iq_rshift);
			iq = saturate(iq, mctl_iq.out_sat);
			iq_pctl = iq;
		}
		else if (gl_ext_cmd_flags.control_mode == POS_VQ)
		{
			int32_t q = (int32_t)uart_encoder_data.v;

			int32_t vq = ctl_PI_fixed( (input_val - q), &mctl_vq.kpki);
			vq -= (gl_dtheta_fixedpoint_rad_p_sec*mctl_vq.kd.i32) >> mctl_vq.kd.radix;
			vq = saturate(vq, mctl_vq.out_sat);
			vq_pctl = vq;
		}

	}
}

void test_current_sensor(void);

int main(void)
{
	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_DMA_Init();
	CAN_Init();
	MX_ADC1_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_USART2_UART_Init();

	if(sizeof(float) != 4)
	{
		while(1)
		{
			print_str("ERROR! INVALID FLOAT SIZE\r\n");
			HAL_Delay(1000);
		}
	}

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)dma_adc_raw, NUM_ADC);	//start dma conversions before the trigger for them (i.e. timer1 triggerout)
	hadc1.Instance->CR |= (1 << 3);	//enable injected conversions (jadstart)
	//disable adc dma interrupts. The adc itself should only have overrun interrupt enabled
	hdma_adc1.Instance->CCR &= ~(1 << 2);	//disable half transfer complete interrupt
	hdma_adc1.Instance->CCR &= ~(1 << 1);	//disable transfer complete interrupt


	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	TIMER_UPDATE_DUTY(0,0,0);
	TIM1->CCR4 = TIM1->ARR-1;	//for 7_5, you have about 8uS of sampling.you want to catch the current waveform right at the middle

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
//	auto_calibrate_fixed();
#endif
	//	manual_align_calib();//-0.236198962

	//	align_offset = sweep_align(-PI/elec_conv_ratio,0, 0.1f, 15.0f, 300);
	//	align_offset = sweep_align(align_offset-.3f,align_offset+0.3f, 0.001f, 30.0f, 400);
	//	align_offset = sweep_align(-0.671f, -0.668f, 0.0001f, 40.0f, 500);

	TIMER_UPDATE_DUTY(0,0,0);

#ifdef TEST_FOC
	test_foc();
#endif

	theta_m_prev = 0;

	can_payload_t can_rx_data = {0};
	can_payload_t can_tx_data = {0};

#ifdef USE_KMZ60_ENCODER
	force_encoder_region();
	if(theta_enc_prev < -PI)
		theta_enc_prev += TWO_PI*2.0f;
	//	theta_m_prev=theta_enc_prev;
#endif

//	CoreDebug->DEMCR |= 0x01000000;
//	DWT->CYCCNT = 0;
//	DWT->CTRL |= 1;	//enable it

//	get_cursensor_calibrations(200,curr_scf);
//	current_sensors_fitness_test();

	uint32_t can_cmd_exp_ts = 0;
	test_current_sensor();

	while(1)
	{
		/*CAN protocol. Get an input arg*/
		if(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) >= 1)
		{
			if(HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &can_rx_header, can_rx_data.d) == HAL_OK)
			{
				if(can_rx_header.StdId == MODULE_NUMBER)
				{
					input_val = (int32_t)can_rx_data.i16[0];
					can_cmd_exp_ts = HAL_GetTick() + 500;

					if(gl_ext_cmd_flags.use_uart_encoder == 0)	//Load CAN message with motor position
					{
						can_tx_data.i32[0] = gl_theta_rem_m;	//bytes 0-3
						can_tx_data.i16[2] = (int16_t)(gl_iq >> gl_iq_rshift);				//bytes 4-5
					}
					else
					{
						can_tx_data.i16[0] = uart_encoder_data.v;	//this is wasteful of 2 bytes. Consider reducing for higher bandwidth CAN

						/*
						 * the choice of shift here was to ensure a reasonable highest speed would fit,
						 * without compromising resolution for lower speeds, while still maintaining a
						 *  unit-governed (i.e. div by seconds) expression of velocity
						 *
						 * This expresses velocity in (2^(12-shift)rad / sec, or 16rad/sec.
						 * This grows the resolution of the fixed point value. divide by 2^(12-shift)
						 * to obtain the true value in rad/sec
						 * */

						can_tx_data.i16[1] = (int16_t)(gl_dtheta_fixedpoint_rad_p_sec);	//delivered in units of 2^(12-shift)rad/sec.
						can_tx_data.i16[2] = (int16_t)(gl_iq >> gl_iq_rshift);
					}
				}
				else if (can_rx_header.StdId == ext_cmd_id)
				{
					handle_ext_cmd(&gl_ext_cmd_flags, can_rx_data.d);
					for(int i = 0; i < sizeof(can_payload_t); i++)
						can_tx_data.d[i] = can_rx_data.d[i];	//acknowledge with the same message
				}
				HAL_CAN_AddTxMessage(&hcan, &can_tx_header, can_tx_data.d, &can_tx_mailbox);
			}	//NOTE the rx header will contain the message ID
		}
		if(HAL_GetTick() > can_cmd_exp_ts)
			input_val = 0;


		/*This actually runs foc. Each section is a wrapper for foc or sinusoidal*/
		if(gl_ext_cmd_flags.control_mode == FOC_MODE)
		{
			int32_t tau = input_val * (1 << gl_iq_rshift);	//scale up to match scale we use to transmit/report iq
			foc_fixed(tau, 0);
		}
		else if(gl_ext_cmd_flags.control_mode == POS_IQ)
		{
			foc_fixed(iq_pctl, 0);
		}
		else if (gl_ext_cmd_flags.control_mode == POS_VQ)
		{
			sinusoidal_fixed(vq_pctl);
		}
		else
			sinusoidal_fixed(input_val);

		get_dtheta();	//estimate velocity. synchronous with sinusoidal/foc calls, uses timer.

		/*Blink the led if enabled*/
		if(HAL_GetTick()>=led_ts && gl_ext_cmd_flags.en_blink_led)
		{
			HAL_GPIO_TogglePin(STAT_PORT,STAT_PIN);
			led_ts = HAL_GetTick() + 200;
		}
	}

}








void test_current_sensor(void)
{
	uint32_t uart_tx_ts = 0;
	int32_t vq = 1000;
	while(1)
	{
		uint32_t tick = HAL_GetTick();
		int32_t theta = (int32_t)(tick*5);


		if(tick < 6000)
			vq = 1000;
		else
			vq = 300;
//		theta = 120*PI_12B/180;

		theta = wrap_2pi_12b(theta);
		int32_t sth = sin_12b(theta);
		int32_t cth = cos_12b(theta);
		int32_t alpha, beta, tA, tB, tC;
		inverse_park_fixed(vq, 0, sth, cth, &alpha, &beta);
		svm_fixed(alpha, beta, COMM_PWM_PERIOD, &tA, &tB, &tC);
		TIMER_UPDATE_DUTY(tA,tB,tC);


//		sinusoidal_fixed(vq,0);


		if(tick >= uart_tx_ts)
		{
			uart_tx_ts = tick + 10;

			int16_fmt_t fmt;
			int bidx = 0;
			uint8_t buf[8*sizeof(int16_t)] = {0};

//			int32_t iA = (int32_t)( (hadc1.Instance->JDR1 << 1) + 1) - current_input_offset_A;
//			int32_t iB = (int32_t)( (hadc1.Instance->JDR2 << 1) + 1) - current_input_offset_B;
//			int32_t iC = (int32_t)( (hadc1.Instance->JDR3 << 1) + 1) - current_input_offset_C;
//			iA = (iA*curr_scf[0])/637100;
//			iB = (iB*curr_scf[1])/637100;
//			iC = (iC*curr_scf[2])/637100;	//this is now equal to the current in amps * 4096
			int32_t iA = dma_adc_raw[0] - current_input_offset_A;
			int32_t iB = dma_adc_raw[1] - current_input_offset_B;
			int32_t iC = -(iA + iB);

			int32_t theta_m = theta_rel_fixed();	//atan2, apply align offset
			int32_t theta_elec = wrap_2pi_12b(theta_m);
			int32_t sin_val = sin_12b(theta_elec);
			int32_t cos_val = cos_12b(theta_elec);
			int32_t iq_meas, id_meas;
			get_current(sin_val, cos_val, &iq_meas, &id_meas);

//			fmt.v = tA;
//			buffer_data(fmt.d, sizeof(int16_t), buf, &bidx);
//			fmt.v = tB;
//			buffer_data(fmt.d, sizeof(int16_t), buf, &bidx);
//			fmt.v = tC;
//			buffer_data(fmt.d, sizeof(int16_t), buf, &bidx);

			fmt.v = iA;
			buffer_data(fmt.d, sizeof(int16_t), buf, &bidx);
			fmt.v = iB;
			buffer_data(fmt.d, sizeof(int16_t), buf, &bidx);
			fmt.v = iC;
			buffer_data(fmt.d, sizeof(int16_t), buf, &bidx);
			fmt.v = wrap_2pi_12b(theta_rel_fixed());
			buffer_data(fmt.d, sizeof(int16_t), buf, &bidx);

//			fmt.v = iq_meas;
//			buffer_data(fmt.d, sizeof(int16_t), buf, &bidx);
//			fmt.v = id_meas;
//			buffer_data(fmt.d, sizeof(int16_t), buf, &bidx);

			m_uart_tx_start(&m_huart2, buf, 4*sizeof(int16_t));	//HAL_UART_Transmit_IT(&huart2,buf,4*sizeof(float));
		}
	}
}


