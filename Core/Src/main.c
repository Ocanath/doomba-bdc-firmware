#include "main.h"
#include "init.h"
#include "adc.h"
#include "foc_commutation.h"
#include "mag-encoder.h"
#include "settings.h"
#include "calibration.h"
#include "CAN.h"
#include "foc_fixed.h"

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

void buffer_data(uint8_t * d, int size_d, uint8_t * buf, int * bidx)
{
	for(int i = 0; i < size_d; i++)
	{
		buf[*bidx] = d[i];
		*bidx = *bidx + 1;
	}
}


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
#endif
	//	manual_align_calib();//-0.236198962

	//	align_offset = sweep_align(-PI/elec_conv_ratio,0, 0.1f, 15.0f, 300);
	//	align_offset = sweep_align(align_offset-.3f,align_offset+0.3f, 0.001f, 30.0f, 400);
	//	align_offset = sweep_align(-0.671f, -0.668f, 0.0001f, 40.0f, 500);

	TIMER_UPDATE_DUTY(0,0,0);

#ifdef TEST_FOC
	test_foc();
#endif
	float iqref = 0.f;

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

	CoreDebug->DEMCR |= 0x01000000;
	DWT->CYCCNT = 0;
	DWT->CTRL |= 1;	//enable it

	uint32_t ts = 0;

	/*This is FOC.*/
	while(1)
	{
		int32_t tau = ( (3000*(TWO_PI_12B*200-gl_theta_rem_m)) - (gl_dtheta*50000)) >> 12;
		tau = saturate(tau,3000);

		foc_fixed(tau, 0);

		uint32_t tick =  HAL_GetTick();
		if(tick > ts)
		{
			ts = tick + 3;

			int32_fmt_t fmt;
			int bidx = 0;
			uint8_t buf[3*sizeof(int32_t)];

			fmt.v = gl_theta_rem_m;
			buffer_data(fmt.d, sizeof(int32_t), buf, &bidx);
			fmt.v = gl_theta_rotations;
			buffer_data(fmt.d, sizeof(int32_t), buf, &bidx);
			fmt.v = gl_iq;
			buffer_data(fmt.d, sizeof(int32_t), buf, &bidx);

			HAL_UART_Transmit_IT(&huart2, buf, 3*sizeof(int32_t));
		}
	}



	while(1)
	{
		uint32_t tick = HAL_GetTick();
		int32_t ft = -tick*20;
		ft = wrap_2pi_12b(ft);
		int32_t sin_theta = sin_fixed(ft);
		int32_t cos_theta = cos_fixed(ft);

		int32_t alpha,beta;
		int32_t tA,tB,tC;
		inverse_park_fixed(200,0,sin_theta,cos_theta,&alpha,&beta);
		svm_fixed(alpha,beta,TIM1->ARR, &tA, &tB, &tC);
		TIMER_UPDATE_DUTY(tA,tB,tC);

		/*Get current in amps*4096*/
		int32_t ia = (int32_t)( (hadc1.Instance->JDR1 << 1) + 1) - current_input_offset_A;
		int32_t ib = (int32_t)( (hadc1.Instance->JDR2 << 1) + 1) - current_input_offset_B;
		int32_t ic = (int32_t)( (hadc1.Instance->JDR3 << 1) + 1) - current_input_offset_C;
		ia = (ia*31900) >> 13;	//from foc_fixed.c, the current conversion weights obtained with the magic method
		ib = (ib*60481) >> 13;
		ic = (ic*149736) >> 13;

		if(tick >= ts)
		{
			ts = tick + 1;

			float ia_f = ((float)ia);
			float ib_f = ((float)ib);
			float ic_f = ((float)ic);
			float va_f = ((float)tA);
			float vb_f = ((float)tB);
			float vc_f = ((float)tC);
			floatsend_t fmt;
			int bidx = 0;
			uint8_t buf[6*sizeof(float)] = {0};
			fmt.v = ia_f;
			buffer_data(fmt.d, sizeof(float), buf, &bidx);
			fmt.v = ib_f;
			buffer_data(fmt.d, sizeof(float), buf, &bidx);
			fmt.v = ic_f;
			buffer_data(fmt.d, sizeof(float), buf, &bidx);
			fmt.v = va_f;
			buffer_data(fmt.d, sizeof(float), buf, &bidx);
			fmt.v = vb_f;
			buffer_data(fmt.d, sizeof(float), buf, &bidx);
			fmt.v = vc_f;
			buffer_data(fmt.d, sizeof(float), buf, &bidx);
			HAL_UART_Transmit_IT(&huart2,buf,6*sizeof(float));
		}
	}

	fixed_PI_params_t g_f_a = {
			.kp_m = 3 << 12,
			.kp_div = 10,
			.ki_m = 1 << 12,
			.ki_div = 1000,
			.x = 0,
			.x_sat = 700 << 12
	};
	fixed_PI_params_t g_f_b = {
			.kp_m = 3 << 12,
			.kp_div = 10,
			.ki_m = 1 << 12,
			.ki_div = 1000,
			.x = 0,
			.x_sat = 700 << 12
	};
	fixed_PI_params_t g_f_c = {
			.kp_m = 3 << 12,
			.kp_div = 10,
			.ki_m = 1 << 12,
			.ki_div = 1000,
			.x = 0,
			.x_sat = 700 << 12
	};
	int32_t iref_a, iref_b, iref_c;
	iref_a = 0;
	iref_b = 0;
	iref_c = 0;
	int32_t mag = 410*7;
	int32_t sat = 500;
	while(1)
	{
		/*Get current in amps*4096*/
		int32_t ia = (int32_t)( (hadc1.Instance->JDR1 << 1) + 1) - current_input_offset_A;
		int32_t ib = (int32_t)( (hadc1.Instance->JDR2 << 1) + 1) - current_input_offset_B;
		int32_t ic = (int32_t)( (hadc1.Instance->JDR3 << 1) + 1) - current_input_offset_C;
		ia = (ia*31900) >> 13;	//from foc_fixed.c, the current conversion weights obtained with the magic method
		ib = (ib*60481) >> 13;
		ic = (ic*149736) >> 13;

		/*Track sinusoidal currents*/
		int32_t va, vb, vc;
		va = ctl_PI_fixed(iref_a, ia, &g_f_a);
		vb = ctl_PI_fixed(iref_b, ib, &g_f_b);
		vc = ctl_PI_fixed(iref_c, ic, &g_f_c);

		/*Our control efforts are really voltages relative to *each other*. That means we can center them about a neutral reference point without changing their effect
		 * on the plant*/
		int32_t vn = (va+vb+vc)/3;	//Create neutral point reference to center our control efforts
		va = va - vn;
		vb = vb - vn;
		vc = vc - vn;

		/*saturate control effort*/
		va = saturate(va,sat);
		vb = saturate(vb,sat);
		vc = saturate(vc,sat);
		TIMER_UPDATE_DUTY(va+562,vb+562,vc+562);

		/*Print off the voltage and current data, and generate new setpoints*/
		uint32_t tick = HAL_GetTick();
		if(tick >= ts)
		{
			ts = tick + 1;

			float ia_f = ((float)ia);
			float ib_f = ((float)ib);
			float ic_f = ((float)ic);
			float va_f = ((float)va);
			float vb_f = ((float)vb);
			float vc_f = ((float)vc);
			floatsend_t fmt;
			int bidx = 0;
			uint8_t buf[6*sizeof(float)] = {0};
			fmt.v = ia_f;
			buffer_data(fmt.d, sizeof(float), buf, &bidx);
			fmt.v = ib_f;
			buffer_data(fmt.d, sizeof(float), buf, &bidx);
			fmt.v = ic_f;
			buffer_data(fmt.d, sizeof(float), buf, &bidx);
			fmt.v = va_f;
			buffer_data(fmt.d, sizeof(float), buf, &bidx);
			fmt.v = vb_f;
			buffer_data(fmt.d, sizeof(float), buf, &bidx);
			fmt.v = vc_f;
			buffer_data(fmt.d, sizeof(float), buf, &bidx);
			HAL_UART_Transmit_IT(&huart2,buf,6*sizeof(float));

			/*Generate setpoints*/
			int32_t ft = -tick*7;
			iref_a = ( mag*sin_fixed(wrap_2pi_12b(ft)) >> 12);
			iref_b = ( mag*sin_fixed(wrap_2pi_12b(ft + 8579)) >> 12);
			iref_c = ( mag*sin_fixed(wrap_2pi_12b(ft + 17157)) >> 12);
		}

	}


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

		sinusoidal(iqref, 0);


		/*Load CAN message with motor position*/
		if(gl_ext_cmd_flags.use_uart_encoder == 0)
			can_tx_data.v = gl_theta_m;
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
