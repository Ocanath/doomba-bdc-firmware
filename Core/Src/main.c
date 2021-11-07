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

/**
  */
void m_uart_rx_cplt_callback(uart_it_t * h)
{
	if( get_checksum(h->rx_buf, sizeof(int16_t)) == h->rx_buf[sizeof(int16_t)] )
	{
		for(int i = 0; i < sizeof(int16_t); i++)	//load unchecked data, in case we want to use that
			uart_encoder_data.d[i] = h->rx_buf[i];
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
	int32_t input_val = 0;
	while(1)
	{
		/*CAN protocol. */
		if(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) >= 1)
		{
			if(HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &can_rx_header, can_rx_data.d) == HAL_OK)
			{
				if(can_rx_header.StdId == MODULE_NUMBER)
				{
					input_val = (int32_t)can_rx_data.i16[0];
					can_cmd_exp_ts = HAL_GetTick() + 500;
				}
				else if (can_rx_header.StdId == ext_cmd_id)
					handle_ext_cmd(&gl_ext_cmd_flags, can_rx_data.d);

				if(gl_ext_cmd_flags.use_uart_encoder == 0)	//Load CAN message with motor position
				{
					can_tx_data.i32[0] = gl_theta_rem_m;	//bytes 0-3
					can_tx_data.i16[2] = (int16_t)gl_iq;				//bytes 4-5
				}
				else
				{
					can_tx_data.i32[0] = (int32_t)uart_encoder_data.v;	//this is wasteful of 2 bytes. Consider reducing for higher bandwidth CAN
					can_tx_data.i16[2] = (int16_t)gl_iq;
				}

				HAL_CAN_AddTxMessage(&hcan, &can_tx_header, can_tx_data.d, &can_tx_mailbox);
			}	//NOTE the rx header will contain the message ID
		}
		if(HAL_GetTick() > can_cmd_exp_ts)
			input_val = 0;

		if(gl_ext_cmd_flags.control_mode == FOC_MODE)
			foc_fixed(input_val,0);
		else
			sinusoidal_fixed(input_val,0);

		if(HAL_GetTick()>=led_ts && gl_ext_cmd_flags.en_blink_led)
		{
			HAL_GPIO_TogglePin(STAT_PORT,STAT_PIN);
			led_ts = HAL_GetTick() + 200;
		}
	}

}
