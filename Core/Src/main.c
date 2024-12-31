#include <FDCAN.h>
#include "main.h"
#include "init.h"
#include "adc.h"
#include "mag-encoder.h"
#include "settings.h"
#include "calibration.h"
#include "calibration_fixed.h"
#include "foc_fixed.h"
#include "m_uart.h"
#include "PPP.h"

static volatile int16_fmt_t uart_encoder_data = {0};

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
		if(gl_ext_cmd_flags.control_mode == PCTL_IQ)
		{
			int32_t q = (int32_t)uart_encoder_data.v;

			int32_t iq = ctl_PI_fixed_2( wrap_2pi_12b(input_val - q), &mctl_iq.kpki);
			iq -= (gl_dtheta_fixedpoint_rad_p_sec*mctl_iq.kd.i32) >> mctl_iq.kd.radix;
			iq = iq * (1 << gl_iq_rshift);
			iq = saturate(iq, mctl_iq.out_sat);
			iq_pctl = iq;
		}
		else if (gl_ext_cmd_flags.control_mode == PCTL_VQ)
		{
			int32_t q = (int32_t)uart_encoder_data.v;

			int32_t vq = ctl_PI_fixed_2( wrap_2pi_12b(input_val - q), &mctl_vq.kpki);
			vq -= (gl_dtheta_fixedpoint_rad_p_sec*mctl_vq.kd.i32) >> mctl_vq.kd.radix;
			vq = saturate(vq, mctl_vq.out_sat);
			vq_pctl = vq;
		}

	}
}

int main(void)
{

	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_FDCAN1_Init();
	MX_TIM1_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_SPI1_Init();

	FDCAN_Config();

	load_flash_params();

	set_encoder_read_type(READ_ANGLECOM);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)dma_adc_raw, NUM_ADC);
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

	HAL_GPIO_WritePin(ENABLE_GPIO_Port,ENABLE_Pin,1);
	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin, 1);

	TIMER_UPDATE_DUTY(1773,1773,1773);
	TIM1->CCR4 = TIM1->ARR-1;	//for 7_5, you have about 8uS of sampling.you want to catch the current waveform right at the middle
	/*
	 *	Rembember!!!!!
	 *	1. OPTION BYTES disable boot0 pin or everything will appear fucked up even though it's fine
	 *	2. use get encoder midpoint to check the inverter
	 *	3. Plug the correct elec conversion coefficient into the flash memory if it is not default correct (11 polepairs)
	 *	3.1 Plug the correct CAN id into entry 0
	 *	3.2 Plug best guess FOC control gains into relevant entries
	 *	4. use autocal to cal IFF 3 is good. Live expressions are your friend for this
	 *	5. check the settings in STM32CubeProgrammer
	 *	6. profit?
	 */
#ifdef LOCK_FOR_POLEPAIR_COUNT
	while(1)
	{
		openloop_fixed(0,1500);
	}
#endif
#ifdef TEST_INVERTER
	uint32_t angle = 0;
	while(1)
	{
		angle++;
		openloop_fixed(angle, 450);
	}
#endif
#ifdef GET_INIT_OFFSET
	auto_calibrate_motor(3000, 700, 7.f,.001);
	m_write_flash((uint64_t*)&fds_mp,sizeof(fds_motor_params_t)/sizeof(uint64_t));
#endif
#ifdef TEST_FOC
	while(1)
		sinusoidal_fixed(3546,0);	//000003E2
#endif

	uint32_t uart_tx_ts = 0;
	int32_t can_command_value = 0;
	uint32_t can_cmd_exp_ts = 0;
	while(1)
	{
		uint32_t tick = HAL_GetTick();

		if(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) != 0)
		{
			HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &can_rx_header, can_rx_data.u8);
			if(can_rx_header.Identifier == fds_mp.module_number)	//obey commands from specific device
			{
				//load negation in ALL terms due to encoder-related sign flipping between V2 and V3 actuators
				can_command_value = can_rx_data.i32[0];
				can_cmd_exp_ts = tick;

				can_tx_data.i32[0] = gl_theta_rem_m;
				can_tx_data.i16[2] = (int16_t)(gl_iq >> gl_iq_rshift);
				can_tx_data.i16[3] = (int16_t)gl_dtheta_fixedpoint_rad_p_sec;
				can_tx_header.Identifier = fds_mp.module_number;
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_header, can_tx_data.u8);
			}
			else if (can_rx_header.Identifier == 0x7FF - fds_mp.module_number)
			{
				handle_ext_cmd(&gl_ext_cmd_flags, can_rx_data.u8);
			}
		}


		/*Do the control modes*/
		if(gl_ext_cmd_flags.control_mode == FOC_MODE)
		{
			foc_fixed(can_command_value, 0);
			if((tick - can_cmd_exp_ts) > 500)
				can_command_value = 0;
		}
		else if(gl_ext_cmd_flags.control_mode == SINUSOIDAL_MODE)
		{
			sinusoidal_fixed(can_command_value, 0);
			if((tick - can_cmd_exp_ts) > 500)
				can_command_value = 0;
		}
		else if (gl_ext_cmd_flags.control_mode == OPEN_LOOP_MODE)
		{
			openloop_fixed(can_command_value, 350);
		}
		else
		{
			if(gl_ext_cmd_flags.use_uart_encoder != 0)
			{
				if(gl_ext_cmd_flags.control_mode == PCTL_IQ)
				{
					foc_fixed(iq_pctl, 0);
				}
				else if (gl_ext_cmd_flags.control_mode == PCTL_VQ)
				{
					sinusoidal_fixed(vq_pctl, 0);
				}
			}
			else
			{
				if(gl_ext_cmd_flags.control_mode == PCTL_VQ)
				{
					int32_t qd = can_command_value;
					int32_t q = gl_theta_rem_m;
					int32_t e = wrap_2pi_14b(qd-q);
					int32_t vq = ctl_PI_fixed_2(e, &mctl_vq.kpki);
					int32_t damping = (gl_dtheta_fixedpoint_rad_p_sec * mctl_vq.kd.i32) >> mctl_vq.kd.radix;
					vq -= damping;
					vq = saturate(vq, mctl_vq.out_sat);
					sinusoidal_fixed(vq,0);

//					if(m_huart2.bytes_to_send == 0 && (tick - uart_tx_ts) > 3 )
//					{
//						uart_tx_ts = tick;
//
//						uint32_t * pwb32 = (uint32_t*)(&gl_working_buf[0]);
//						int i32widx = 0;
//						pwb32[i32widx++] = qd;
//						pwb32[i32widx++] = gl_theta_rem_m;
//						pwb32[i32widx++] = mctl_vq.kpki.x >> mctl_vq.kpki.ki.radix;
//						pwb32[i32widx++] = vq;
//						pwb32[i32widx++] = gl_dtheta_fixedpoint_rad_p_sec;
//
//						//			pwb32[i32widx++] =  ((int16_t)ADC1->JDR1) - 1990;
//						//			pwb32[i32widx++] =  ((int16_t)ADC1->JDR2) - 1990;
//						//			pwb32[i32widx++] =  ((int16_t)ADC1->JDR3) - 1990;
//
//						//			pwb32[i32widx++] = theta_elec;
//						//			pwb32[i32widx++] = vq;
//						//			pwb32[i32widx++] = gl_iq;
//
//						//			pwb32[i32widx++] = 0;
//						//			pwb32[i32widx++] = id_req;
//						//			pwb32[i32widx++] = gl_id;
//						//			pwb32[i32widx++] = 300;
//						//			pwb32[i32widx++] = gl_iq;
//
//						//			int32_t s,c;
//						//			get_sc_no_ortho_comp(&s,&c);
//						//			pwb32[i32widx++] = s;
//						//			pwb32[i32widx++] = c;
//
//						pwb32[i32widx++] = HAL_GetTick();
//
//						int pld_size = i32widx*sizeof(uint32_t);
//						int stuffed_length = PPP_stuff( (uint8_t*)(&gl_working_buf[0]), pld_size, gl_ppp_stuffed_buf, STUFF_BUF_SIZE );
//						m_uart_tx_start((uart_it_t*)(&m_huart2), gl_ppp_stuffed_buf, stuffed_length);
//					}
				}
			}
		}


		get_dtheta();
		//		if(tick-led_ts > 500)
		//		{
		//			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		//			led_ts = tick;
		//		}
	}
}
