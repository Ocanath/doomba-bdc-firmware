#include <FDCAN.h>
#include "main.h"
#include "init.h"
#include "adc.h"
#include "mag-encoder.h"
#include "settings.h"
#include "calibration.h"
#include "calibration_fixed.h"
#include "foc_fixed.h"
#include "m_dma_uart.h"
#include "PPP.h"
#include "dartt.h"
#include "dartt_mctl_params.h"
#include "adc.h"

static volatile int16_fmt_t uart_encoder_data = {0};

int32_t input_val = 0;
static volatile int32_t vq_pctl = 0;
static volatile int32_t iq_pctl = 0;

uint8_t gl_save_flag = 0;		//static volatile?



void start_pwm(void)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

}
void stop_pwm(void)
{
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

}


/**
 */
void m_uart_rx_cplt_callback(uart_it_t * h)
{

}

void dartt_handle_fs_update_request(void)
{
	if(gl_dp.update_fs != 0)
	{
		TIMER_UPDATE_DUTY(500,500,500);
		m_write_flash((uint64_t*)fds_mp, sizeof(fds_motor_params_t)/sizeof(uint64_t));
		gl_dp.update_fs = 0;
	}
}

void dartt_openloop_test(void)
{
	if(gl_dp.do_open_loop_test)
	{
		uint32_t angle = 0;
		uint32_t exp_ts = HAL_GetTick();
		while((HAL_GetTick() - exp_ts) < 10000)
		{
			angle++;
			openloop_fixed(angle, 0, gl_dp.open_loop_vd);	//NOTE: use vd. lock to vd gives approximate align offset.
		}
		gl_dp.do_open_loop_test = 0;
	}
}

void dartt_autocalibrate(void)
{
	if(gl_dp.auto_calibrate_command != NO_ACTION)
	{
		if(gl_dp.auto_calibrate_command == AUTO_CALIBRATE)
		{
			auto_calibrate_motor(gl_dp.autocalibration_voltage, 700, 7.f,.001);
			m_write_flash((uint64_t*)fds_mp, sizeof(fds_motor_params_t)/sizeof(uint64_t));
		}
		else if(gl_dp.auto_calibrate_command == GET_CURRENSENSOR_ZEROPOINT)
		{
			get_currensensor_zeropoint();
			m_write_flash((uint64_t*)fds_mp, sizeof(fds_motor_params_t)/sizeof(uint64_t));
		}

		gl_dp.auto_calibrate_command = NO_ACTION;
	}
}

payload_layer_msg_t gl_can_rx_pld_msg = {};

//rx alias buffer
dartt_buffer_t can_rx_alias =
{
		.buf = &can_rx_data.u8[0],
		.size = sizeof(can_rx_data),
		.len = 0
};

//tx alias buffer
dartt_buffer_t can_tx_alias =
{
		.buf = &can_tx_data.u8[0],
		.size = sizeof(can_rx_data),
		.len = 0
};

//dartt params buffer alias
dartt_mem_t gl_dp_alias =
{
		.buf = (unsigned char *)(&gl_dp),
		.size = sizeof(gl_dp),
};

int main(void)
{

	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_FDCAN1_Init();
	MX_TIM1_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_SPI1_Init();

	FDCAN_Config();

	load_flash_params();
	check_idpi_xdiv_param();
	gl_dp.control_mode = gl_dp.fds_mp.startup_control_mode;

	unsigned char dartt_misc_address = dartt_get_complementary_address(gl_dp.fds_mp.module_number);
	set_encoder_read_type(READ_ANGLECOM);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)dma_adc_raw, NUM_ADC);
	hadc1.Instance->CR |= (1 << 3);	//enable injected conversions (jadstart)
	//disable adc dma interrupts. The adc itself should only have overrun interrupt enabled
	hdma_adc1.Instance->CCR &= ~(1 << 2);	//disable half transfer complete interrupt
	hdma_adc1.Instance->CCR &= ~(1 << 1);	//disable transfer complete interrupt
	
	start_pwm();

	HAL_GPIO_WritePin(ENABLE_GPIO_Port,ENABLE_Pin,1);
	HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin, 1);

	TIMER_UPDATE_DUTY(1773,1773,1773);
	TIM1->CCR4 = TIM1->ARR-1;	//for 7_5, you have about 8uS of sampling.you want to catch the current waveform right at the middle

	check_currentsensor_calibration();	//automatically load current sensor values if uninitialized. Should be called after pwm initialization. Runs only once - then save to flash

	/*
	 *	Rembember!!!!!
	 *	1. OPTION BYTES disable boot0 pin or everything will appear fucked up even though it's fine
	 *	2. use get encoder midpoint to check the inverter
	 *	3. Plug the correct elec conversion coefficient into the flash memory if it is not default correct (11 polepairs)
	 *	3.1 Plug the correct CAN id into entry 0
	 *	3.2 Plug best guess FOC control gains into relevant entries
	 *	4. use autocal to cal IFF 3 is good. Live expressions are your friend for this
	 *	5. check the settings in STM32CubeProgrammer
	 *	6. Verify the current sensor calibration went well. Ideally, calibrate it to an electronic load!
	 *	7. profit???
	 */
	while(1)
	{

		if(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) != 0)
		{
			HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &can_rx_header, can_rx_data.u8);
			can_rx_alias.len = (can_rx_header.DataLength >> 16) & 0xF;
			if(can_rx_header.Identifier == fds_mp->module_number)	//motor command - dartt specifies custom implementation
			{
				//load negation in ALL terms due to encoder-related sign flipping between V2 and V3 actuators
				gl_dp.command_word = can_rx_data.i32[0];

				can_tx_data.i32[0] = gl_dp.theta_rem_m;
				can_tx_data.i16[2] = (int16_t)(gl_dp.iq >> gl_iq_rshift);
				can_tx_data.i16[3] = (int16_t)gl_dp.dtheta_fixedpoint_rad_p_sec;
				can_tx_header.Identifier = MASTER_MOTOR_ADDRESS;
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_header, can_tx_data.u8);
			}
			else if (can_rx_header.Identifier == dartt_misc_address)
			{
			    int rc = dartt_frame_to_payload(&can_rx_alias, TYPE_ADDR_CRC_MESSAGE, PAYLOAD_ALIAS, &gl_can_rx_pld_msg);
			    if(rc == DARTT_PROTOCOL_SUCCESS)
			    {
			    	dartt_parse_general_message(&gl_can_rx_pld_msg, TYPE_ADDR_CRC_MESSAGE, &gl_dp_alias, &can_tx_alias);
			    }
			    if(can_tx_alias.len != 0 && rc == DARTT_PROTOCOL_SUCCESS)
			    {
			    	send_fdcan_frame(MASTER_MISC_ADDRESS, &can_tx_alias);
			    }
			}
		}

		/*Handle DARTT over UART*/
		if(m_huart2.rx_decoded.length != 0)
		{
			if(m_huart2.rx_decoded.buf[0] == dartt_misc_address)
			{
				int rc = dartt_frame_to_payload(&m_huart2.rx_decode_alias, TYPE_SERIAL_MESSAGE, PAYLOAD_ALIAS, &m_huart2.rx_pld_msg);
				if(rc == DARTT_PROTOCOL_SUCCESS)
				{
					rc = dartt_parse_general_message(&m_huart2.rx_pld_msg, TYPE_SERIAL_MESSAGE, &gl_dp_alias, &m_huart2.tx_buf_alias);
				}
				if(m_huart2.tx_buf_alias.len != 0 && rc == DARTT_PROTOCOL_SUCCESS)
				{
					m_huart2.tx_mem.length = m_huart2.tx_buf_alias.len;
					rc = cobs_encode_single_buffer(&m_huart2.tx_mem);
					if(rc == COBS_SUCCESS)
					{
						m_uart_dma_transmit(&m_huart2);
					}
				}
			}
			m_huart2.rx_decoded.length = 0;
		}

		/*Do the control modes*/
		if(gl_dp.control_mode == FOC_MODE)
		{
			foc_fixed(gl_dp.command_word, 0);
//			if((tick - can_cmd_exp_ts) > 500)
//				gl_dp.command_word = 0;
		}
		else if(gl_dp.control_mode == SINUSOIDAL_MODE)
		{
			sinusoidal_fixed(gl_dp.command_word, 0);
//			if((tick - can_cmd_exp_ts) > 500)
//				gl_dp.command_word = 0;
		}
		else if (gl_dp.control_mode == OPEN_LOOP_MODE)
		{
			openloop_fixed(gl_dp.command_word, gl_dp.open_loop_vq, gl_dp.open_loop_vd);
		}
		else
		{
			if(gl_dp.use_uart_encoder != 0)
			{
				if(gl_dp.control_mode == PCTL_IQ)
				{
					foc_fixed(iq_pctl, 0);
				}
				else if (gl_dp.control_mode == PCTL_VQ)
				{
					sinusoidal_fixed(vq_pctl, 0);
				}
			}
			else
			{
				if(gl_dp.control_mode == PCTL_VQ)
				{
					int32_t qd = gl_dp.command_word;
					int32_t q = gl_dp.theta_rem_m;
					int32_t e = wrap_2pi_14b(qd-q);
					int32_t vq = ctl_PI_fixed_2(e, &gl_dp.mctl_vq.kpki);
					int32_t damping = (gl_dp.dtheta_fixedpoint_rad_p_sec * gl_dp.mctl_vq.kd.i32) >> gl_dp.mctl_vq.kd.radix;
					vq -= damping;
					vq = saturate(vq, gl_dp.mctl_vq.out_sat);
					sinusoidal_fixed(vq,0);
				}
				else if(gl_dp.control_mode == PCTL_IQ)
				{
					int32_t qd = gl_dp.command_word;
					int32_t q = gl_dp.theta_rem_m;
					int32_t e = wrap_2pi_14b(qd-q);
					int32_t iq = ctl_PI_fixed_2(e, &gl_dp.mctl_iq.kpki);
					int32_t damping = (gl_dp.dtheta_fixedpoint_rad_p_sec * gl_dp.mctl_iq.kd.i32) >> gl_dp.mctl_iq.kd.radix;
					iq -= damping;
					iq = saturate(iq, gl_dp.mctl_iq.out_sat);
					foc_fixed(iq,0);
				}
			}
		}
		if(gl_dp.load_dartt_flags)	//wrapper to all flag handlers for slow or blocking ops. also a double guard for filesystem - you can write
		{
			gl_dp.load_dartt_flags = 0;

			dartt_handle_fs_update_request();
			dartt_openloop_test();
			dartt_autocalibrate();
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, gl_dp.led_state);
		}


		get_dtheta();
		//		if(tick-led_ts > 500)
		//		{
		//			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		//			led_ts = tick;
		//		}
	}
}









//					if(m_huart2.bytes_to_send == 0 && (tick - uart_tx_ts) > 3 )
//					{
//						uart_tx_ts = tick;
//
//						uint32_t * pwb32 = (uint32_t*)(&gl_working_buf[0]);
//						int i32widx = 0;
//						pwb32[i32widx++] = qd;
//						pwb32[i32widx++] = gl_dp.theta_rem_m;
//						pwb32[i32widx++] = mctl_vq.kpki.x >> mctl_vq.kpki.ki.radix;
//						pwb32[i32widx++] = vq;
//						pwb32[i32widx++] = gl_dp.dtheta_fixedpoint_rad_p_sec;
//
//						//			pwb32[i32widx++] =  ((int16_t)ADC1->JDR1) - 1990;
//						//			pwb32[i32widx++] =  ((int16_t)ADC1->JDR2) - 1990;
//						//			pwb32[i32widx++] =  ((int16_t)ADC1->JDR3) - 1990;
//
//						//			pwb32[i32widx++] = theta_elec;
//						//			pwb32[i32widx++] = vq;
//						//			pwb32[i32widx++] = gl_dp.iq;
//
//						//			pwb32[i32widx++] = 0;
//						//			pwb32[i32widx++] = id_req;
//						//			pwb32[i32widx++] = gl_dp.id;
//						//			pwb32[i32widx++] = 300;
//						//			pwb32[i32widx++] = gl_dp.iq;
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

