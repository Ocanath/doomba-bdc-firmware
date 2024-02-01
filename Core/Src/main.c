#include "main.h"
#include "init.h"
#include "adc.h"
#include "mag-encoder.h"
#include "settings.h"
#include "calibration.h"
#include "calibration_fixed.h"
#include "CAN.h"
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

			int32_t iq = ctl_PI_fixed( wrap_2pi_12b(input_val - q), &mctl_iq.kpki);
			iq -= (gl_dtheta_fixedpoint_rad_p_sec*mctl_iq.kd.i32) >> mctl_iq.kd.radix;
			iq = iq * (1 << gl_iq_rshift);
			iq = saturate(iq, mctl_iq.out_sat);
			iq_pctl = iq;
		}
		else if (gl_ext_cmd_flags.control_mode == PCTL_VQ)
		{
			int32_t q = (int32_t)uart_encoder_data.v;

			int32_t vq = ctl_PI_fixed( wrap_2pi_12b(input_val - q), &mctl_vq.kpki);
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

	load_flash_params();

	eval_encoder_params();	//initial call to get centerpoints and gains, for commutation

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

	uint32_t led_ts = 0;

#ifdef GET_ENCODER_MIDPOINT
	obtain_encoder_midpoints();
	fds_mp.gl_oerr_total = estimate_orthogonality_error();
	eval_encoder_params();
	m_write_flash((uint64_t*)&fds_mp,sizeof(fds_motor_params_t)/sizeof(uint64_t));
#endif
#ifdef GET_INIT_OFFSET
	auto_calibrate_motor(2000, 700, 7.f,.0001);
	m_write_flash((uint64_t*)&fds_mp,sizeof(fds_motor_params_t)/sizeof(uint64_t));
#endif
#ifdef TEST_FOC
//	volatile int32_t error = openloop_eang_pctl(0,0xFFFFFFFF);
	TIMER_UPDATE_DUTY(500,500,500);
	for(uint32_t end_ts = HAL_GetTick() + 5000; HAL_GetTick() < end_ts;)
	{
//		if(error < 714 && error > -714)	//714 is 10 degrees
		{
			//indicate pass
			test_foc();
		}
	}
#endif

//	int32_t vq = 500;
//	while(1)
//	{
//		uint32_t tick = HAL_GetTick();
//
//
////		int32_t theta_m = tick;
//		int32_t theta_m = theta_rel_fixed() + TWO_PI_12B;
//
//
//		int32_t theta_elec = wrap_2pi_12b(theta_m * fds_mp.elec_conv_ratio_fixed);
//		int32_t sin_val = sin_12b(theta_elec);
//		int32_t cos_val = cos_12b(theta_elec);
//
//		int32_t iq_meas, id_meas;
//		get_current(sin_val, cos_val, &iq_meas, &id_meas);
//
//		int32_t vd = 0;//ctl_PI_fixed(vd_in - id_meas, &fds_mp.id_pi);
//
//		int32_t alpha,beta,tA,tB,tC;
//		inverse_park_fixed(vq, vd, sin_val,cos_val,&alpha,&beta);
//		svm_fixed(alpha,beta,TIM1->ARR,&tA,&tB,&tC);
//		TIMER_UPDATE_DUTY(tA,tB,tC);
//
//
//
////		if(m_huart2.bytes_to_send == 0)
////		{
////			uint32_t * pwb32 = (uint32_t*)(&gl_working_buf[0]);
////			int i32widx = 0;
////	//		pwb32[i32widx++] = theta_elec;
//////			pwb32[i32widx++] = tau;
////			pwb32[i32widx++] = gl_theta_rem_m;
////			pwb32[i32widx++] = gl_iq;
////			pwb32[i32widx++] = gl_id;
////
////			int32_t s,c;
////			get_sc_no_ortho_comp(&s,&c);
////			pwb32[i32widx++] = s;
////			pwb32[i32widx++] = c;
////
////			pwb32[i32widx++] = HAL_GetTick();
////
////			int pld_size = i32widx*sizeof(uint32_t);
////			int stuffed_length = PPP_stuff( (uint8_t*)(&gl_working_buf[0]), pld_size, gl_ppp_stuffed_buf, STUFF_BUF_SIZE );
////			m_uart_tx_start((uart_it_t*)(&m_huart2), gl_ppp_stuffed_buf, stuffed_length);
////		}
//
//
//		if(tick-led_ts > 333)
//		{
//			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
//			led_ts = tick;
//		}
//	}



	fixed_PI_2_params_t pctl_vq = {
			.kp = {.i32 = 40, .radix = 8},
			.ki = {.i32 = 7, .radix = 10},
			.x_integral_div = 100,
			.x = 0,	//no reason to init this to another value (unless you want the motor to jump when you start pctl... like a psycho...)
			.x_sat = 300,	//no integral efforts for now
			.out_rshift = 0
	};
	i32_t pctl_kd_vq = {	//filter this will help for pos control. Need to do fixed point IIR
			.i32 = 12,	//no deriv effort either, for now
			.radix = 5
	};
	uint32_t uart_tx_ts = 0;
	while(1)
	{
		uint32_t tick = HAL_GetTick();

		int32_t qd = 0;
		int32_t q = gl_theta_rem_m;
		int32_t vq = ctl_PI_fixed_2(qd - q, (fixed_PI_2_params_t*)(&pctl_vq));
		int32_t damping = (gl_dtheta_fixedpoint_rad_p_sec * pctl_kd_vq.i32) >> pctl_kd_vq.radix;
//		vq -= damping;
		vq = saturate(vq, 500);

//		int32_t iq_req = (sin_12b(wrap_2pi_12b(tick))*100)/4096;

		sinusoidal_fixed(vq,0);

		get_dtheta();

//		tau = (sin_12b(wrap_2pi_12b( (tick*TWO_PI_12B)/4096 ))*2000)/4096;
//		foc_fixed(tau, 0);

//		while(m_huart2.bytes_to_send != 0);	//spin until uart is ready again. blocking uart
		if(m_huart2.bytes_to_send == 0 )//&& (tick - uart_tx_ts) > 1 )
		{
			uart_tx_ts = tick;

			uint32_t * pwb32 = (uint32_t*)(&gl_working_buf[0]);
			int i32widx = 0;
//			pwb32[i32widx++] = qd/10;
//			pwb32[i32widx++] = gl_theta_rem_m/10;
//			pwb32[i32widx++] = pctl_vq.x >> pctl_vq.ki.radix;
//			pwb32[i32widx++] = vq;
//			pwb32[i32widx++] = damping;


//		pwb32[i32widx++] = theta_elec;
			pwb32[i32widx++] = vq;
			pwb32[i32widx++] = gl_iq;
			pwb32[i32widx++] = gl_id;
//			int32_t s,c;
//			get_sc_no_ortho_comp(&s,&c);
//			pwb32[i32widx++] = s;
//			pwb32[i32widx++] = c;

			pwb32[i32widx++] = HAL_GetTick();

			int pld_size = i32widx*sizeof(uint32_t);
			int stuffed_length = PPP_stuff( (uint8_t*)(&gl_working_buf[0]), pld_size, gl_ppp_stuffed_buf, STUFF_BUF_SIZE );
			m_uart_tx_start((uart_it_t*)(&m_huart2), gl_ppp_stuffed_buf, stuffed_length);
		}



//		if(tick-led_ts > 500)
//		{
//			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
//			led_ts = tick;
//		}
	}
}
