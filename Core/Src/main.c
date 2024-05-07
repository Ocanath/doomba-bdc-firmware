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

	uint32_t led_ts = 0;

#ifdef GET_ENCODER_MIDPOINT
	obtain_encoder_midpoints();
	fds_mp.gl_oerr_total = estimate_orthogonality_error();
	m_write_flash((uint64_t*)&fds_mp,sizeof(fds_motor_params_t)/sizeof(uint64_t));
#endif
#ifdef GET_INIT_OFFSET
	auto_calibrate_motor(1000, 700, 7.f,.0001);
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




	fixed_PI_2_params_t speed_controller = {
			.kp = {.i32 = 500, .radix = 8},
			.ki = {.i32 = 5, .radix = 4},
			.x_integral_div = 1000,
			.x = 0,
			.x_sat = 3546,
			.out_rshift = 0
	};
	uint32_t uart_tx_ts = 0;
	int32_t qdot_targ = 462;
	while(1)
	{
		uint32_t tick = HAL_GetTick();

//		int32_t twrapped = wrap_2pi_12b(tick);
//		int32_t qd = wrap_2pi_12b( ((twrapped*TWO_PI_12B)/1000)*5);
//		int32_t q = theta_rel_14b() >> 2;
//		int32_t e = wrap_2pi_12b(qd-q);	//go down to 12
//		int32_t vq = ctl_PI_fixed_2(e, (fixed_PI_2_params_t*)(&pctl_vq));
//		int32_t damping = (gl_dtheta_fixedpoint_rad_p_sec * pctl_kd_vq.i32) >> pctl_kd_vq.radix;
//		vq -= damping;
//		vq = saturate(vq, 1000);

		int32_t e = (qdot_targ - gl_dtheta_fixedpoint_rad_p_sec);
		int32_t u = ctl_PI_fixed_2(e, &speed_controller);
		u = saturate(u, 3546);

		sinusoidal_fixed(u,0);
//		sinusoidal_fixed(1257,0);
//		foc_fixed(1000, 0);
//		foc_fixed(u, 0);

		get_dtheta();

		if(m_huart2.bytes_to_send == 0 && (tick - uart_tx_ts) > 3 )
		{
			uart_tx_ts = tick;

			uint32_t * pwb32 = (uint32_t*)(&gl_working_buf[0]);
			int i32widx = 0;
			pwb32[i32widx++] = 0;
			pwb32[i32widx++] = qdot_targ;
			pwb32[i32widx++] = gl_dtheta_fixedpoint_rad_p_sec;
			pwb32[i32widx++] = u;
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





	fixed_PI_2_params_t pctl_vq = {
			.kp = {.i32 = 60, .radix = 8},
			.ki = {.i32 = 17, .radix = 10},
			.x_integral_div = 100,
			.x = 0,	//no reason to init this to another value (unless you want the motor to jump when you start pctl... like a psycho...)
			.x_sat = 1000,	//no integral efforts for now
			.out_rshift = 0
	};
	i32_t pctl_kd_vq = {	//filter this will help for pos control. Need to do fixed point IIR
			.i32 = 15,	//no deriv effort either, for now
			.radix = 5
	};

	while(1)
	{
		uint32_t tick = HAL_GetTick();

		int32_t qd = 0;
//		qd = (sin_12b(wrap_2pi_12b(tick))*PI_12B)/(1<<10);
		qd = wrap_2pi_14b((tick * 1715)/1000);
		int32_t q = wrap_2pi_14b(gl_theta_rem_m);
		int32_t e = wrap_2pi_14b(qd-q) >> 2;	//go down to 12
		int32_t vq = ctl_PI_fixed_2(e, (fixed_PI_2_params_t*)(&pctl_vq));
		int32_t damping = (gl_dtheta_fixedpoint_rad_p_sec * pctl_kd_vq.i32) >> pctl_kd_vq.radix;
		vq -= damping;
		vq = saturate(vq, 2047);

//		int32_t id_req = (sin_12b(wrap_2pi_12b(tick*10))*1200)/4096;
//		uint32_t tsince = tick-create_error_ts;
//		if(tsince > 0 && tsince < 1000)
//		{
//			vq = 1000;
//		}
//		else if (tsince > 3000)
//		{
//			create_error_ts = tick;
//		}
		sinusoidal_fixed(vq,0);
//		vq = 1000;
//		foc_fixed(vq, 0);

		get_dtheta();

//		tau = (sin_12b(wrap_2pi_12b( (tick*TWO_PI_12B)/4096 ))*2000)/4096;


//		while(m_huart2.bytes_to_send != 0);	//spin until uart is ready again. blocking uart
		if(m_huart2.bytes_to_send == 0 && (tick - uart_tx_ts) > 3 )
		{
			uart_tx_ts = tick;

			uint32_t * pwb32 = (uint32_t*)(&gl_working_buf[0]);
			int i32widx = 0;
			pwb32[i32widx++] = qd;
			pwb32[i32widx++] = gl_theta_rem_m;
			pwb32[i32widx++] = pctl_vq.x >> pctl_vq.ki.radix;
			pwb32[i32widx++] = vq;
			pwb32[i32widx++] = gl_dtheta_fixedpoint_rad_p_sec;

//			pwb32[i32widx++] =  ((int16_t)ADC1->JDR1) - 1990;
//			pwb32[i32widx++] =  ((int16_t)ADC1->JDR2) - 1990;
//			pwb32[i32widx++] =  ((int16_t)ADC1->JDR3) - 1990;

//			pwb32[i32widx++] = theta_elec;
//			pwb32[i32widx++] = vq;
//			pwb32[i32widx++] = gl_iq;

//			pwb32[i32widx++] = 0;
//			pwb32[i32widx++] = id_req;
//			pwb32[i32widx++] = gl_id;
//			pwb32[i32widx++] = 300;
//			pwb32[i32widx++] = gl_iq;

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
