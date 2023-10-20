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

#define PRESTUFF_BUF_SIZE 64
#define STUFF_BUF_SIZE	((2*PRESTUFF_BUF_SIZE)+2)
uint8_t gl_working_buf[PRESTUFF_BUF_SIZE] = {0};
uint8_t gl_ppp_stuffed_buf[STUFF_BUF_SIZE] = {0};


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

//	fds_mp.elec_conv_ratio_fixed = 7;
//	m_write_flash((uint64_t*)&fds_mp,sizeof(fds_motor_params_t)/sizeof(uint64_t));
#ifdef GET_ENCODER_MIDPOINT
	obtain_encoder_midpoints();
	fds_mp.gl_oerr_total = estimate_orthogonality_error();
	eval_encoder_params();
	m_write_flash((uint64_t*)&fds_mp,sizeof(fds_motor_params_t)/sizeof(uint64_t));
#endif
#ifdef GET_INIT_OFFSET
	auto_calibrate_motor(2000, 700, 5.f,.00001);
	m_write_flash((uint64_t*)&fds_mp,sizeof(fds_motor_params_t)/sizeof(uint64_t));
#endif
#ifdef TEST_FOC
	volatile int32_t error = openloop_eang_pctl(0,0xFFFFFFFF);
	TIMER_UPDATE_DUTY(500,500,500);
	for(uint32_t end_ts = HAL_GetTick() + 5000; HAL_GetTick() < end_ts;)
	{
		if(error < 714 && error > -714)	//714 is 10 degrees
		{
			//indicate pass
			test_foc();
		}
	}
#endif

//	int32_t vq_in = 3546;
	int32_t tau = 2000;
	while(1)
	{
		uint32_t tick = HAL_GetTick();

//		sinusoidal_fixed(vq_in,0);
//		tau = (sin_12b(wrap_2pi_12b( (tick*TWO_PI_12B)/4096 ))*2000)/4096;
		foc_fixed(tau, 0);

//		while(m_huart2.bytes_to_send != 0);	//spin until uart is ready again. blocking uart
		if(m_huart2.bytes_to_send == 0)
		{
			uint32_t * pwb32 = (uint32_t*)(&gl_working_buf[0]);
			int i32widx = 0;
	//		pwb32[i32widx++] = theta_elec;
//			pwb32[i32widx++] = tau;
			pwb32[i32widx++] = gl_iq;
			pwb32[i32widx++] = gl_id;
			pwb32[i32widx++] = HAL_GetTick();
			int pld_size = i32widx*sizeof(uint32_t);
			int stuffed_length = PPP_stuff( (uint8_t*)(&gl_working_buf[0]), pld_size, gl_ppp_stuffed_buf, STUFF_BUF_SIZE );
			m_uart_tx_start((uart_it_t*)(&m_huart2), gl_ppp_stuffed_buf, stuffed_length);
		}

		if(tick-led_ts > 333)
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
			led_ts = tick;
		}
	}
}
