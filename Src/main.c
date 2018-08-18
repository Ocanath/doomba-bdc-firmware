#include "main.h"
#include "stm32f0xx_hal.h"
#include "adc.h"
#include "delay_uS.h"
#include "commutation.h"
#include "comm.h"
#include "foc_commutation.h"
#include "mag-encoder.h"
#include "i2c.h"
#include <string.h>


#define BRAKE 0
#define STOP 1
#define FORWARD_OPEN 2
#define FORWARD_CLOSED 3
#define BACKWARD_OPEN 4
#define BACKWARD_CLOSED 5


#define MAX_CUR 60.0

/*
 * Since I don't have a .README yet... i'll just put the revision notes up here
 *
 * Tested i2c position control!!! current implementation uses memcpy and works directly with floats
 * I think memcpy is faster than division, but i have not benchmarked
 * 	- note: Fastest version may be to use pointers?: i.e. float->uint32_t is point a uint32_t * to a float??
 *
 *TODO: i2c fast mode
 *		alternative to memcpy/benchmark memcpy
 *		implement on multiple legs!
 *
 *NOTE: if bandwidth becomes an issue in the future, the only acceptable solution is likely SPI or CAN
 *		since foc boards are spi capable, this is an easy to do but horribly inconvenient switch
 */


/*
char msg_buf[32];
void print_string(char * c)
{
	int i;
	for(i=0; c[i] != 0; i++);
	HAL_UART_Transmit(&huart1, (uint8_t*)c, i, 100);
}
void print_int16(int16_t val)
{
	msg_buf[0] = (val & 0x00FF);
	msg_buf[1] = (val & 0xFF00)>>8;
	HAL_UART_Transmit(&huart1, (uint8_t*)msg_buf, 2, 100);
}
void print_int32(int32_t val)
{
	msg_buf[0] = (val & 0x00FF);
	msg_buf[1] = (val & 0xFF00)>>8;
	msg_buf[2] = (val & 0xFF0000)>>16;
	msg_buf[3] = (val & 0xFF000000)>>24;
	HAL_UART_Transmit(&huart1, (uint8_t*)msg_buf, 4, 100);
}
 */

int led_state =0;

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	parse_master_cmd();
	execute_master_cmd();
}

int idx = 0;
volatile int total_delay = 0;

float gl_angle = 89;

typedef enum {ADVANCE_TO_ZERO = 0, REST_AT_ZERO = 1, ADVANCE_TO_ANGLE = 2, REST_AT_ANGLE = 3} bang_test_state;

int main(void)
{
	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC_Init();
	MX_I2C1_Init();
	//	MX_SPI1_Init();
	//	MX_USART1_UART_Init();
	MX_TIM1_Init();
	MX_TIM14_Init();

	HAL_TIM_Base_Start(&htim14);
	HAL_TIM_PWM_Start_IT(&htim14, TIM_CHANNEL_1);

	start_pwm();
	TIMER_UPDATE_DUTY(0,0,0);
	TIM1->CCR4 = 650;	//for 7_5, you have about 8uS of sampling.you want to catch the current waveform right at the middle

	HAL_ADC_Start_DMA(&hadc, (uint32_t *)dma_adc_raw, NUM_ADC);
	//	HAL_SPI_TransmitReceive_DMA(&hspi1, t_data, r_data,2);	//think need to change DMA settings to word from byte or half word

	HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);

	HAL_Delay(100);
	HAL_GPIO_WritePin(CAL_PORT, CAL_PIN, 1);
	HAL_Delay(100);
	HAL_GPIO_WritePin(CAL_PORT, CAL_PIN, 0);
	HAL_Delay(100);

	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
	get_current_cal_offsets();
	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);

	init_observer();

	int led_ts = HAL_GetTick()+100;
	int led_state = 1;

	gl_angle = 0;

#ifdef GET_INIT_OFFSET
	obtain_encoder_offset();
#endif

	TIMER_UPDATE_DUTY(0,0,0);

	float x_iq_PI = 0;	//torque pi state
	float x_id_PI = 0;	//flux pi state
	float uq = 0;
	float ud = 0;

	float iqref = 0;
	float idref = 0;
	float theta_ref = 2*M_PI/4;
	float p_theta = 0;
	float tprev = 0;
	float tstart = time_seconds();



	while(1)
	{
		handle_i2c();
		uint32_t data_in = ((i2c_rx_data[0] << 24) | (i2c_rx_data[1] << 16) | (i2c_rx_data[2] << 8) | (i2c_rx_data[3]));
		memcpy(&iqref,&data_in,4);
		if(iqref > MAX_CUR)
			iqref = MAX_CUR;
		else if (iqref < -MAX_CUR)
			iqref = -MAX_CUR;
		if(HAL_GetTick() > recieved_data_ts + 100)
			iqref = 0;

		float i_a,i_b,i_c;
		conv_raw_current(&i_a,&i_b, &i_c);

		float i_alpha,i_beta;
		clarke_transform(i_a,i_b,i_c,&i_alpha, &i_beta);

		float theta_elec = theta_rel_rad()*NUM_POLE_PAIRS;		//test this! MS motor has 22 pole pairs, which means multiply by 11
		theta_elec = fmod_2pi(theta_elec + PI) - PI;		//re-modulate theta_m. ensure that the angle is constrained from -pi to pi!!
		float sin_theta = sin_fast(theta_elec);				//calculate the sin of the electrical (magnetic flux) angle
		float cos_theta = cos_fast(theta_elec);				//and the cosine for park and inverse park domains

		float i_q, i_d;
		park_transform(i_alpha, i_beta, sin_theta,cos_theta, &i_q, &i_d);

		float theta_m = unwrap(theta_abs_rad());
		uint32_t tx_data; //= (uint32_t)theta_m;
		memcpy(&tx_data,&theta_m,4);
		i2c_tx_data[0] = (tx_data & 0xFF000000) >> 24;
		i2c_tx_data[1] = (tx_data & 0x00FF0000) >> 16;
		i2c_tx_data[2] = (tx_data & 0x0000FF00) >> 8;
		i2c_tx_data[3] = (tx_data & 0x000000FF);


		controller_PI(iqref, i_q, 0.01, 0.0000000001, &x_iq_PI, &uq);		//this sort of works
		controller_PI(idref, i_d, 0.01, 0.0000000001, &x_id_PI, &ud);		//high current


		uint32_t tA,tB,tC;
		inverse_park_transform(uq, ud, sin_theta,cos_theta, &i_alpha, &i_beta);	//maybe call theta rel again?sin_theta
		svm(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);
		TIMER_UPDATE_DUTY(tA,tB,tC);

		if(HAL_GetTick()>=led_ts)
		{
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,led_state);
			led_state = !led_state & 1;
			led_ts = HAL_GetTick() + 200;
		}
	}

}


/*
 * 		old 'discrete' position controller (on driver)
 * 		//		theta_ref = .4*sin(time_seconds()*6+1.45)+HALF_PI;
		//		theta_ref = M_PI*.5 + sin(time_seconds()*6);
		//		theta_ref = 5*TWO_PI*sin(time_seconds()*3);

		//		theta_ref = .5*HALF_PI*sin((time_seconds()-tstart)*TWO_PI*2+1.57);
		//		theta_ref = .6*sin(time_seconds()*12);
		//		theta_ref = 0;
		//		float  err = theta_ref - theta_m;
		//		float dtheta = theta_m - p_theta;
		//		p_theta = theta_m;
		//		float dt = time_seconds()-tprev;
		//		tprev = time_seconds();
		//		iqref = 30*err - dtheta/dt*0.2;
		//		if(iqref > 40.0)
		//			iqref = 40.0;
		//		if(iqref < -40.0)
		//			iqref = -40.0;

		//		iqref = 5.0;
 *
 */
