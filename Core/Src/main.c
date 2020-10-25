
#include "main.h"
#include "init.h"
#include "adc.h"
#include "foc_commutation.h"
#include "mag-encoder.h"
#include "settings.h"
#include "calibration.h"

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

typedef union floatsend_t
{
	float v;
	uint8_t d[sizeof(float)];
}floatsend_t;

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

typedef struct nfault_handle_flags_t
{
	uint8_t fe_flag;	//falling edge
	uint32_t ts;
	uint8_t ts_flag;
}nfault_handle_flags_t;

//uint8_t gl_fault_flag = 0;
nfault_handle_flags_t gl_nfault_flags = {0};

/**
 * @brief  EXTI line detection callback.
 * @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line.
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == NFAULT_Pin)
	{
		gl_nfault_flags.fe_flag = 1;
	}
}

typedef union
{
	uint32_t v;
	uint8_t d[sizeof(uint32_t)];
}uint32_fmt_t;

CAN_TxHeaderTypeDef   can_tx_header;
CAN_RxHeaderTypeDef   can_rx_header;
uint8_t               can_tx_data[4] = {0xDE,0xAD,0xBE,0xEF};//,77,23,0x09,0xBE};
//uint8_t               can_rx_data[4] = {0};
uint32_fmt_t can_rx_data = {0};
uint32_t              TxMailbox;


int main(void)
{
	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_DMA_Init();

	HAL_Delay(100);
	CAN_FilterTypeDef  sFilterConfig;

	/*##-1- Configure the CAN peripheral #######################################*/
	hcan.Instance = CAN;

	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_9TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_8TQ;
	hcan.Init.Prescaler = 16;

	if (HAL_CAN_Init(&hcan) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}

	/*##-2- Configure the CAN Filter ###########################################*/
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0xFFFF;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}

	/*##-3- Start the CAN peripheral ###########################################*/
	if (HAL_CAN_Start(&hcan) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}

	/*##-4- Activate CAN RX notification #######################################*/
	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		/* Notification Error */
		Error_Handler();
	}

	/*##-5- Configure Transmission process #####################################*/
	can_tx_header.StdId = 0x002;
	can_tx_header.ExtId = 0x00;
	can_tx_header.RTR = CAN_RTR_DATA;
	can_tx_header.IDE = CAN_ID_STD;
	can_tx_header.DLC = 4;
	can_tx_header.TransmitGlobalTime = DISABLE;

	can_tx_data[0] = 0x55;
	can_tx_data[1] = 0x55;
	can_tx_data[2] = 0x55;
	can_tx_data[3] = 0x55;

	can_rx_header.StdId = 0x003;
	can_rx_header.ExtId = 0x00;
	can_rx_header.IDE = CAN_ID_STD;
	can_rx_header.RTR = CAN_RTR_DATA;
	can_rx_header.DLC = 4;
	can_rx_header.Timestamp = 0;
	can_rx_header.FilterMatchIndex = 0;


	MX_ADC1_Init();
	MX_TIM1_Init();
	MX_USART2_UART_Init();

	HAL_Delay(50);
	HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
	uint32_t tx_ts = 0;
	while(1)
	{
		if(HAL_GetTick()>tx_ts)
		{
			HAL_CAN_AddTxMessage(&hcan, &can_tx_header, can_tx_data, &TxMailbox);
			tx_ts = HAL_GetTick()+10;
		}
		if(HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) >= 1)
		{
			if(HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &can_rx_header, can_rx_data.d) != HAL_OK)
			{}

			if(can_rx_data.v == 0xDEADBEEF)
				HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
			else if (can_rx_data.v == 0xFEEDDEED)
				HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
		}
		//HAL_Delay(10);
	}

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

	while(1)
	{
		for(uint32_t run_ts = HAL_GetTick()+1000;HAL_GetTick()<run_ts;)
		{
			sinusoidal(0.5f,0);

			if(gl_nfault_flags.fe_flag)
			{
				if(HAL_GPIO_ReadPin(NFAULT_GPIO_Port,NFAULT_Pin) == 1)
				{
					gl_nfault_flags.fe_flag = 0;
					gl_nfault_flags.ts_flag = 0;
				}
				else
				{
					if(gl_nfault_flags.ts_flag == 0)
					{
						gl_nfault_flags.ts = HAL_GetTick();
						gl_nfault_flags.ts_flag = 1;
					}
					else if (HAL_GetTick() - gl_nfault_flags.ts > 10)
					{
						HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 0);
						unwrap( theta_rel_rad(), &theta_enc_prev);
						HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);
						gl_nfault_flags.ts_flag = 0;
						gl_nfault_flags.fe_flag = 0;
					}
				}
			}

		}
		for(uint32_t run_ts = HAL_GetTick()+1000;HAL_GetTick()<run_ts;)
		{
			sinusoidal(-0.5f,0);
			//			if(gl_fault_flag)
			//			{
			//				HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 0);
			//				unwrap( theta_rel_rad(), &theta_enc_prev);
			//				HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);
			//				gl_fault_flag = 0;
			//			}
		}
	}

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

	floatsend_t uart_encoder_data;
	uart_encoder_data.v = 0.f;
	floatsend_t tau_data_in;
	floatsend_t q_data_out;

	force_encoder_region();
	if(theta_enc_prev < -PI)
		theta_enc_prev += TWO_PI*2.0f;
	//	theta_m_prev=theta_enc_prev;

	//	uint32_t uart_tx_ts = 0;
	uint32_t dq_ts = 0;
	float prev_theta_m_dq = 0;
	float phase_advance_angle = 0;
	//uint8_t use_uart_encoder = 0;	//default to not using uart encoder stream
	while(1)
	{
		uint32_t * fptr_chk = (uint32_t*)(&(tau_data_in.v));	//check to make sure your input torque is not NaN
		if( (*fptr_chk & 0x7f800000) == 0x7f800000)	//NaN check. can put else in here for future i2c checks and whatnot
			iqref = 0;
		else
			iqref = tau_data_in.v;
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
		controller_PI(idref, i_d, id_gain, 0.0f, &x_id_PI, &ud);		//high current


		uint32_t tA,tB,tC;
		if(uq > 0.7f)
			uq = 0.7f;
		if(uq < - 0.7f)
			uq = -0.7f;
		inverse_park_transform(uq, ud, sin_theta,cos_theta, &i_alpha, &i_beta);	//maybe call theta rel again?sin_theta
		svm(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);
		TIMER_UPDATE_DUTY(tA,tB,tC);
		/**********************************************END**********************************************/

		float theta_m = theta_enc*.5f;//unwrap(theta_abs_rad(), &theta_m_prev)*.5f;

		//		if(use_uart_encoder == 0)
#ifndef USE_UART_ENCODER
		q_data_out.v = theta_m;
#else
		HAL_UART_Receive_IT(&huart2, uart_encoder_data.d, 4);
		if(gl_uart_cplt_flag == 1)	//should work
		{
			q_data_out.v = uart_encoder_data.v;
			gl_uart_cplt_flag=0;
		}
#endif

		if(HAL_GetTick() > dq_ts) //in this implementation, mechanical theta is computed outside the loop and shared with foc.
		{
			float dq = (theta_m-prev_theta_m_dq)*25.0f;
			prev_theta_m_dq = theta_m;
			phase_advance_angle = dq/gl_prop_delay_const;//division yields more accurate advance angle for better performance.
			dq_ts = HAL_GetTick()+40;


			/*
			 * TODO: swap the two drivers (my pop and pcbgogo mod). compare against module 019
			 */
			if(abs_f(gl_dq) < 20.f && abs_f(iqref) > 5.f && abs_f(i_q_meas) < 1.f)
			{
				HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 0);
				unwrap( theta_rel_rad(), &theta_enc_prev);
				HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);
			}
		}

		if(HAL_GetTick()>=led_ts)
		{
			HAL_GPIO_TogglePin(STAT_PORT,STAT_PIN);
			led_ts = HAL_GetTick() + 200;
		}
	}

}
