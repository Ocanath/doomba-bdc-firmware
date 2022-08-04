/*
 * CAN.c
 *
 *  Created on: Oct 25, 2020
 *      Author: Ocanath Robotman
 *
 *      CAN Motor Protocol Breakdown:
 *
 *      For 'NORMAL OPERATION': (torque and position)
 *      	Message ID will be equal to the serial number of the motor (i.e. 23)
 *      	Master controller sends a floating point (ieee 32bit spec) number, parsed as a torque command
 *      	Upon receiving the matching message ID, the motor will put its current position on the bus
 *
 *      For 'EXTENDED COMMAND': (miscellaneous settings)
 *      	Message ID will be equal to MAX ID (0x7FF) minus the motor serial number (i.e. 0x7FF - 23)
 *      	The byte index 3 will contain a word that corresponds to setting or resetting a given binary setting
 *      	Upon receiving this message ID, the motor will also respond with its current position on the bus.
 *
 */
#include "CAN.h"

#define DEFAULT_V_REPORT_RADIX	4
#define DEFAULT_IQ_RSHIFT		3	//estimated max value of iq is in the ~17bit domain. shift 3 to get it between 14-15bit

static int32_t v_radix = DEFAULT_V_REPORT_RADIX;
int32_t gl_v_uS_conv = 1000000 >> (12 - DEFAULT_V_REPORT_RADIX);
int32_t gl_iq_rshift = DEFAULT_IQ_RSHIFT;

CAN_TxHeaderTypeDef		can_tx_header;
CAN_RxHeaderTypeDef		can_rx_header;
can_payload_t 			can_tx_data = {0};
can_payload_t 			can_rx_data = {0};

uint32_t				can_tx_mailbox;

ext_cmd_flags_t gl_ext_cmd_flags = {
		.en_blink_led = 1,
		.use_uart_encoder = 1,
		.control_mode = SINUSOIDAL_MODE
};

/*Initialize params for position control using FOC as 0*/
pctl_params_t mctl_iq = {
		.kpki =
		{
			.kp = {.i32 = 0, .radix = 0},
			.ki = {.i32 = 0, .radix = 0},
			.x = 0,
			.x_sat = 0,
			.out_rshift = 0
		},
		.kd = {.i32 = 0, .radix = 0},
		.out_sat = 0,
};
/*Initialize params for position control using sinusoidal as 0*/
pctl_params_t mctl_vq = {
		.kpki =
		{
			.kp = {.i32 = 0, .radix = 0},
			.ki = {.i32 = 0, .radix = 0},
			.x = 0,
			.x_sat = 0,
			.out_rshift = 0
		},
		.kd = {.i32 = 0, .radix = 0},
		.out_sat = 0,
};

const uint16_t ext_cmd_id = (0x7FF - MODULE_NUMBER);

enum {

	CALC_ALIGN_OFFSET = 0x1,
	CALC_ENC_MIDPOINTS = 0x2,


	LED_ON = 0xDE,
	LED_OFF =0xFE,
	LED_BLINK= 0xAA,

	EN_UART_ENC = 0x34,
	DIS_UART_ENC = 0x35,

	SET_FOC_MODE = 0x36,	//configure to foc mode. need to do once and it takes
	SET_SINUSOIDAL_MODE = 0x37,	//configure to velocity mode. send once and it takes
	CHANGE_V_RADIX = 0x38,	//radix for velocity expression setting
	CHANGE_IQ_RSHIFT = 0x39,	//rightshift for iq expression setting

	SET_PCTL_IQ_MODE = 0x40,	//configure to position control using FOC mode. send once and it takes. Needs all position settings written to do anything (stable)
	CHANGE_PCTL_IQ_KP_VALUE = 0x41,
	CHANGE_PCTL_IQ_KP_RADIX = 0x42,
	CHANGE_PCTL_IQ_KI_VALUE = 0x43,
	CHANGE_PCTL_IQ_KI_RADIX = 0x44,
	CHANGE_PCTL_IQ_KD_VALUE = 0x45,
	CHANGE_PCTL_IQ_KD_RADIX = 0x46,
	CHANGE_PCTL_IQ_XSAT = 0x47,
	CHANGE_PCTL_IQ_OUTSAT = 0x48,
	CHANGE_PCTL_IQ_OUT_RSHIFT = 0x49,


	SET_PCTL_VQ_MODE = 0x4A,	//configure to position control using sinusoidal mode. send once and it takes. Needs all position settings written to do anything (stable)
	CHANGE_PCTL_VQ_KP_VALUE = 0x4B,
	CHANGE_PCTL_VQ_KP_RADIX = 0x4C,
	CHANGE_PCTL_VQ_KI_VALUE = 0x4D,
	CHANGE_PCTL_VQ_KI_RADIX = 0x4E,
	CHANGE_PCTL_VQ_KD_VALUE = 0x4F,
	CHANGE_PCTL_VQ_KD_RADIX = 0x50,
	CHANGE_PCTL_VQ_XSAT = 0x51,
	CHANGE_PCTL_VQ_OUTSAT = 0x52,
	CHANGE_PCTL_VQ_OUT_RSHIFT = 0x53
};

void CAN_Init(void)
{
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
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;	//use id list
	sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;	//16 bit for idlist
	sFilterConfig.FilterIdHigh = ext_cmd_id<<5;	//this is the generic message id, for controlling encoder input and the LED
	sFilterConfig.FilterIdLow = (MODULE_NUMBER)<<5;	//this is the work id, for torque and position commands
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
	can_tx_header.StdId = MODULE_NUMBER;
	can_tx_header.ExtId = 0x00;
	can_tx_header.RTR = CAN_RTR_DATA;
	can_tx_header.IDE = CAN_ID_STD;
	can_tx_header.DLC = sizeof(int32_t) + sizeof(int16_t);	//6
	can_tx_header.TransmitGlobalTime = DISABLE;

	can_rx_header.StdId = 0x000;	//gets loaded by getmessage
	can_rx_header.ExtId = 0x00;
	can_rx_header.IDE = CAN_ID_STD;
	can_rx_header.RTR = CAN_RTR_DATA;
	can_rx_header.DLC = sizeof(int16_t);	//2
	can_rx_header.Timestamp = 0;
	can_rx_header.FilterMatchIndex = 0;
}

void handle_ext_cmd(ext_cmd_flags_t * flags, uint8_t * d)
{
	uint8_t cmd = d[0];

	/*Hard load fmt. only valid for certain commands, but it's not expensive to load it into
	 * our type pun structure every time, and makes this marginally easier to read*/
	int32_fmt_t fmt;
	for(int i = 0; i < sizeof(uint32_t); i++)
	{
		fmt.d[i] = d[i+1];	//load the remainder of the message as the word
	}

	switch(cmd)	//NOTE! changed from d[1]! this breaks old misc communication strat!
	{
		case LED_ON:
		{
			flags->en_blink_led = 0;
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
			break;
		}
		case LED_OFF:
		{
			flags->en_blink_led = 0;
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
			break;
		}
		case LED_BLINK:
			flags->en_blink_led = 1;
			break;
		case EN_UART_ENC:
			flags->use_uart_encoder = 1;
			break;
		case DIS_UART_ENC:
			flags->use_uart_encoder = 0;
			break;
		case SET_FOC_MODE:
			gl_ext_cmd_flags.control_mode = FOC_MODE;
			break;
		case SET_SINUSOIDAL_MODE:
			gl_ext_cmd_flags.control_mode = SINUSOIDAL_MODE;
			break;
		case CHANGE_V_RADIX:
		{
			v_radix = d[1];
			gl_v_uS_conv = 1000000 >> (12 - v_radix);
			break;
		}
		case CHANGE_IQ_RSHIFT:
		{
			gl_iq_rshift = d[1];
			break;
		}

		/*THIS SECTION IS FOR IQ POS CONTROL MODE*/
		case SET_PCTL_IQ_MODE:
			gl_ext_cmd_flags.control_mode = POS_IQ_AENC;
			break;
		case CHANGE_PCTL_IQ_KP_VALUE:
			mctl_iq.kpki.kp.i32 = fmt.v;
			break;
		case CHANGE_PCTL_IQ_KP_RADIX:
			mctl_iq.kpki.kp.radix = d[1];
			break;
		case CHANGE_PCTL_IQ_KI_VALUE:
			mctl_iq.kpki.ki.i32 = fmt.v;
			break;
		case CHANGE_PCTL_IQ_KI_RADIX:
			mctl_iq.kpki.ki.radix = d[1];
			break;
		case CHANGE_PCTL_IQ_KD_VALUE:
			mctl_iq.kd.i32 = fmt.v;
			break;
		case CHANGE_PCTL_IQ_KD_RADIX:
			mctl_iq.kd.radix = d[1];
			break;
		case CHANGE_PCTL_IQ_XSAT:
			mctl_iq.kpki.x_sat = fmt.v;
			break;
		case CHANGE_PCTL_IQ_OUTSAT:
			mctl_iq.out_sat = ((uint32_t)fmt.v) << gl_iq_rshift;
			break;
		case CHANGE_PCTL_IQ_OUT_RSHIFT:
			mctl_iq.kpki.out_rshift = d[1];
			break;

		/*THIS SECTION IS FOR VQ POS CONTROL MODE*/
		case SET_PCTL_VQ_MODE:
			gl_ext_cmd_flags.control_mode = POS_VQ_AENC;
			break;
		case CHANGE_PCTL_VQ_KP_VALUE:
			mctl_vq.kpki.kp.i32 = fmt.v;
			break;
		case CHANGE_PCTL_VQ_KP_RADIX:
			mctl_vq.kpki.kp.radix = fmt.v;
			break;
		case CHANGE_PCTL_VQ_KI_VALUE:
			mctl_vq.kpki.ki.i32 = fmt.v;
			break;
		case CHANGE_PCTL_VQ_KI_RADIX:
			mctl_vq.kpki.ki.radix = d[1];
			break;
		case CHANGE_PCTL_VQ_KD_VALUE:
			mctl_vq.kd.i32 = fmt.v;
			break;
		case CHANGE_PCTL_VQ_KD_RADIX:
			mctl_vq.kd.radix = d[1];
			break;
		case CHANGE_PCTL_VQ_XSAT:
			mctl_vq.kpki.x_sat = fmt.v;
			break;
		case CHANGE_PCTL_VQ_OUTSAT:
			mctl_vq.out_sat = fmt.v;
			break;
		case CHANGE_PCTL_VQ_OUT_RSHIFT:
			mctl_vq.kpki.out_rshift = d[1];
			break;

	}
}
