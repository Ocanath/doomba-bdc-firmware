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
#include <FDCAN.h>

#define DEFAULT_V_REPORT_RADIX	4
#define DEFAULT_IQ_RSHIFT		3	//estimated max value of iq is in the ~17bit domain. shift 3 to get it between 14-15bit

uint16_t gl_ext_cmd_id;

static int32_t v_radix = DEFAULT_V_REPORT_RADIX;
int32_t gl_v_uS_conv = 1000000 >> (14 - DEFAULT_V_REPORT_RADIX);
int32_t gl_iq_rshift = DEFAULT_IQ_RSHIFT;

FDCAN_TxHeaderTypeDef		can_tx_header;
FDCAN_RxHeaderTypeDef		can_rx_header;
can_payload_t 			can_tx_data = {0};
can_payload_t 			can_rx_data = {0};

uint32_t				can_tx_mailbox;




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

void FDCAN_Config(void)
{
	//This is all different for FD CAN
	FDCAN_FilterTypeDef sFilterConfig;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = (0x7FF - fds_mp->module_number);
	sFilterConfig.FilterID2 = fds_mp->module_number;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
		Error_Handler();
	}
	/* Start the FDCAN module */
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
		Error_Handler();
	}

	can_tx_header.Identifier = fds_mp->module_number;
	can_tx_header.IdType = FDCAN_STANDARD_ID;
	can_tx_header.TxFrameType = FDCAN_DATA_FRAME;
	can_tx_header.DataLength = FDCAN_DLC_BYTES_8;
	can_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	can_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	can_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
	can_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	can_tx_header.MessageMarker = 0;
}

