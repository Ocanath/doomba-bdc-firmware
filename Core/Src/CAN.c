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

CAN_TxHeaderTypeDef		can_tx_header;
CAN_RxHeaderTypeDef		can_rx_header;
uint32_fmt_t 			can_tx_data = {0x55555555};
uint32_fmt_t 			can_rx_data = {0};
uint32_t				can_tx_mailbox;

ext_cmd_flags_t gl_ext_cmd_flags = {
		.en_blink_led = 1,
		.use_uart_encoder = 1,
		.control_mode = FOC_MODE
};

const uint16_t ext_cmd_id = (0x7FF - MODULE_NUMBER);

enum {
	LED_ON = 0xDE,
	LED_OFF =0xFE,
	LED_BLINK= 0xAA,
	EN_UART_ENC = 0x34,
	DIS_UART_ENC = 0x35,
	SET_FOC_MODE = 0x36,
	SET_SINUSOIDAL_MODE = 0x37
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

void handle_ext_cmd(ext_cmd_flags_t * flags, uint8_t d[sizeof(float)])
{
	if(d[3] == LED_ON)
	{
		flags->en_blink_led = 0;
		HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
	}
	else if (d[3] == LED_OFF)
	{
		flags->en_blink_led = 0;
		HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
	}
	else if (d[3] == LED_BLINK)
		flags->en_blink_led = 1;
	else if (d[3] == EN_UART_ENC)
		flags->use_uart_encoder = 1;
	else if (d[3] == DIS_UART_ENC)
		flags->use_uart_encoder = 0;
}
