/*
 * CAN.c
 *
 *  Created on: Oct 25, 2020
 *      Author: Ocanath Robotman
 */
#include "CAN.h"

CAN_TxHeaderTypeDef		can_tx_header;
CAN_RxHeaderTypeDef		can_rx_header;
uint32_fmt_t 			can_tx_data = {0x55555555};
uint32_fmt_t 			can_rx_data = {0};
uint32_t				can_tx_mailbox;

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
	sFilterConfig.FilterIdHigh = 0x0001<<5;	//this is the (unimplemented) generic message id? idk
	sFilterConfig.FilterIdLow = (MODULE_NUMBER)<<5;	//this is the work id
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
	can_tx_header.DLC = sizeof(float);	//4
	can_tx_header.TransmitGlobalTime = DISABLE;

	can_rx_header.StdId = 0x000;	//gets loaded by getmessage
	can_rx_header.ExtId = 0x00;
	can_rx_header.IDE = CAN_ID_STD;
	can_rx_header.RTR = CAN_RTR_DATA;
	can_rx_header.DLC = sizeof(float);	//4
	can_rx_header.Timestamp = 0;
	can_rx_header.FilterMatchIndex = 0;
}
