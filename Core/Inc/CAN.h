/*
 * CAN.h
 *
 *  Created on: Oct 25, 2020
 *      Author: Ocanath Robotman
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_
#include "init.h"

typedef union
{
	uint32_t v;
	uint8_t d[sizeof(uint32_t)];
}uint32_fmt_t;

typedef union floatsend_t
{
	float v;
	uint8_t d[sizeof(float)];
}floatsend_t;

extern CAN_TxHeaderTypeDef   can_tx_header;
extern CAN_RxHeaderTypeDef   can_rx_header;
extern uint32_t			can_tx_mailbox;

void CAN_Init(void);


#endif /* INC_CAN_H_ */
