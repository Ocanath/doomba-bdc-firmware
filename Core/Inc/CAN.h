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

typedef struct ext_cmd_flags_t
{
	uint8_t en_blink_led;
	uint8_t use_uart_encoder;
}ext_cmd_flags_t;
extern ext_cmd_flags_t gl_ext_cmd_flags;
extern const uint16_t ext_cmd_id;

extern CAN_TxHeaderTypeDef   can_tx_header;
extern CAN_RxHeaderTypeDef   can_rx_header;
extern uint32_t			can_tx_mailbox;

void CAN_Init(void);
void handle_ext_cmd(ext_cmd_flags_t * flags, uint8_t d[sizeof(float)]);


#endif /* INC_CAN_H_ */
