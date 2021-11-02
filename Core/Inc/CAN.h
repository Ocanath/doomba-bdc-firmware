/*
 * CAN.h
 *
 *  Created on: Oct 25, 2020
 *      Author: Ocanath Robotman
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_
#include "init.h"

#define PAYLOAD_SIZE_CAN 8
//no matter what, the HAL backend loads 8 bytes of CAN data from registers. So you better be
//ready to receive them

typedef enum {FOC_MODE, SINUSOIDAL_MODE} control_mode_t;	//foc with velocity?

typedef union
{
	uint8_t d[PAYLOAD_SIZE_CAN];
	int32_t i32[PAYLOAD_SIZE_CAN/sizeof(int32_t)];	//all types are even multiples of 8, and sizeof evals at compile time so this is safe
	uint32_t ui32[PAYLOAD_SIZE_CAN/sizeof(uint32_t)];
	int16_t i16[PAYLOAD_SIZE_CAN/sizeof(int16_t)];
	float f32[PAYLOAD_SIZE_CAN/sizeof(float)];
//	double f64[PAYLOAD_SIZE_CAN/sizeof(double)];	//can include if use. 1 element array thing kind of skeeves me out so im commenting it
}can_payload_t;

typedef union
{
	int16_t v;
	uint8_t d[sizeof(int16_t)];
}int16_fmt_t;

typedef union
{
	uint32_t v;
	uint8_t d[sizeof(uint32_t)];
}uint32_fmt_t;

typedef union
{
	int32_t v;
	uint8_t d[sizeof(int32_t)];
}int32_fmt_t;

typedef union floatsend_t
{
	float v;
	uint8_t d[sizeof(float)];
}floatsend_t;

typedef union floatsend_chk_t
{
	float v;
	uint8_t d[sizeof(float) + 1];
}floatsend_chk_t;


typedef struct ext_cmd_flags_t
{
	uint8_t en_blink_led;
	uint8_t use_uart_encoder;
	control_mode_t control_mode;
}ext_cmd_flags_t;



extern ext_cmd_flags_t gl_ext_cmd_flags;
extern const uint16_t ext_cmd_id;

extern CAN_TxHeaderTypeDef   can_tx_header;
extern CAN_RxHeaderTypeDef   can_rx_header;
extern uint32_t			can_tx_mailbox;

void CAN_Init(void);
void handle_ext_cmd(ext_cmd_flags_t * flags, uint8_t d[sizeof(float)]);


#endif /* INC_CAN_H_ */
