/*
 * CAN.h
 *
 *  Created on: Oct 25, 2020
 *      Author: Ocanath Robotman
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_
#include "init.h"
#include "fds.h"

#define PAYLOAD_SIZE_CAN 8
//no matter what, the HAL backend loads 8 bytes of CAN data from registers. So you better be
//ready to receive them

extern int32_t gl_v_uS_conv;	//used to calculate expression of rotor velocity in (2^radix)*radians/sec
extern int32_t gl_iq_rshift;

typedef enum {FOC_MODE, SINUSOIDAL_MODE, PCTL_IQ, PCTL_VQ} control_mode_t;	//foc with velocity?

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

typedef struct pctl_params_t
{
	fixed_PI_params_t kpki;
	i32_t kd;
	int32_t out_sat;
}pctl_params_t;

extern pctl_params_t mctl_iq;
extern pctl_params_t mctl_vq;

extern ext_cmd_flags_t gl_ext_cmd_flags;
extern uint16_t gl_ext_cmd_id;

extern FDCAN_TxHeaderTypeDef   can_tx_header;
extern FDCAN_RxHeaderTypeDef   can_rx_header;
extern uint32_t			can_tx_mailbox;

void CAN_Init(void);
void handle_ext_cmd(ext_cmd_flags_t * flags, uint8_t * d);


#endif /* INC_CAN_H_ */
