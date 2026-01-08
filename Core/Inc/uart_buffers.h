/*
 * uart_buffers.h
 *
 *  Created on: Jan 7, 2026
 *      Author: Ocanath Robotman
 */

#ifndef INC_UART_BUFFERS_H_
#define INC_UART_BUFFERS_H_
#include <stdint.h>

#define PRESTUFF_BUF_SIZE 64
#define STUFF_BUF_SIZE	((2*PRESTUFF_BUF_SIZE)+2)
extern uint8_t gl_working_buf[PRESTUFF_BUF_SIZE];
extern uint8_t gl_ppp_stuffed_buf[STUFF_BUF_SIZE];


#endif /* INC_UART_BUFFERS_H_ */
