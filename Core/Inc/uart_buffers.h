/*
 * uart_buffers.h
 *
 *  Created on: Jan 7, 2026
 *      Author: Ocanath Robotman
 */

#ifndef INC_UART_BUFFERS_H_
#define INC_UART_BUFFERS_H_
#include <stdint.h>
#include "m_dma_uart.h"

#define UART_BUF_SIZE 128

extern uint8_t gl_uart1_tx_buf[UART_BUF_SIZE];
extern uint8_t gl_uart1_rx_buf[UART_BUF_SIZE];
extern uint8_t gl_uart1_rx_decoded_buf[UART_BUF_SIZE];

extern uint8_t gl_uart2_tx_buf[UART_BUF_SIZE];
extern uint8_t gl_uart2_rx_buf[UART_BUF_SIZE];
extern uint8_t gl_uart2_rx_decoded_buf[UART_BUF_SIZE];


extern dma_uart_t m_huart1;
extern dma_uart_t m_huart2;


#endif /* INC_UART_BUFFERS_H_ */
