/*
 * uart_buffers.c
 *
 *  Created on: Jan 7, 2026
 *      Author: Ocanath Robotman
 */
#include "uart_buffers.h"


uint8_t gl_uart1_rx_buf[UART_BUF_SIZE] = {};
uint8_t gl_uart1_rx_decoded_buf[UART_BUF_SIZE] = {};
uint8_t gl_uart1_tx_buf[UART_BUF_SIZE] = {};

uint8_t gl_uart2_rx_buf[UART_BUF_SIZE] = {};
uint8_t gl_uart2_rx_decoded_buf[UART_BUF_SIZE] = {};
uint8_t gl_uart2_tx_buf[UART_BUF_SIZE] = {};

dma_uart_t m_huart1;
dma_uart_t m_huart2;

