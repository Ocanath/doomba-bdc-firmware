/*
 * uart_buffers.c
 *
 *  Created on: Jan 7, 2026
 *      Author: Ocanath Robotman
 */
#include "uart_buffers.h"

uint8_t gl_working_buf[PRESTUFF_BUF_SIZE] = {0};
uint8_t gl_ppp_stuffed_buf[STUFF_BUF_SIZE] = {0};

