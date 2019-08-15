/*
 * i2c.h
 *
 *  Created on: Feb 9, 2018
 *      Author: Ocanath
 */

#ifndef I2C_H_
#define I2C_H_

#include "init.h"

typedef union
{
	float v;
	uint8_t d[4];
}floatsend_t;

#define I2C_MODE_WAIT 0
#define I2C_MODE_TRANSMIT 1
#define I2C_MODE_RECIEVE 2
#define I2C_MODE_BUSY 3

#define I2C_DIRECTION_MASTER_TRANSMIT 0
#define I2C_DIRECTION_MASTER_RECIEVE 1
uint8_t gl_i2c_mode;

#define TX_SIZE 4
#define RX_SIZE 4
//uint8_t i2c_tx_data[TX_SIZE];
//uint8_t i2c_rx_data[RX_SIZE];
extern floatsend_t i2c_tx_data;
extern floatsend_t i2c_rx_data;

uint32_t recieved_data_ts;

uint8_t handle_i2c(I2C_HandleTypeDef * hi2c);
void I2C1_Reset();

enum {LED_ON, LED_OFF, ENABLE_ON, ENABLE_OFF, BLINK_ENABLE};
inline void parse_master_cmd(uint32_t * led_ts)
{
	uint32_t rx_cast = (uint32_t)i2c_rx_data.v;
	if((rx_cast & 0xFFF00000) == 0x7fc00000)	//if we've been fed a NaN, check it for commands. we sneaky like that
	{
		uint32_t cmd = rx_cast & 0x000FFFFF;
		switch(cmd)
		{
		case LED_ON:
		{
			*led_ts = 0xFFFFFFFF;
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
			break;
		}
		case LED_OFF:
		{
			*led_ts = 0xFFFFFFFF;
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,0);
			break;
		}
		case BLINK_ENABLE:
		{
			*led_ts = HAL_GetTick()+200;
			break;
		}
		default:
			break;
		};
	}
}


#endif /* I2C_H_ */
