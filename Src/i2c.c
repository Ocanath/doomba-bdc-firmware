/*
 * i2c.c
 *
 *  Created on: Feb 9, 2018
 *      Author: Ocanath
 */
#include "i2c.h"

uint8_t gl_i2c_mode = I2C_MODE_WAIT;
/*
 * interrupt handler callback for when address is recieved.
 */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	if(TransferDirection == I2C_DIRECTION_MASTER_TRANSMIT && gl_i2c_mode != I2C_MODE_BUSY)
	{
		gl_i2c_mode = I2C_MODE_RECIEVE;
	}
	else if (TransferDirection == I2C_DIRECTION_MASTER_RECIEVE && gl_i2c_mode != I2C_MODE_BUSY)
	{
		gl_i2c_mode = I2C_MODE_TRANSMIT;
	}
}

uint32_t recieved_data_ts = 0;

uint8_t i2c_tx_data[TX_SIZE] = {0};
uint8_t i2c_rx_data[RX_SIZE] = {0};
uint32_t listen_lock_ts = 0;
/*
 * called in main loop, performs state machine which will manage transmission and reception while properly managing the address interrupt.
 */
void handle_i2c(I2C_HandleTypeDef * hi2c)
{
	HAL_I2C_EnableListen_IT(hi2c); //probably only need 1 call to this.

	if(hi2c->State != HAL_I2C_STATE_BUSY_TX_LISTEN && hi2c->State != HAL_I2C_STATE_BUSY_RX_LISTEN && hi2c->State != HAL_I2C_STATE_LISTEN)
		listen_lock_ts = HAL_GetTick();

	if(gl_i2c_mode == I2C_MODE_TRANSMIT)
	{
		HAL_I2C_Slave_Sequential_Transmit_IT(&hi2c1, i2c_tx_data, TX_SIZE, I2C_FIRST_AND_LAST_FRAME);
		gl_i2c_mode = I2C_MODE_WAIT;
	}
	else if(gl_i2c_mode == I2C_MODE_RECIEVE)
	{
		HAL_I2C_Slave_Sequential_Receive_IT(&hi2c1, i2c_rx_data, RX_SIZE, I2C_FIRST_AND_LAST_FRAME);
		gl_i2c_mode = I2C_MODE_WAIT;
		recieved_data_ts = HAL_GetTick();
	}
	else
	{
		if(HAL_GetTick() - listen_lock_ts > 15)	//if you've been listening for a while for some instructions, but ur master has left you hanging, punch him in his fatass bitch mouth kill your fucking masters
		{
			//do something
			HAL_GPIO_WritePin(STAT_PORT,STAT_PIN,1);
		}

	}
}

/*
 * optional wrapper for rx data
 * OUTPUTS: rx_data array by reference
 */
void get_i2c_rx(uint8_t * rx_data)
{
	int i;
	for(i=0;i<RX_SIZE;i++)
		rx_data[i] = i2c_rx_data[i];
}
/*
 * optional wrapper for tx data
 * OUTPUTS: tx_data array by reference
 */
void get_i2c_tx(uint8_t * tx_data)
{
	int i;
	for(i=0;i<RX_SIZE;i++)
		tx_data[i] = i2c_tx_data[i];
}
