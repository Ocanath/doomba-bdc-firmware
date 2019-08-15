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

floatsend_t i2c_tx_data;
floatsend_t i2c_rx_data;
uint32_t listen_lock_ts = 0;

/*
 * called in main loop, performs state machine which will manage transmission and reception while properly managing the address interrupt.
 */
uint8_t handle_i2c(I2C_HandleTypeDef * hi2c)
{
	HAL_I2C_EnableListen_IT(hi2c); //probably only need 1 call to this.

	if(hi2c->State != HAL_I2C_STATE_BUSY_TX_LISTEN && hi2c->State != HAL_I2C_STATE_BUSY_RX_LISTEN && hi2c->State != HAL_I2C_STATE_LISTEN)
		listen_lock_ts = HAL_GetTick();

	if(gl_i2c_mode == I2C_MODE_TRANSMIT)
	{
		HAL_I2C_Slave_Sequential_Transmit_IT(&hi2c1, i2c_tx_data.d, TX_SIZE, I2C_LAST_FRAME);
		gl_i2c_mode = I2C_MODE_WAIT;
	}
	else if(gl_i2c_mode == I2C_MODE_RECIEVE)
	{
		HAL_I2C_Slave_Sequential_Receive_IT(&hi2c1, i2c_rx_data.d, RX_SIZE, I2C_FIRST_FRAME);
		gl_i2c_mode = I2C_MODE_WAIT;
		recieved_data_ts = HAL_GetTick();
	}
	if(listen_lock_ts > 300)
		return 1;
	return 0;
}

void I2C1_Reset()
{
	hi2c1.Instance->CR1 &= ~(0x1);
	while(hi2c1.Instance->CR1 & 0x1 != 0);
	hi2c1.Instance->CR1 |= (0x1);
	while(hi2c1.Instance->CR1 & 0x1 == 0);

	hi2c1.State = HAL_I2C_STATE_RESET;
	MX_I2C1_Init();
}
