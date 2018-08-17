/*
 * i2c.h
 *
 *  Created on: Feb 9, 2018
 *      Author: Ocanath
 */

#ifndef I2C_H_
#define I2C_H_

#include "init.h"

#define I2C_MODE_WAIT 0
#define I2C_MODE_TRANSMIT 1
#define I2C_MODE_RECIEVE 2
#define I2C_MODE_BUSY 3

#define I2C_DIRECTION_MASTER_TRANSMIT 0
#define I2C_DIRECTION_MASTER_RECIEVE 1
uint8_t gl_i2c_mode;

#define TX_SIZE 5
#define RX_SIZE 4
uint8_t i2c_tx_data[TX_SIZE];
uint8_t i2c_rx_data[RX_SIZE];

void handle_i2c();
void get_i2c_rx(uint8_t * rx_data);
void get_i2c_tx(uint8_t * tx_data);

#endif /* I2C_H_ */
