/*
 * settings.h
 *
 *  Created on: Aug 13, 2018
 *      Author: Ocanath
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

#define I2C_BASE 0x20
#define I2C_OFFSET 0
#define I2C_ADDRESS (I2C_BASE+I2C_OFFSET)


#define NUM_POLE_PAIRS 11.0		//MS4822
//#define NUM_POLE_PAIRS 6.0	//v-spec
//#define NUM_POLE_PAIRS 7.0	//A2217

//#define GET_INIT_OFFSET
#define MODULE_NUMBER 3	//module num for preset encoder offset


#endif /* SETTINGS_H_ */
