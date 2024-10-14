/*
 * settings.h
 *
 *  Created on: Aug 13, 2018
 *      Author: Ocanath
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_
#include "stm32g4xx_hal.h"
#include "profiles.h"
/***********************Begin define hardware precomp settings*********************/

/*Motor serial number, for the #define lookup list in settings.c . Also the CAN Node ID*/
//#define MODULE_NUMBER 38


//#define FLIP_TAU

//#define GET_INIT_OFFSET
//#define TEST_INVERTER
//#define TEST_FOC

//#define GIMBAL_MOTOR


extern int32_t gl_cos_min;
extern int32_t gl_cos_max;
extern int32_t gl_sin_min;
extern int32_t gl_sin_max;
extern int32_t gl_oerr_x;

extern float enc_mid_voltage_amp;
//uint32_t i2c_address;
extern uint32_t ser_num;

void load_flash_params(void);


#endif /* SETTINGS_H_ */
