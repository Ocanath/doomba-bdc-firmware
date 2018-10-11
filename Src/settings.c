/*
 * settings.c
 *
 *  Created on: Sep 9, 2018
 *      Author: Ocanath
 */
#include "settings.h"

#define I2C_BASE 0x20
#define MODULE_NUMBER 6	//module num for preset encoder offset


#if MODULE_NUMBER==0
float align_offset = -2.69547725;				//original leg hip motor
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+0;
const float num_pole_pairs = 11.0;	//MS4822
int16_t cos_mid = 1986;
int16_t sin_mid = 2006;
#endif
#if MODULE_NUMBER==1
float align_offset = -0.296428591;				//original leg knee motor
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+1;
const float num_pole_pairs = 11.0;	//MS4822
int16_t cos_mid = 1986;
int16_t sin_mid = 2006;
#endif
#if MODULE_NUMBER == 2				//A2217 car
float align_offset = 1.29285502;
const float iq_gain = 0.005;
const float id_gain = 0.001;
uint32_t i2c_address = I2C_BASE+0;
const float num_pole_pairs = 7.0;	//A2217
int16_t cos_mid = 1986;
int16_t sin_mid = 2006;
#endif
#if MODULE_NUMBER == 3							//loose ms4822 3mm magnet
float align_offset = -0.881572366;
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+0;
const float num_pole_pairs = 11.0;	//MS4822
int16_t cos_mid = 1986;
int16_t sin_mid = 2006;
#endif
#if MODULE_NUMBER == 4				//A2217 car
float align_offset = 1.19608891;
const float iq_gain = 0.005;
const float id_gain = 0.001;
uint32_t i2c_address = I2C_BASE+1;
const float num_pole_pairs = 7.0;	//A2217
int16_t cos_mid = 1986;
int16_t sin_mid = 2006;
#endif
#if MODULE_NUMBER == 5							//ms4822 3mm magnet
float align_offset = -1.82664216;
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+0;
const float num_pole_pairs = 11.0;	//MS4822
int16_t cos_mid = 1986;
int16_t sin_mid = 2006;
#endif
#if MODULE_NUMBER == 6							//ms4822 3mm magnet
float align_offset = 0.842972875;
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+0;
const float num_pole_pairs = 11.0;	//MS4822
int16_t cos_mid = 1911;
int16_t sin_mid = 1911;
#endif

