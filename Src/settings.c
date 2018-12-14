/*
 * settings.c
 *
 *  Created on: Sep 9, 2018
 *      Author: Ocanath
 */
#include "settings.h"

#define I2C_BASE 0x20
#define MODULE_NUMBER 11	//module num for preset encoder offset


#if MODULE_NUMBER==0
float align_offset = -2.69547725;				//original leg hip motor (dyna leg module 0)
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+0;
//const float num_pole_pairs = 11.0;	//MS4822
const float elec_conv_ratio = 11.0;	//MS4822 has 11 pole pairs.
int16_t cos_mid = 1986;
int16_t sin_mid = 2006;
#endif
#if MODULE_NUMBER==1
float align_offset = -0.296428591;				//original leg knee motor (dyna leg module 1)
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+1;
//const float num_pole_pairs = 11.0;	//MS4822
const float elec_conv_ratio = 11.0;	//MS4822 has 11 pole pairs
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
#if MODULE_NUMBER == 3							//dynacat module number 3
float align_offset = -0.165077627;
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+2;
const float elec_conv_ratio = 5.5;	//MS4822 has 11 pole pairs. this number is (pole pairs)/2 for the KMZ60 encoder to absorb the extra multiply
int16_t cos_mid = 1912;
int16_t sin_mid = 2054;
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
#if MODULE_NUMBER == 5							//dynacat module number 5
float align_offset = -0.504629734;
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+3;
const float elec_conv_ratio = 5.5;	//MS4822 has 11 pole pairs. this number is (pole pairs)/2 for the KMZ60 encoder to absorb the extra multiply
int16_t cos_mid = 1912;
int16_t sin_mid = 2054;
#endif
#if MODULE_NUMBER == 6							//ms4822 3mm magnet (dynacat module number 6 i hope)
//float align_offset = -0.264224828;				//
float align_offset = -0.273223341;
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+5;
const float elec_conv_ratio = 5.5;	//MS4822 has 11 pole pairs. this number is (pole pairs)/2 for the KMZ60 encoder to absorb the extra multiply
int16_t cos_mid = 1912;
int16_t sin_mid = 2054;
#endif
#if MODULE_NUMBER == 7							//dyna leg modle 7
//float align_offset = -0.264224828;				//
float align_offset = 0.271173596;
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+4;
const float elec_conv_ratio = 5.5;	//MS4822 has 11 pole pairs. this number is (pole pairs)/2 for the KMZ60 encoder to absorb the extra multiply
int16_t cos_mid = 1912;
int16_t sin_mid = 2054;
#endif
#if MODULE_NUMBER == 8							//dyna leg module 8
float align_offset = 0.190164273;
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+6;
const float elec_conv_ratio = 5.5;	//MS4822 has 11 pole pairs. this number is (pole pairs)/2 for the KMZ60 encoder to absorb the extra multiply
int16_t cos_mid = 1912;
int16_t sin_mid = 2054;
#endif
#if MODULE_NUMBER == 9							//dyna leg module 9
float align_offset = -0.419265962;
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+7;
const float elec_conv_ratio = 5.5;	//MS4822 has 11 pole pairs. this number is (pole pairs)/2 for the KMZ60 encoder to absorb the extra multiply
int16_t cos_mid = 1912;
int16_t sin_mid = 2054;
#endif
#if MODULE_NUMBER == 10							//grapple spooler, now cycle tester
float align_offset = -0.155;
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+0;
const float elec_conv_ratio = 5.5;	//MS4822 has 11 pole pairs. this number is (pole pairs)/2 for the KMZ60 encoder to absorb the extra multiply
int16_t cos_mid = 1912;
int16_t sin_mid = 2054;
#endif
#if MODULE_NUMBER == 11							//gibbon gearbox 1
float align_offset = -0.17531758;
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+0;
const float elec_conv_ratio = 5.5;	//MS4822 has 11 pole pairs. this number is (pole pairs)/2 for the KMZ60 encoder to absorb the extra multiply
int16_t cos_mid = 1912;
int16_t sin_mid = 2054;
#endif
