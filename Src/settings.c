/*
 * settings.c
 *
 *  Created on: Sep 9, 2018
 *      Author: Ocanath
 */
#include "settings.h"

#define I2C_BASE 0x20
#define MODULE_NUMBER 8	//module num for preset encoder offset


#ifdef GIMBAL_MOTOR
float enc_mid_voltage_amp = .6;
#else
float enc_mid_voltage_amp = .09;
#endif


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
float align_offset = -0.0661990345;
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+2;
const float elec_conv_ratio = 5.5;	//MS4822 has 11 pole pairs. this number is (pole pairs)/2 for the KMZ60 encoder to absorb the extra multiply
int16_t cos_mid = 2100;
int16_t sin_mid = 1977;
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
float align_offset = -.42;
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+0;
const float elec_conv_ratio = 5.5;	//MS4822 has 11 pole pairs. this number is (pole pairs)/2 for the KMZ60 encoder to absorb the extra multiply
int16_t cos_mid = 2133;
int16_t sin_mid = 2015;
#endif
#if MODULE_NUMBER == 11							//gibbon gearbox 1 UNCALIBRATED
float align_offset = 0.345;
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+0;
const float elec_conv_ratio = 5.5;	//MS4822 has 11 pole pairs. this number is (pole pairs)/2 for the KMZ60 encoder to absorb the extra multiply
int16_t cos_mid = 1964;		//NOTE that this is different, and that this varies probably with each unique device on which it is placed. try it on vishan?
int16_t sin_mid = 1988;
#endif
#if MODULE_NUMBER == 12							//gibbon gearbox 2 UNCALIBRATED
float align_offset = 0.23;	//0.273627996, p-0.00627872162, n-0.553534687
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+0;
const float elec_conv_ratio = 5.5;	//MS4822 has 11 pole pairs. this number is (pole pairs)/2 for the KMZ60 encoder to absorb the extra multiply
int16_t cos_mid = 1956;
int16_t sin_mid = 1980;
#endif
#if MODULE_NUMBER == 14							//gimbal large
float align_offset = 0.115;	//0.273627996, p-0.00627872162, n-0.553534687
const float iq_gain = 0.1;	//need higher gain for higher resistance motor?
const float id_gain = 0.05;
uint32_t i2c_address = I2C_BASE+0;
const float elec_conv_ratio = 10.5;	//MS4822 has 11 pole pairs. this number is (pole pairs)/2 for the KMZ60 encoder to absorb the extra multiply
int16_t cos_mid = 2020;
int16_t sin_mid = 1951;
#endif
#if MODULE_NUMBER == 15			//vishan
float align_offset = -1.3;
const float iq_gain = 0.01;
const float id_gain = 0.003;
uint32_t i2c_address = I2C_BASE+0;
const float elec_conv_ratio = 0.5;
int16_t cos_mid = 2043;
int16_t sin_mid = 1941;
#endif
#if MODULE_NUMBER == 16		//AX2804 motor 1
//float align_offset = -0.653;		//good offset
float align_offset = .2452;

const float iq_gain = 0.15;
const float id_gain = 0.003;
uint32_t i2c_address = I2C_BASE+0;
const float elec_conv_ratio = 3.5;
int16_t cos_mid = 2026;
int16_t sin_mid = 1950;
#endif
#if MODULE_NUMBER == 17
float align_offset = -0.106199056;
const float iq_gain = 0.01;
const float id_gain = 0.001;
uint32_t i2c_address = I2C_BASE+0;
const float elec_conv_ratio = 5.5;	//MS4822 has 11 pole pairs. this number is (pole pairs)/2 for the KMZ60 encoder to absorb the extra multiply
int16_t cos_mid = 2027;
int16_t sin_mid = 1933;
#endif


