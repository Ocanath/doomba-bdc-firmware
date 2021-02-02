/*
 * settings.c
 *
 *  Created on: Sep 9, 2018
 *      Author: Ocanath
 */
#include "settings.h"

/*
 * NOTE:
 *There are some dead serial numbers here. Calibration is so quick... try and be descriptive in your annotations,
 *and delete serial nums you know are dead, but if you get a bad serial number just do a new one. Generally higher numbers are
 *more likely to be ok.
 */


#ifdef GIMBAL_MOTOR
float enc_mid_voltage_amp = .6;
#else
float enc_mid_voltage_amp = .09;
#endif


#if MODULE_NUMBER == 0							//mC4225 module
float align_offset = -0.232598543;	//0.273627996, p-0.00627872162, n-0.553534687
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+1;
const float elec_conv_ratio = 3.5f;	//MC4225 has 14 poles, 7 pole pairs, and our encoder returns 2*theta.
int16_t cos_mid = 2078;
int16_t sin_mid = 2006;
#endif
#if MODULE_NUMBER == 1							//ms4822 v4 module, Gimbal Car
float align_offset = -0.0450;	//0.273627996, p-0.00627872162, n-0.553534687
const float iq_gain = 0.01f;
const float id_gain = 0.01f;
uint32_t i2c_address = I2C_BASE+4;
const float elec_conv_ratio = 5.5f;	//MC4225 has 22 poles, 11 pole pairs, and our encoder returns 2*theta.
int16_t cos_mid = 2041;
int16_t sin_mid = 2006;
#endif
#if MODULE_NUMBER == 2							//mC4225 module
float align_offset = -0.777598023;	//0.273627996, p-0.00627872162, n-0.553534687
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+2;
const float elec_conv_ratio = 3.5f;	//MC4225 has 14 poles, 7 pole pairs, and our encoder returns 2*theta.
int16_t cos_mid = 2078;
int16_t sin_mid = 1968;
#endif
#if MODULE_NUMBER == 3							//ms4822 v4 module, CYCLETESTER
float align_offset = -0.1265f;	//0.273627996, p-0.00627872162, n-0.553534687
const float iq_gain = 0.01f;
const float id_gain = 0.01f;
uint32_t i2c_address = I2C_BASE+0;
const float elec_conv_ratio = 5.5f;	//MC4225 has 22 poles, 11 pole pairs, and our encoder returns 2*theta.
int16_t cos_mid = 2090;
int16_t sin_mid = 1971;
#endif
#if MODULE_NUMBER == 4							//ms4822 v4 module, for foc car
float align_offset = -0.226198962f;	//-0.236198962f, p-0.00627872162, n-0.553534687
const float iq_gain = 0.01f;
const float id_gain = 0.01f;
uint32_t i2c_address = I2C_BASE+3;
const float elec_conv_ratio = 5.5f;	//MC4225 has 22 poles, 11 pole pairs, and our encoder returns 2*theta.
int16_t cos_mid = 2054;
int16_t sin_mid = 1988;
#endif
#if MODULE_NUMBER == 5							//ms9235
float align_offset = -0.119159445;	//-0.236198962f, p-0.00627872162, n-0.553534687
const float iq_gain = 0.01f;
const float id_gain = 0.01f;
uint32_t i2c_address = I2C_BASE+0;
const float elec_conv_ratio = 10.0f;	//ms9235 has 40 poles, 20 pole pairs, and our encoder returns 2*theta.
int16_t cos_mid = 2008;
int16_t sin_mid = 2028;
#endif
#if MODULE_NUMBER == 6							//MC4114 v4 module
float align_offset = -0.101199064f;
const float iq_gain = 0.01f;
const float id_gain = 0.01f;
uint32_t i2c_address = I2C_BASE+0;
const float elec_conv_ratio = 5.5f;	//MC4114 has 22 poles, 11 pole pairs, and our encoder returns 2*theta.
int16_t cos_mid = 2053;
int16_t sin_mid = 1951;
#endif
#if MODULE_NUMBER == 7		//motor used in gimbal car after the disasterous i2c explosion of 2019
float align_offset = -0.436198771f;
const float iq_gain = 0.01f;
const float id_gain = 0.01f;
uint32_t i2c_address = I2C_BASE+0;
const float elec_conv_ratio = 5.5f;	//MC4114 has 22 poles, 11 pole pairs, and our encoder returns 2*theta.
int16_t cos_mid = 2028;
int16_t sin_mid = 1993;
#endif
#if MODULE_NUMBER == 8		//motor 2 used in gimbal car after the disasterous i2c explosion of 2019
float align_offset = -0.271198928f;
const float iq_gain = 0.01f;
const float id_gain = 0.01f;
uint32_t i2c_address = I2C_BASE+1;
const float elec_conv_ratio = 5.5f;	//MC4114 has 22 poles, 11 pole pairs, and our encoder returns 2*theta.
int16_t cos_mid = 2052;
int16_t sin_mid = 1982;
#endif
#if MODULE_NUMBER == 9						//mC4225 Gearbox module 4
float align_offset = -0.635597825f;	//0.273627996, p-0.00627872162, n-0.553534687
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+4;
const float elec_conv_ratio = 3.5f;	//MC4225 has 14 poles, 7 pole pairs, and our encoder returns 2*theta.
int16_t cos_mid = 2071;
int16_t sin_mid = 1991;
#endif
#if MODULE_NUMBER == 10						//mC4225 Gearbox module 5
float align_offset = -0.05f;	//0.273627996, p-0.00627872162, n-0.553534687
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+5;
const float elec_conv_ratio = 3.5f;	//MC4225 has 14 poles, 7 pole pairs, and our encoder returns 2*theta.
int16_t cos_mid = 2064;
int16_t sin_mid = 1985;
#endif
#if MODULE_NUMBER == 11						//mC4225 Gearbox module 5
float align_offset = -0.412598372f;	//0.273627996, p-0.00627872162, n-0.553534687
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+7;
const float elec_conv_ratio = 3.5f;	//MC4225 has 14 poles, 7 pole pairs, and our encoder returns 2*theta.
int16_t cos_mid = 2079;
int16_t sin_mid = 1988;
#endif
#if MODULE_NUMBER == 12						//mC4225 Gearbox module 5
float align_offset = -0.578984737;	//-0.578984737
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+8;
const float elec_conv_ratio = 4.0f;	//MC4225 has 14 poles, 7 pole pairs, and our encoder returns 2*theta.
int16_t cos_mid = 2072;
int16_t sin_mid = 1948;
#endif
#if MODULE_NUMBER == 14						//mC4225 Gearbox module 5. HAS EXTERNAL OSCILLATOR
float align_offset = -0.578984737;
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+9;
const float elec_conv_ratio = 4.0f;
int16_t cos_mid = 2072;
int16_t sin_mid = 1948;
//has external osc
#endif
#if MODULE_NUMBER == 15						//mC4225 Gearbox module 6
float align_offset = -0.390f;	//-0.578984737
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+10;
const float elec_conv_ratio = 4.0f;	//MC4225 has 14 poles, 7 pole pairs, and our encoder returns 2*theta.
int16_t cos_mid = 2027;
int16_t sin_mid = 1971;
#endif
#if MODULE_NUMBER == 16						//mC4225 Gearbox module 6
float align_offset = -0.671f;	//-0.578984737
const float iq_gain = 0.01;
const float id_gain = 0.01;
const float gl_prop_delay_const = 475;
uint32_t i2c_address = I2C_BASE+11;
const float elec_conv_ratio = 4.0f;	//MC4225 has 14 poles, 7 pole pairs, and our encoder returns 2*theta.
int16_t cos_mid = 2030;
int16_t sin_mid = 1988;
#endif
#if MODULE_NUMBER == 17						//gimbal, test module (severely used motor)
float align_offset = -0.470f;	//-0.578984737
const float iq_gain = 0.01;
const float id_gain = 0.01;
const float gl_prop_delay_const = 475;
uint32_t i2c_address = I2C_BASE+1;
const float elec_conv_ratio = 5.5f;	//MC4225 has 14 poles, 7 pole pairs, and our encoder returns 2*theta.
int16_t cos_mid = 2077;
int16_t sin_mid = 1954;
#endif
#if MODULE_NUMBER == 18						//gimbal, test module (severely used motor)
float align_offset = -0.28f;	//-0.578984737
const float iq_gain = 0.01;
const float id_gain = 0.01;
const float gl_prop_delay_const = 475;
uint32_t i2c_address = I2C_BASE+2;
const float elec_conv_ratio = 5.5f;	//MC4225 has 14 poles, 7 pole pairs, and our encoder returns 2*theta.
int16_t cos_mid = 2064;
int16_t sin_mid = 1975;
#endif
#if MODULE_NUMBER == 19						//rando.
float align_offset = -0.306198895f;	//-0.578984737
const float iq_gain = 0.01;
const float id_gain = 0.01;
const float gl_prop_delay_const = 475;
uint32_t i2c_address = I2C_BASE+2;
const float elec_conv_ratio = 5.5f;	//MC4225 has 14 poles, 7 pole pairs, and our encoder returns 2*theta.
int16_t cos_mid = 2056;
int16_t sin_mid = 1983;
#endif
#if MODULE_NUMBER == 20						//rando.
float align_offset = -0.0561990365f;	//-0.578984737
const float iq_gain = 0.01;
const float id_gain = 0.01;
const float gl_prop_delay_const = 475;
uint32_t i2c_address = I2C_BASE+2;
const float elec_conv_ratio = 5.5f;	//MC4225 has 14 poles, 7 pole pairs, and our encoder returns 2*theta.
int16_t cos_mid = 2068;
int16_t sin_mid = 1941;
#endif
#if MODULE_NUMBER == 21						//rando.
float align_offset = -0.297f;
const float iq_gain = 0.01;
const float id_gain = 0.01;
const float gl_prop_delay_const = 475;
uint32_t i2c_address = I2C_BASE+2;
const float elec_conv_ratio = 5.5f;	//MC4225 has 14 poles, 7 pole pairs, and our encoder returns 2*theta.
int16_t cos_mid = 2050;
int16_t sin_mid = 1987;
#endif
#if MODULE_NUMBER == 22						//rando.
float align_offset = -0.261833549f;
const float iq_gain = 0.01;
const float id_gain = 0.01;
const float gl_prop_delay_const = 475;
uint32_t i2c_address = I2C_BASE+0;
const float elec_conv_ratio = 5.5f;	//MC4225 has 14 poles, 7 pole pairs, and our encoder returns 2*theta.
int16_t cos_mid = 2062;
int16_t sin_mid = 1974;
#endif
#if MODULE_NUMBER == 23						//rando. TLE encoder/v4-r5
float align_offset = 0.0945335031f;
const float iq_gain = 0.01;
const float id_gain = 0.01;
const float gl_prop_delay_const = 476;
const float elec_conv_ratio = 11.f;	//MC4225 has 14 poles, 7 pole pairs, and our encoder returns 2*theta.
int16_t cos_mid = 1996;
int16_t sin_mid = 1975;
#endif
#if MODULE_NUMBER == 24						//rando. TLE encoder/v4-r5
float align_offset = -0.170561865f;
const float iq_gain = 0.01;
const float id_gain = 0.01;
const float gl_prop_delay_const = 476;
const float elec_conv_ratio = 11.f;	//MC4225 has 14 poles, 7 pole pairs, and our encoder returns 2*theta.
int16_t cos_mid = 1976;
int16_t sin_mid = 1976;
#endif
#if MODULE_NUMBER == 25						//rando. TLE encoder/v4-r5
float align_offset = -0.0f;	//needs algining
const float iq_gain = 0.01;
const float id_gain = 0.01;
const float gl_prop_delay_const = 476;	//needs tuning
const float elec_conv_ratio = 8.f;	//MS motor with 16 poles, with TLE makes 8 pole pairs
int16_t cos_mid = 1976;
int16_t sin_mid = 1976;
#endif
#if MODULE_NUMBER == 26						//rando. TLE encoder/v4-r5
float align_offset = -0.0818852335f;
const float iq_gain = 0.01;
const float id_gain = 0.01;
const float gl_prop_delay_const = 476;
const float elec_conv_ratio = 11.f;	//22 poles + TLE encoder, so 11 pole pairs
int16_t cos_mid = 1986;
int16_t sin_mid = 1984;
#endif
#if MODULE_NUMBER == 27						//rando. TLE encoder/v4-r5
float align_offset = -0.170397103f;
const float iq_gain = 0.01;
const float id_gain = 0.01;
const float gl_prop_delay_const = 476;
const float elec_conv_ratio = 11.f;	//22 poles + TLE encoder, so 11 pole pairs
int16_t cos_mid = 1977;
int16_t sin_mid = 1990;
#endif
#if MODULE_NUMBER == 28						//rando. TLE encoder/v4-r5
float align_offset = -0.0786211267f;
const float iq_gain = 0.01;
const float id_gain = 0.01;
const float gl_prop_delay_const = 476;
const float elec_conv_ratio = 11.f;	//22 poles + TLE encoder, so 11 pole pairs
int16_t cos_mid = 1993;
int16_t sin_mid = 1984;
#endif
