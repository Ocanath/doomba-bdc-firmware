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
#if MODULE_NUMBER == 20				//currently in gearbox
	#define ALIGN_OFFSET	0.0229281429f
int16_t cos_mid = 1973;
int16_t sin_mid = 1973;
#define PROFILE MULTISTAR_MS4225
#endif
#if MODULE_NUMBER == 21					//currently in spinner launcher
	#define ALIGN_OFFSET	0.0899067894f
	int16_t cos_mid = 1997;
	int16_t sin_mid = 1985;
	#define PROFILE MULTISTAR_MS4225
#endif
#if MODULE_NUMBER == 22						//currently in spinner launcher
	#define ALIGN_OFFSET	-0.877466202f
	int16_t cos_mid = 1996;
	int16_t sin_mid = 1974;
	#define PROFILE MULTISTAR_MS4822
#endif
#if MODULE_NUMBER == 23						//rando. TLE encoder/v4-r5
	#define ALIGN_OFFSET	0.0346194729f
	int16_t cos_mid = 1985;
	int16_t sin_mid = 1986;
	#define PROFILE MULTISTAR_MS4822
#endif
#if MODULE_NUMBER == 24						//rando. TLE encoder/v4-r5
	#define ALIGN_OFFSET	0.0907510966f
	int16_t cos_mid = 1986;
	int16_t sin_mid = 1984;
	#define PROFILE MULTISTAR_MS4822
#endif
#if MODULE_NUMBER == 25						//rando. TLE encoder/v4-r5
	#define ALIGN_OFFSET	-0.0f	//needs algining
	int16_t cos_mid = 1976;
	int16_t sin_mid = 1976;
	#define PROFILE MULTISTAR_MS4225
#endif
#if MODULE_NUMBER == 26						//rando. TLE encoder/v4-r5
	#define ALIGN_OFFSET	-0.0818852335f
	int16_t cos_mid = 1988;
	int16_t sin_mid = 1962;
	#define PROFILE MULTISTAR_MS4822
#endif
#if MODULE_NUMBER == 27						//rando. TLE encoder/v4-r5
	#define ALIGN_OFFSET	-0.170397103f
	int16_t cos_mid = 1977;
	int16_t sin_mid = 1990;
	#define PROFILE MULTISTAR_MS4822
#endif
#if MODULE_NUMBER == 28						//rando. TLE encoder/v4-r5
	#define ALIGN_OFFSET	-0.0970252305f
	int16_t cos_mid = 1993;
	int16_t sin_mid = 1984;
	#define PROFILE MULTISTAR_MS4822
#endif
#if MODULE_NUMBER == 29						//TLE Aerosky
	#define ALIGN_OFFSET	2.01f
	int16_t cos_mid = 1985;
	int16_t sin_mid = 1985;
	#define PROFILE AEROSKY_MC4225
#endif
#if MODULE_NUMBER == 30						//rando. TLE encoder/v4-r5
	#define ALIGN_OFFSET 	0.216673866f
	int16_t cos_mid = 2000;
	int16_t sin_mid = 1987;
	#define PROFILE AEROSKY_MC4225
#endif
#if MODULE_NUMBER == 31						//rando. TLE encoder/v4-r5
	#define ALIGN_OFFSET	0.393527597f
	int16_t cos_mid = 1973;
	int16_t sin_mid = 1974;
	#define PROFILE AEROSKY_MC4225
#endif
#if MODULE_NUMBER == 32						//rando. TLE encoder/v4-r5
	#define ALIGN_OFFSET	-0.256763965f
	int16_t cos_mid = 1971;
	int16_t sin_mid = 1978;
	#define PROFILE AEROSKY_MC4225
#endif
#if MODULE_NUMBER == 33						//rando. TLE encoder/v4-r5
	#define ALIGN_OFFSET	-0.115011632f
	int16_t cos_mid = 1991;
	int16_t sin_mid = 1970;
	#define PROFILE AEROSKY_MC4225
#endif
#if MODULE_NUMBER == 34						//rando. TLE encoder/v4-r5
	#define ALIGN_OFFSET	0.42705825f
	int16_t cos_mid = 1976;
	int16_t sin_mid = 1974;
	#define PROFILE AEROSKY_MC4225
#endif
#if MODULE_NUMBER == 35						//hex motor (nw needs fixing, not yet calibrated). TLE encoder/v4-r5
	#define ALIGN_OFFSET -0.108883113f
	int16_t cos_mid = 1972;
	int16_t sin_mid = 1981;
	#define PROFILE MULTISTAR_MS4225
#endif
#if MODULE_NUMBER == 36						//hex motor. TLE encoder/v4-r5
	#define ALIGN_OFFSET -0.0932625011f
	int16_t cos_mid = 1982;
	int16_t sin_mid = 1963;
	#define PROFILE MULTISTAR_MS4822
#endif
#if MODULE_NUMBER == 37						//hex motor. TLE encoder/v4-r5
	#define ALIGN_OFFSET 0.206599116f
	int16_t cos_mid = 1984;
	int16_t sin_mid = 1977;
	#define PROFILE MULTISTAR_MS4225
#endif
#if MODULE_NUMBER == 38
	#define ALIGN_OFFSET 3230
	int32_t gl_cos_min = 594;
	int32_t gl_cos_max = 3408;
	int32_t gl_sin_min = 583;
	int32_t gl_sin_max = 3395;
	int32_t gl_oerr_total = 154;
	#define PROFILE AEROSKY_MC4114
#endif
#if MODULE_NUMBER == 39
	#define ALIGN_OFFSET	1.12354243f
	int32_t gl_cos_min = 605;
	int32_t gl_cos_max = 3371;
	int32_t gl_sin_min = 626;
	int32_t gl_sin_max = 3374;
	int32_t gl_oerr_total = 0;
	int32_t gl_oerr_x = 0;
	#define PROFILE AEROSKY_MC4114
#endif

/*Load align offset globals from the define*/
float align_offset = (float)(ALIGN_OFFSET)/4096.f;
int32_t align_offset_fixed = ALIGN_OFFSET;


#if PROFILE == AEROSKY_MC4114
	/*Fixed crap that makes the thing work good*/
	/*number of polepairs.*/
	const int32_t elec_conv_ratio_fixed = 11;
	/*Phase advancement params*/
	int32_t gl_prop_delayloop_interval = 35;
	int32_t gl_prop_delay_const_12b = 1;//(int32_t)(25.f/ gl_prop_delay_const);
	/*PI controllers paramteres*/
	fixed_PI_params_t iq_pi = {
			.kp = {.i32 = 15887, .radix = 5},		//(4 << 12, 33)
			.ki = {.i32 = 11356, .radix = 8},	// 3 << 12, 277
			.x = 0,
			.x_sat = MAX_VQ,
			.out_rshift = 12
	};
	fixed_PI_params_t id_pi = {
			.kp = {.i32 = 9830,  .radix = 3},	//(3 << 12, 10)
			.ki = {.i32 = 4369,	.radix = 5},	//(1 << 12, 30)
			.x = 0,
			.x_sat = MAX_VQ,
			.out_rshift = 12
	};

	/*Floating crap i ought to just get rid of it already cammmaaaaaannnnnn*/
	const float iq_gain = 0.03f;
	const float id_gain = 0.01f;
	const float gl_prop_delay_const = 900.f;
	const float elec_conv_ratio = (float)elec_conv_ratio_fixed;		//7.f;	//22 poles + TLE encoder, so 11 pole pairs
#endif
#if PROFILE == AEROSKY_MC4225
	/*Fixed crap that makes the thing work good*/
	/*number of polepairs.*/
	const int32_t elec_conv_ratio_fixed = 7;
	/*Phase advancement params*/
	int32_t gl_prop_delayloop_interval = 25;
	int32_t gl_prop_delay_const_12b = 1;//(int32_t)(25.f/ gl_prop_delay_const);
	/*PI controllers paramteres*/
	fixed_PI_params_t iq_pi = {
			.kp = {.i32 = 15887, .radix = 5},		//(4 << 12, 33)
			.ki = {.i32 = 11356, .radix = 8},	// 3 << 12, 277
			.x = 0,
			.x_sat = MAX_VQ,
			.out_rshift = 12
	};
	fixed_PI_params_t id_pi = {
			.kp = {.i32 = 9830,  .radix = 3},	//(3 << 12, 10)
			.ki = {.i32 = 4369,	.radix = 5},	//(1 << 12, 30)
			.x = 0,
			.x_sat = MAX_VQ,
			.out_rshift = 12
	};

	/*Floating crap i ought to just get rid of it already cammmaaaaaannnnnn*/
	const float iq_gain = 0.03f;
	const float id_gain = 0.01f;
	const float gl_prop_delay_const = 900.f;
	const float elec_conv_ratio = (float)elec_conv_ratio_fixed;		//7.f;	//22 poles + TLE encoder, so 11 pole pairs
#endif
#if PROFILE == MULTISTAR_MS4822
	/*Fixed crap that makes the thing work good*/
	/*number of polepairs.*/
	const int32_t elec_conv_ratio_fixed = 11;
	/*Phase advancement params*/
	int32_t gl_prop_delayloop_interval = 35;
	int32_t gl_prop_delay_const_12b = 1;//(int32_t)(25.f/ gl_prop_delay_const);
	/*PI controllers paramteres*/
	fixed_PI_params_t iq_pi = {
			.kp = {.i32 = 15887, .radix = 5},		//(4 << 12, 33)
			.ki = {.i32 = 11356, .radix = 8},	// 3 << 12, 277
			.x = 0,
			.x_sat = MAX_VQ,
			.out_rshift = 12
	};
	fixed_PI_params_t id_pi = {
			.kp = {.i32 = 9830,  .radix = 3},	//(3 << 12, 10)
			.ki = {.i32 = 4369,	.radix = 5},	//(1 << 12, 30)
			.x = 0,
			.x_sat = MAX_VQ,
			.out_rshift = 12
	};
	/*Floating crap i ought to just get rid of it already cammmaaaaaannnnnn*/
	const float iq_gain = 0.03f;
	const float id_gain = 0.01f;
	const float elec_conv_ratio = (float)elec_conv_ratio_fixed;		//7.f;	//22 poles + TLE encoder, so 11 pole pairs
	const float gl_prop_delay_const = 476.f;
#endif
#if PROFILE == MULTISTAR_MS4225
	const int32_t elec_conv_ratio_fixed = 8;
	/*Phase advancement params*/
	int32_t gl_prop_delayloop_interval = 26;
	int32_t gl_prop_delay_const_12b = 1;//(int32_t)(25.f/ gl_prop_delay_const);
	/*PI controllers paramteres*/
	fixed_PI_params_t iq_pi = {
			.kp = {.i32 = 15887, .radix = 5},		//(4 << 12, 33)
			.ki = {.i32 = 11356, .radix = 8},	// 3 << 12, 277
			.x = 0,
			.x_sat = MAX_VQ,		//value << 12, where value is the vq value your power delivery system/motor can handle indefinitely at stall
			.out_rshift = 12
	};
	fixed_PI_params_t id_pi = {
			.kp = {.i32 = 9830,  .radix = 3},	//(3 << 12, 10)
			.ki = {.i32 = 4369,	.radix = 5},	//(1 << 12, 30)
			.x = 0,
			.x_sat = MAX_VQ,
			.out_rshift = 12
	};
	const float iq_gain = 0.01;
	const float id_gain = 0.01;
	const float elec_conv_ratio = (float)elec_conv_ratio_fixed;		//7.f;	//22 poles + TLE encoder, so 11 pole pairs
	const float gl_prop_delay_const = 800.f;
#endif
