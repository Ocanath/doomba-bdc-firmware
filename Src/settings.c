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

#define I2C_BASE 0x20
#define MODULE_NUMBER 0	//module num for preset encoder offset


#ifdef GIMBAL_MOTOR
float enc_mid_voltage_amp = .6;
#else
float enc_mid_voltage_amp = .09;
#endif


#if MODULE_NUMBER == 0							//spooler module 2
float align_offset = -0.0;	//0.273627996, p-0.00627872162, n-0.553534687
const float iq_gain = 0.01;
const float id_gain = 0.01;
uint32_t i2c_address = I2C_BASE+0;
const float elec_conv_ratio = 5.5;	//MS4822 has 11 pole pairs. this number is (pole pairs)/2 for the KMZ60 encoder to absorb the extra multiply
int16_t cos_mid = 2110;
int16_t sin_mid = 1974;
#endif
