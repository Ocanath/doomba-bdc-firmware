/*
 * mag-encoder.c
 *
 *  Created on: Jun 20, 2018
 *      Author: Ocanath
 */
#include "mag-encoder.h"
#include <math.h>

//const int16_t cos_mid = 1992;
//const int16_t sin_mid = 1992;		//my encoder
//
const int16_t cos_mid = 1986;
const int16_t sin_mid = 2006;		//my encoder

//const int16_t sin_mid = 1250;		//steven encoder
//const int16_t cos_mid = 1243.5;

float align_offset = 1.40370858;				//offset angle IN RADIANS


float atan2_approx(float sinVal, float cosVal)
{
	float abs_s = sinVal;
	if(abs_s < 0)
		abs_s = -abs_s;
	float abs_c = cosVal;
	if(abs_c < 0)
		abs_c = -abs_c;
	float min_v = abs_c;
	float max_v = abs_s;
	if(abs_s < abs_c)
	{
		min_v = abs_s;
		max_v = abs_c;
	}
	float a = min_v/max_v;
	float sv = a*a;
	float r = ((-0.0464964749 * sv + 0.15931422)*sv- 0.327622764) * sv * a + a;
	if(abs_s > abs_c)
		r = 1.57079637 -r;
	if(cosVal < 0)
		r = 3.14159274 - r;
	if(sinVal < 0)
		r = -r;
	return r;
}

float theta_abs_rad()
{
	int16_t sinVal = dma_adc_raw[ADC_SIN_CHAN]-sin_mid;
	int16_t cosVal = dma_adc_raw[ADC_COS_CHAN]-cos_mid;
	return atan2((float)sinVal,(float)cosVal);
}

float theta_rel_rad()
{
	int16_t sinVal = dma_adc_raw[ADC_SIN_CHAN]-sin_mid;
	int16_t cosVal = dma_adc_raw[ADC_COS_CHAN]-cos_mid;
	return atan2((float)sinVal,(float)cosVal) - align_offset;
}
const float rad_to_deg = 180.0/M_PI;
float theta_abs_deg()
{
	return theta_abs_rad()*rad_to_deg;
}

float theta_rel_deg()
{
	return theta_rel_rad()*rad_to_deg;
}

