/*
 * mag-encoder.c
 *
 *  Created on: Jun 20, 2018
 *      Author: Ocanath
 */
#include "mag-encoder.h"
#include "settings.h"
#include "trig_fixed.h"

int32_t sin_mid = 2048;
int32_t cos_mid = 2048;
int32_t mul_cos = 1;
int32_t div_cos = 1;
int32_t mul_sin = 1;
int32_t div_sin = 1;
int32_t oerr_total_nsin = 0;
int32_t oerr_total_ncos = 1;

//#define HIGH_ACCURACY_SIN
float theta_m_prev;

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

/*
 * Fast taylor series based approximation of sin(theta).
 * TODO: implement lookup table implementation
 * NOTE:
 * designed to play with fast atan2, meaning it's only valid for positive quadrants I, II and negative quadrants III, IV.
 */
float sin_fast(float theta)
{
	uint8_t is_neg = 0;
	if(theta > HALF_PI && theta <= PI)	// if positive and in quadrant II, put in quadrant I (same)
	{
		theta = PI - theta;
	}
	else if (theta >= PI && theta < THREE_BY_TWO_PI)  // if positive and in quadrant III (possible for cosine)
	{
		is_neg = 1;
		theta = theta - PI;
	}

	else if (theta > THREE_BY_TWO_PI && theta < TWO_PI)  // if positive and in quadrant IV (edge case of cosine, rare but possible)
	{
		theta = theta - TWO_PI;
	}
	else if (theta < -HALF_PI && theta >= -PI ) // if negative and in quadrant III,
	{
		is_neg = 1;
		theta = PI + theta;
	}

	float theta_2 = theta*theta;
	float theta_3 = theta*theta_2;
	float theta_5 = theta_3*theta_2;
#ifdef HIGH_ACCURACY_SIN
	float theta_7 = theta_5*theta_2;
	float res = theta-theta_3*ONE_BY_THREE_FACTORIAL + theta_5 * ONE_BY_FIVE_FACTORIAL - ONE_BY_SEVEN_FACTORIAL*theta_7;
#else
	float res = theta-theta_3*ONE_BY_THREE_FACTORIAL + theta_5 * ONE_BY_FIVE_FACTORIAL;
#endif

	if(is_neg == 1)
		return -res;
	else
		return res;
}

/*
 * returns cos approx using sin approx
 */
float cos_fast(float theta)
{
	return sin_fast(theta + HALF_PI);
}


float theta_abs_rad()
{
	int16_t sinVal = dma_adc_raw[ADC_SIN_CHAN]-sin_mid;
	int16_t cosVal = dma_adc_raw[ADC_COS_CHAN]-cos_mid;
//	return atan2((float)sinVal,(float)cosVal);
	return atan2_approx((float)sinVal,(float)cosVal);
}

float theta_rel_rad()
{
	int16_t sinVal = dma_adc_raw[ADC_SIN_CHAN]-sin_mid;
	int16_t cosVal = dma_adc_raw[ADC_COS_CHAN]-cos_mid;
//	return atan2((float)sinVal,(float)cosVal) - align_offset;
	return atan2_approx((float)sinVal,(float)cosVal) - align_offset;
}

/*use the global encoder max and min params to claculate gains
 * and midpoints for commutation encoder.*/
void eval_encoder_params(void)
{
	sin_mid = (fds_mp.gl_sin_max + fds_mp.gl_sin_min)/2;
	cos_mid = (fds_mp.gl_cos_max + fds_mp.gl_cos_min)/2;
	mul_cos = (1<<12);
	div_cos = (fds_mp.gl_cos_max - fds_mp.gl_cos_min)/2;
	mul_sin = (1<<12);
	div_sin = (fds_mp.gl_sin_max - fds_mp.gl_sin_min)/2;
	oerr_total_nsin = sin_12b(-fds_mp.gl_oerr_total);
	oerr_total_ncos = cos_12b(-fds_mp.gl_oerr_total);
}

int32_t theta_rel_fixed(void)
{
	int32_t sinVal = ((int32_t)(dma_adc_raw[ADC_SIN_CHAN]))-sin_mid;
	int32_t cosVal = ((int32_t)(dma_adc_raw[ADC_COS_CHAN]))-cos_mid;
	sinVal = (sinVal*mul_sin)/div_sin;
	cosVal = (cosVal*mul_cos)/div_cos;
	/*
	 * sin phase compensator term.
	 */
	sinVal = (sinVal*4096 - cosVal*oerr_total_nsin)/oerr_total_ncos;	//this is dangerous iff gl_oerr_total == PI_12B/2 (divide by 0 error). however this should be well outside the actual range of achievable values for this number
	return atan2_fixed(sinVal,cosVal) - fds_mp.align_offset_fixed;
}

/*return raw atan2 angle, no commutation offset or orthogonality/phase error comp*/
int32_t theta_abs_fixed(void)
{
	int32_t sinVal = ((int32_t)(dma_adc_raw[ADC_SIN_CHAN]))-sin_mid;
	int32_t cosVal = ((int32_t)(dma_adc_raw[ADC_COS_CHAN]))-cos_mid;
	sinVal = (sinVal*mul_sin)/div_sin;
	cosVal = (cosVal*mul_cos)/div_cos;
	return atan2_fixed(sinVal,cosVal);
}

/*
 * Expose sin and cos min-maxed-offsetted to rest of the sw, WITHOUT
 * ORTHO COMP! (important for *calculating* the ortho comp)
 * */
void get_sc_no_ortho_comp(int32_t * c, int32_t * s)
{
	int32_t sinVal = ((int32_t)(dma_adc_raw[ADC_SIN_CHAN]))-sin_mid;
	int32_t cosVal = ((int32_t)(dma_adc_raw[ADC_COS_CHAN]))-cos_mid;
	sinVal = (sinVal*mul_sin)/div_sin;
	cosVal = (cosVal*mul_cos)/div_cos;
	*c = cosVal;
	*s = sinVal;
}

const float rad_to_deg = 180.0/PI;
float theta_abs_deg()
{
	return theta_abs_rad()*rad_to_deg;
}

float theta_rel_deg()
{
	return theta_rel_rad()*rad_to_deg;
}

/*
 * Helper Function to quickly perform floating point mod of two pi
 */
float fmod_2pi(float in)
{
	uint8_t aneg = 0;
	float in_eval = in;
	if(in < 0)
	{
		aneg = 1;
		in_eval = -in;
	}
	float fv = (float)((int)(in_eval*ONE_BY_TWO_PI));
	if(aneg == 1)
		fv = (-fv)-1;
	return in-TWO_PI*fv;
}

float unwrap(float theta, float * prev_theta)
{
	float dif = fmod_2pi(fmod_2pi(*prev_theta) - theta + PI);
	if(dif < 0)
		dif = dif + TWO_PI;
	dif = dif - PI;
	float theta_ret = *prev_theta - dif;
	*prev_theta = theta_ret;
	return theta_ret;
}

