/*
 * mag-encoder.h
 *
 *  Created on: Jun 20, 2018
 *      Author: Ocanath
 */

#ifndef MAG_ENCODER_H_
#define MAG_ENCODER_H_
#include "adc.h"

#define READ_ANGLEUNC	0x7FFE
#define READ_ANGLECOM	0xFFFF

#define ADC_SIN_CHAN 	1
#define ADC_COS_CHAN 	0

#define ONE_BY_THREE_FACTORIAL 	0.16666666666
#define ONE_BY_FIVE_FACTORIAL 	0.00833333333
#define ONE_BY_SEVEN_FACTORIAL	0.00019841269
#define HALF_PI 				1.57079632679
#define PI						3.14159265359
#define THREE_BY_TWO_PI     	4.71238898038
#define TWO_PI              	6.28318530718
#define ONE_BY_TWO_PI 			0.1591549

const extern float rad_to_deg;

extern float theta_m_prev;

extern int32_t sin_mid;
extern int32_t cos_mid;
extern int32_t mul_cos;
extern int32_t div_cos;
extern int32_t mul_sin;
extern int32_t div_sin;
extern int32_t oerr_total_nsin;
extern int32_t oerr_total_ncos;
extern int32_t gl_spi_angle;
//extern int16_t cos_mid;
//extern int16_t sin_mid;


float theta_abs_rad();
float theta_rel_rad();
float theta_abs_deg();
float theta_rel_deg();
int32_t theta_rel_14b(void);
int32_t theta_rel_12b(void);
int32_t theta_abs_14b(void);
int32_t theta_abs_12b(void);
void get_sc_no_ortho_comp(int32_t * c, int32_t * s);


/*
 * fast trig functions
 */
float atan2_approx(float sinVal, float cosVal);
float sin_fast(float theta);
float cos_fast(float theta);
float fmod_2pi(float in);
float unwrap(float theta, float * prev_theta);
//void eval_encoder_params(void);

#endif /* MAG_ENCODER_H_ */
