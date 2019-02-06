/*
 * foc_commutation.h
 *
 *  Created on: Mar 21, 2018
 *      Author: Ocanath
 */

#ifndef FOC_COMMUTATION_H_
#define FOC_COMMUTATION_H_
#include "init.h"
#include "mag-encoder.h"
#include "settings.h"


#define CONST_E 0.367879441171442321595523770161460867445811131031767834507
float gl_I;
float gl_T;
float gl_V;


int32_t gl_current_input_offset_A;
int32_t gl_current_input_offset_B;
int32_t gl_current_input_offset_C;
float L;
float R;

typedef struct vect2
{
	float v1;
	float v2;
}vect2;


#define TIMER_UPDATE_DUTY(duty1, duty2, duty3) \
		TIM1->CR1 |= TIM_CR1_UDIS; \
		TIM1->CCR1 = duty1; \
		TIM1->CCR2 = duty2; \
		TIM1->CCR3 = duty3; \
		TIM1->CR1 &= ~TIM_CR1_UDIS;


void get_current_cal_offsets();
void conv_raw_current(float * i_a, float * i_b, float * i_c);
void convert_phase_voltage(float * va, float * vb, float * vc);

void clarke_transform(float i_a, float i_b, float i_c, float * i_alpha, float * i_beta);
void park_transform(float i_alpha, float i_beta, float sin_theta, float cos_theta, float * i_q, float * i_d);

void controller_PI(float i_q_ref, float i_q, float Kp, float Ki, float * x, float * u);

void inverse_park_transform(float i_q, float i_d, float sin_theta, float cos_theta, float * i_alpha, float * i_beta);
void inverse_clarke_transform(float i_alpha, float i_beta, float * i_a, float * i_b, float * i_c);
int svm(float alpha, float beta, uint32_t pwm_period_cnt, uint32_t * tA, uint32_t * tB, uint32_t * tC);

void init_observer();
float observer_update(float v_a, float v_b, float i_a, float i_b, float * x1, float * x2);

void sector_check(float alpha, float beta, float * theta, uint32_t * sector);
float est_R();
void obtain_encoder_offset();
void obtain_encoder_midpoints();




extern float theta_enc_prev;
extern float x_iq_PI;
extern float x_id_PI;
/*
 *	FOC loop for readability/cleanliness
 */
extern float i_q_meas;
extern float i_d_meas;
inline void foc(float iqref, float idref)
{
	float i_a,i_b,i_c;
	conv_raw_current(&i_a,&i_b, &i_c);

	float i_alpha,i_beta;
	clarke_transform(i_a,i_b,i_c,&i_alpha, &i_beta);

	float theta_enc = unwrap( theta_rel_rad(), &theta_enc_prev);
	float theta_elec = theta_enc*elec_conv_ratio;		//test this! MS motor has 22 pole pairs, which means multiply by 11. ENCODER RETURNS 2*theta so we absorb .5 into this term
	theta_elec = fmod_2pi(theta_elec + PI) - PI;		//re-modulate theta_m. ensure that the angle is constrained from -pi to pi!!
	float sin_theta = sin_fast(theta_elec);				//calculate the sin of the electrical (magnetic flux) angle
	float cos_theta = cos_fast(theta_elec);				//and the cosine for park and inverse park domains

	park_transform(i_alpha, i_beta, sin_theta,cos_theta, &i_q_meas, &i_d_meas);

	float uq,ud;
	controller_PI(iqref, i_q_meas, iq_gain, 0.0000000001, &x_iq_PI, &uq);		//this sort of works
	controller_PI(idref, i_d_meas, id_gain, 0.0000000001, &x_id_PI, &ud);		//high current

	uint32_t tA,tB,tC;
	inverse_park_transform(uq, ud, sin_theta,cos_theta, &i_alpha, &i_beta);	//maybe call theta rel again?sin_theta
	svm(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);
	TIMER_UPDATE_DUTY(tA,tB,tC);
}

#endif /* FOC_COMMUTATION_H_ */
