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





extern float theta_enc_prev;
extern float x_iq_PI;
extern float x_id_PI;
/*
 *	FOC loop for readability/cleanliness
 */
extern float i_q_meas;
extern float i_d_meas;
extern uint32_t dq_ts;
extern float prev_theta_m_dq;
extern float phase_advance_angle;

extern float gl_theta_m;
extern float gl_dq;

extern const float adc_conv_A;
extern const float adc_conv_B;
extern const float adc_conv_C;

float abs_f(float in);

/**/
static inline void sinusoidal(float Vq, float Vd)
{
	float i_a,i_b,i_c;
	float i_alpha,i_beta;
	conv_raw_current(&i_a, &i_b, &i_c);
	clarke_transform(i_a,i_b,i_c,&i_alpha, &i_beta);

	float theta_enc = theta_rel_rad();
	float theta_elec = theta_enc*elec_conv_ratio + phase_advance_angle;		//test this! MS motor has 22 pole pairs, which means multiply by 11. ENCODER RETURNS 2*theta so we absorb .5 into this term
	theta_elec = fmod_2pi(theta_elec + PI) - PI;		//re-modulate theta_m. ensure that the angle is constrained from -pi to pi!!
	float sin_theta = sin_fast(theta_elec);				//calculate the sin of the electrical (magnetic flux) angle
	float cos_theta = cos_fast(theta_elec);				//and the cosine for park and inverse park domains

	float iqm,idm;
	park_transform(i_alpha, i_beta, sin_theta,cos_theta, &iqm, &idm);

	if(Vq > .86f)
		Vq = .86f;
	else if(Vq < -.86f)
		Vq = -.86f;

	uint32_t tA,tB,tC;
	inverse_park_transform(Vq, Vd, sin_theta,cos_theta, &i_alpha, &i_beta);	//maybe call theta rel again?sin_theta
	svm(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);
	TIMER_UPDATE_DUTY(tA,tB,tC);

	gl_theta_m = unwrap(theta_enc, &theta_enc_prev);
	if(HAL_GetTick() > dq_ts) //in this implementation, mechanical theta is computed outside the loop and shared with foc.
	{
		gl_dq = (gl_theta_m-prev_theta_m_dq)*25.0f;
		prev_theta_m_dq = gl_theta_m;
		phase_advance_angle = gl_dq/gl_prop_delay_const;//division yields more accurate advance angle for better performance.
		dq_ts = HAL_GetTick()+40;

		/*
		 * TODO: swap the two drivers (my pop and pcbgogo mod). compare against module 019
		 */
	}
}

/**/
static inline void foc(float iqref, float idref)
{
	float theta_elec = theta_rel_rad()*elec_conv_ratio + phase_advance_angle;
	theta_elec = fmod_2pi(theta_elec + PI) - PI;		//re-modulate theta_m. ensure that the angle is constrained from -pi to pi!!
	float sin_theta = sin_fast(theta_elec);				//calculate the sin of the electrical (magnetic flux) angle
	float cos_theta = cos_fast(theta_elec);				//and the cosine for park and inverse park domains

	float i_alpha,i_beta;
	float i_a,i_b,i_c;
	conv_raw_current(&i_a,&i_b, &i_c);
	clarke_transform(i_a,i_b,i_c,&i_alpha, &i_beta);
	park_transform(i_alpha, i_beta, sin_theta,cos_theta, &i_q_meas, &i_d_meas);

	float uq,ud;
	controller_PI(iqref, -i_q_meas, iq_gain, 1000.f, &x_iq_PI, &uq);		//this sort of works
	controller_PI(0.f, 	-i_d_meas, id_gain, 100000000.f, &x_id_PI, &ud);		//this sort of works

	const float clamp_v = .86f;
	if(uq > clamp_v)
		uq = clamp_v;
	else if (uq < -clamp_v)
		uq = -clamp_v;
	if(ud > clamp_v)
		ud = clamp_v;
	else if(ud < -clamp_v)
		ud = -clamp_v;

	uint32_t tA,tB,tC;
	inverse_park_transform(uq, ud, sin_theta,cos_theta, &i_alpha, &i_beta);	//maybe call theta rel again?sin_theta
	svm(i_alpha,i_beta,TIM1->ARR, &tA, &tB, &tC);
	TIMER_UPDATE_DUTY(tA,tB,tC);

	//float theta_m = unwrap(theta_abs_rad(), &theta_m_prev)*.5f;
//	gl_theta_m = theta_enc*.5f;

	float theta_enc = unwrap( theta_abs_rad(), &theta_enc_prev);
	gl_theta_m = theta_enc;
	if(HAL_GetTick() > dq_ts) //in this implementation, mechanical theta is computed outside the loop and shared with foc.
	{
		gl_dq = (theta_enc-prev_theta_m_dq)*25.0f;
		prev_theta_m_dq = theta_enc;
		phase_advance_angle = gl_dq/gl_prop_delay_const;//division yields more accurate advance angle for better performance.
		dq_ts = HAL_GetTick()+40;

		//clear_latched_nfault();
//		/*
//		 * TODO: swap the two drivers (my pop and pcbgogo mod). compare against module 019
//		 */
//		if(abs_f(gl_dq) < 20.f && abs_f(iqref) > 5.f && abs_f(i_q_meas) < 1.f)
//		{
//			HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 0);
//			unwrap( theta_rel_rad(), &theta_enc_prev);
//			HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);
//		}
	}
}



#endif /* FOC_COMMUTATION_H_ */
