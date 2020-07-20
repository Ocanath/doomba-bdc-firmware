/*
 * foc_commutation.c
 *
 *  Created on: Mar 21, 2018
 *      Author: Ocanath
 */
#include "foc_commutation.h"
#include "adc.h"
//#include <math.h>
#include "mag-encoder.h"
float mL = 0;
float mR = 0;
float V_psi = 0;
float L;
float R;
float i_q_meas = 0;
float i_d_meas = 0;

float gl_theta_m = 0;
float gl_dq = 0;

uint32_t dq_ts = 0;
float prev_theta_m_dq = 0;
float phase_advance_angle = 0;

#define ONE_BY_SQRT_3 	0.57735026919
#define TWO_BY_SQRT_3 	1.15470053838
#define SQRT_3_BY_2 	0.866025404
#define SQRT_3 			1.7320508075688772935274463415f

//0.02929394531f;//TODO: use actual current shunt val and retune
//0.005754743303571;
const float adc_conv_A = 0.005754743303571;
const float adc_conv_B = 0.005754743303571;
const float adc_conv_C = 0.005754743303571;

int gl_sector=1;	//for SVM


float theta_enc_prev = 0;
float x_iq_PI = 0;
float x_id_PI = 0;


/*
 * procedure:
 * First divide (alpha,beta) vector into 4 quadrants.
 * Then, for each quadrant, check which sector. two possibilities for each.
 * once sectors are parsed, convert to times.
 */
int svm(float alpha, float beta, uint32_t pwm_period_cnt, uint32_t * tA, uint32_t * tB, uint32_t * tC)
{
	float pwm_half_period = (float)pwm_period_cnt;
	uint32_t sector;
	if(beta >= 0.0f)	//quadrant 1 or two
	{
		if(alpha >= 0.0f)	//quadrant 1
		{
			if(beta <= alpha*SQRT_3)	//sector 1
				sector = 1;
			else							//sector 2
				sector = 2;
		}
		else				//quadrant 2
		{
			if(beta <= alpha*-SQRT_3)	//sector 2
				sector = 3;
			else							//sector 3
				sector = 2;
		}
	}
	else			//quadrant 3 or 4
	{
		if(alpha >= 0.0f)	//quadrant 4
		{
			if(beta < -SQRT_3*alpha)
				sector = 5;
			else
				sector = 6;
		}
		else				//quadrant 3
		{
			if(beta < alpha*SQRT_3)
				sector = 5;
			else
				sector = 4;
		}
	}

	switch (sector)
	{
	case 1:
	{
		uint32_t t1 = (uint32_t)((alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t2 = (uint32_t)((TWO_BY_SQRT_3 * beta) * pwm_half_period);
		*tA = (pwm_half_period - t1 - t2) / 2;
		*tB = *tA + t1;
		*tC = *tB + t2;
		break;
	}
	case 2:
	{
		uint32_t t2 = (uint32_t)((alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t3 = (uint32_t)((-alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
		*tB = (pwm_half_period - t2 - t3) / 2;
		*tA = *tB + t3;
		*tC = *tA + t2;
		break;
	}
	case 3:
	{
		uint32_t t3 = (uint32_t)((TWO_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t4 = (uint32_t)((-alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
		*tB = (pwm_half_period - t3 - t4) / 2;
		*tC = *tB + t3;
		*tA = *tC + t4;
		break;
	}
	case 4:
	{
		uint32_t t4 = (uint32_t)((-alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t5 = (uint32_t)((-TWO_BY_SQRT_3 * beta) * pwm_half_period);
		*tC = (pwm_half_period - t4 - t5) / 2;
		*tB = *tC + t5;
		*tA = *tB + t4;

		break;
	}
	case 5:
	{
		uint32_t t5 = (uint32_t)((-alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t6 = (uint32_t)((alpha - ONE_BY_SQRT_3 * beta) * pwm_half_period);
		*tC = (pwm_half_period - t5 - t6) / 2;
		*tA = *tC + t5;
		*tB = *tA + t6;
		break;
	}
	case 6:
	{
		uint32_t t6 = (uint32_t)((-TWO_BY_SQRT_3 * beta) * pwm_half_period);
		uint32_t t1 = (uint32_t)((alpha + ONE_BY_SQRT_3 * beta) * pwm_half_period);
		*tA = (pwm_half_period - t6 - t1) / 2;
		*tC = *tA + t1;
		*tB = *tC + t6;
		break;
	}
	}
	gl_sector = sector;
	return sector;
}

const float Kcor = 0.0;
const float u_max = .3;
const float u_min = -.3;
/*
 * x is state variable for integral delay
 */
void controller_PI(float i_q_ref, float i_q, float Kp, float Ki, float * x, float * u)
{
	float err = i_q_ref-i_q;
	*u = *x + Kp*err;
	*x = *x + Ki*err;
}

void clarke_transform(float i_a, float i_b, float i_c, float * i_alpha, float * i_beta)
{
	(*i_alpha) = i_a;
	*i_beta = ONE_BY_SQRT_3*i_a + TWO_BY_SQRT_3*i_b;
}
void park_transform(float i_alpha, float i_beta, float sin_theta, float cos_theta, float * i_q, float * i_d)
{
	*i_d = i_alpha*cos_theta+i_beta*sin_theta;
	*i_q = -i_alpha*sin_theta+i_beta*cos_theta;
}

void inverse_clarke_transform(float i_alpha, float i_beta, float * i_a, float * i_b, float * i_c)
{
	*i_a = i_alpha;
	*i_b = -.5*i_alpha + SQRT_3_BY_2*i_beta;
	*i_c = -.5*i_alpha - SQRT_3_BY_2*i_beta;
}

void inverse_park_transform(float i_q, float i_d, float sin_theta, float cos_theta, float * i_alpha, float * i_beta)
{
	*i_alpha = i_d*cos_theta - i_q*sin_theta;
	*i_beta = i_d*sin_theta + i_q*cos_theta;
}



void get_current_cal_offsets()
{
	TIMER_UPDATE_DUTY(0,0,0);
	gl_current_input_offset_A = 0;
	gl_current_input_offset_B = 0;
	gl_current_input_offset_C = 0;
	int i;
	const int numSamples = 300;
	for(i=0;i<numSamples;i++)
	{
		gl_current_input_offset_A += dma_adc_raw[ADC_CHAN_CURRENT_A];
		gl_current_input_offset_B += dma_adc_raw[ADC_CHAN_CURRENT_B];
		gl_current_input_offset_C += dma_adc_raw[ADC_CHAN_CURRENT_C];
		HAL_Delay(1);
	}
	gl_current_input_offset_A /= numSamples;
	gl_current_input_offset_B /= numSamples;
	gl_current_input_offset_C /= numSamples;
}

//float time_seconds()
//{
//	return ((float)HAL_GetTick())*.001;
//}



/*
 *	convert each phase adc voltage to actual current value
 */
void conv_raw_current(float * i_a, float * i_b, float * i_c)
{
	*i_a = (float)(gl_current_input_offset_A - dma_adc_raw[ADC_CHAN_CURRENT_A])*adc_conv_A;
	*i_b = (float)(gl_current_input_offset_B - dma_adc_raw[ADC_CHAN_CURRENT_B])*adc_conv_B;
	*i_c = (float)(gl_current_input_offset_C - dma_adc_raw[ADC_CHAN_CURRENT_C])*adc_conv_C;
}

/*
 * returns phase voltage based on svm output
 * TODO: scale outputs to bus voltage
 */
void convert_phase_voltage(float * va, float * vb, float * vc)
{
	int16_t t1 = TIM1->CCR1;
	int16_t t2 = TIM1->CCR2;
	int16_t t3 = TIM1->CCR3;
	*va = (float)t1/1000.0;	//period
	*vb = (float)t2/1000.0;	//period
	*vc = (float)t3/1000.0;	//period
}



