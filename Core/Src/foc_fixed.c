#include <FDCAN.h>
#include "foc_fixed.h"
#include "mag-encoder.h"
#include "init.h"
#include "m_uart.h"
#include "PPP.h"

#define MAX_INT32 	0x7FFFFFFF
#define MIN_INT32	-0x80000000


/*Some constants for FOC math*/
#define SQRT_3_12B			7094	//(sqrt(3))*2^12
#define TWO_BY_SQRT_3_12B	4730	//(2/sqrt(3))*2^12
#define	ONE_BY_SQRT_3_12B	2365	//(1/sqrt(3))*2^12
#define ONE_THIRD_12B		1365


/*Static globals for current inputs*/
int32_t current_input_offset_A = 0;
int32_t current_input_offset_B = 0;
int32_t current_input_offset_C = 0;
//static int32_t curr_scf[3] = {31900, 49286, 90294}; //conversion constants for current measurements. Divide by 2^24 to obtain the current in amps.
//static int32_t curr_scf[3] = {31900, 64254, 136527};
int32_t curr_scf[3] = {31900, 61860, 151265}; //conversion constants. take true adc-> current constant and multiply by 2^24 to obtain these values

/*Static globals for sinusoidal*/
static int32_t phase_adv_12b = 0;
static int32_t v_12b = 0;
static int32_t prev_mtheta_12b = 0;
uint32_t count = 0;


/*Shared variables (current, unwrapped theta)*/
int32_t gl_iq = 0;
int32_t gl_id = 0;
int32_t gl_theta_rem_m = 0;
int32_t gl_theta_rotations = 0;
int32_t gl_dtheta_fixedpoint_rad_p_sec = 0;
int32_t gl_dtheta_prev_theta = 0;//helper variable for the previous theta used for dtheta calculation
int32_t gl_x_iqpi = 0;	//x for iq pi controller
int32_t gl_x_idpi = 0;	//x for id pi controller


/*Previous Theta for position estimator.*/
static int32_t prev_theta_m = 0;

/*This function estimates velocity. It executes
 * synchronously with the estimation of rotor angles,
 * periodically with counter logic. The periodicity
 * is not guaranteed; i.e. it is pre-empted by
 * the execution of the main loop. This means the
 * period is *at minimum* the period specified.
 *
 * The rationale here is that it's more important
 * to be accurate about time elapsed than to enforce
 * 0 jitter in the estimation period. So we have
 * a jittery estimator period, that is synchronous
 * with rotor position calculation, and a precise
 * value for elapsed time in ms.*/
void get_dtheta(void)
{
	int32_t cnt = TIM2->CNT;
	if(cnt >= VELOCITY_ESTIMATOR_MINIMUM_PERIOD_US)
	{

		int32_t dtheta = gl_theta_rem_m - gl_dtheta_prev_theta;
		gl_dtheta_prev_theta = gl_theta_rem_m;
		cnt = TIM2->CNT;	//read one final time for best accuracy
		TIM2->CNT = 0;
		gl_dtheta_fixedpoint_rad_p_sec = ((dtheta * gl_v_uS_conv) / cnt);
	}
}


int32_t ctl_PI_fixed_2(int32_t error, fixed_PI_2_params_t * p)
{
	//NOTE: out_rshift applied TWICE, to avoid x overflow, while maintaining thesame radix when adding x and effort to get u.
	p->x = p->x + ((p->ki.i32*error)/p->x_integral_div);	//accumulate x with controlled slope: two controls are div scaling factor, and frequency of control law updates
	int32_t sat = p->x_sat << (p->out_rshift + p->ki.radix);
	if(p->x > sat)
		p->x = sat;
	if(p->x < -sat)
		p->x = -sat;
	int32_t u = ((p->x >> p->ki.radix) + ((p->kp.i32*error) >> p->kp.radix)) >> p->out_rshift;	//one application of outshift
	return u;
}

/*
 * x is state variable for integral delay
 */
int32_t ctl_PI_fixed_3(int32_t error, fixed_PI_params_t * p, int32_t * x)
{
	//NOTE: out_rshift applied TWICE, to avoid x overflow, while maintaining thesame radix when adding x and effort to get u.
	*x = *x + ((p->ki.i32*error)/p->x_integral_div);	//accumulate x with controlled slope: two controls are div scaling factor, and frequency of control law updates
	int32_t sat = p->x_sat << (p->out_rshift + p->ki.radix);
	if(*x > sat)
		*x = sat;
	if(*x < -sat)
		*x = -sat;
	int32_t u = ((*x >> p->ki.radix) + ((p->kp.i32*error) >> p->kp.radix)) >> p->out_rshift;	//one application of outshift
	return u;
}


int32_t get_theta_m(void)
{
	int32_t theta_m = theta_rel_14b()+TWO_PI_14B;	//atan2, apply align offset
	gl_theta_rem_m = unwrap_angle_32b_overflow(theta_m, &prev_theta_m, TWO_PI_14B, &gl_theta_rotations);
	return gl_theta_rem_m;
}

/*
 * */
void sinusoidal_fixed(int32_t vq_in, int32_t vd_in)
{
	int32_t theta_m = theta_rel_14b()+TWO_PI_14B;	//atan2, apply align offset
	int32_t theta_elec = wrap_2pi_14b(theta_m * (fds_mp->elec_conv_ratio_fixed) + phase_adv_12b);
	int32_t sin_val = sin_14b(theta_elec);
	int32_t cos_val = cos_14b(theta_elec);

	gl_theta_rem_m = unwrap_angle_32b_overflow(theta_m, &prev_theta_m, TWO_PI_14B, &gl_theta_rotations);

	int32_t iq_meas, id_meas;
	get_current(sin_val, cos_val, &iq_meas, &id_meas, 14);

	int32_t vd = ctl_PI_fixed_3(vd_in - id_meas, &(fds_mp->id_pi), &gl_x_idpi);

	int32_t alpha,beta,tA,tB,tC;
	inverse_park_fixed(vq_in, vd, sin_val,cos_val,&alpha,&beta, 14);
	svm_fixed(alpha,beta,TIM1->ARR,&tA,&tB,&tC);
	TIMER_UPDATE_DUTY(tA,tB,tC);

	if(count >= fds_mp->gl_prop_delayloop_interval)
	{
		/*This breaks if the rotor moves quickly enough. Consider reducing calculation period, downscaling
		 * the velocity estimate, or using implementation of a fixed point unwrap subroutine.*/
		v_12b = wrap_2pi_12b(theta_m - prev_mtheta_12b);	//no scaling factor. Only works if the rotor does not rotate by more than a full rotation between estimates.
		phase_adv_12b = v_12b/fds_mp->gl_prop_delay_const_12b;
		prev_mtheta_12b = theta_m;
		count = 0;
	}
	count++;

	gl_iq = iq_meas;
	gl_id = id_meas;
}

void openloop_fixed(int32_t angle_targ_prewrapping, int32_t vq, int32_t vd)
{
	int32_t theta_wrapped = wrap_2pi_14b(angle_targ_prewrapping);
	int32_t sin_val = sin_14b(theta_wrapped);
	int32_t cos_val = cos_14b(theta_wrapped);
	int32_t alpha,beta,tA,tB,tC;
	inverse_park_fixed(vq, vd, sin_val,cos_val,&alpha,&beta, 14);
	svm_fixed(alpha,beta,TIM1->ARR,&tA,&tB,&tC);
	TIMER_UPDATE_DUTY(tA,tB,tC);
}

/*
 * */
void foc_fixed(int32_t iq_targ, int32_t id_targ)
{
	int32_t theta_m = theta_rel_14b()+TWO_PI_14B;	//atan2, apply align offset
	int32_t theta_elec = wrap_2pi_14b(theta_m * fds_mp->elec_conv_ratio_fixed + phase_adv_12b);
	int32_t sin_val = sin_14b(theta_elec);
	int32_t cos_val = cos_14b(theta_elec);

	gl_theta_rem_m = unwrap_angle_32b_overflow(theta_m, &prev_theta_m, TWO_PI_14B, &gl_theta_rotations);


	int32_t iq_meas, id_meas;
	get_current(sin_val, cos_val, &iq_meas, &id_meas, 14);

	int32_t vq = ctl_PI_fixed_3(iq_targ -iq_meas, &(fds_mp->iq_pi), &gl_x_iqpi);
	int32_t vd = ctl_PI_fixed_3(id_targ -id_meas, &(fds_mp->id_pi), &gl_x_idpi);

	vq = saturate(vq,MAX_VQ);
	vd = saturate(vd,MAX_VQ);


	int32_t alpha,beta,tA,tB,tC;
	inverse_park_fixed(vq, vd, sin_val,cos_val,&alpha,&beta, 14);
	svm_fixed(alpha,beta,COMM_PWM_PERIOD,&tA,&tB,&tC);
	TIMER_UPDATE_DUTY(tA,tB,tC);

	if(count >= fds_mp->gl_prop_delayloop_interval)
	{
		/*This breaks if the rotor moves quickly enough. Consider reducing calculation period, downscaling
		 * the velocity estimate, or using implementation of a fixed point unwrap subroutine.*/
		v_12b = wrap_2pi_12b(theta_m - prev_mtheta_12b);	//no scaling factor. Only works if the rotor does not rotate by more than a full rotation between estimates.
		phase_adv_12b = v_12b/fds_mp->gl_prop_delay_const_12b;
		prev_mtheta_12b = theta_m;
		count = 0;
	}
	count++;

	gl_iq = iq_meas;
	gl_id = id_meas;
}

void get_current(int32_t sth, int32_t cth, int32_t * iq, int32_t * id, uint8_t shift)
{
	int32_t ia = ((int16_t)ADC1->JDR1) - 1990;
	int32_t ib = ((int16_t)ADC1->JDR2) - 1990;
	int32_t ic = ((int16_t)ADC1->JDR3) - 1990;

	int32_t ialpha,ibeta;

	//software equivalent of rewiring the two phases
	if(fds_mp->is_flipped == 0)
		clarke_fixed(ia,ib,ic,&ialpha,&ibeta);
	else
		clarke_fixed(ib,ia,ic,&ialpha,&ibeta);

	park_fixed(ialpha,ibeta,sth,cth,iq,id, shift);
}

void clarke_fixed(int32_t ia, int32_t ib, int32_t ic, int32_t * alpha, int32_t * beta)
{
	*alpha = ia;
	*beta = (ONE_BY_SQRT_3_12B*ia + TWO_BY_SQRT_3_12B*ib) >> 12;

	/*This is the same as the original clarke, EXCEPT that it uses all 3 current measurements. Idk if it will do better or worse*/
//	*alpha = (ONE_THIRD_12B * (ia*2 - ib - ic) ) >> 12;	//this is the same as (2/3)*(ia - ib/2 - ic/2). We just do some shuffling about of factors of 2
//	*beta = ((SQRT_3_12B*ib - SQRT_3_12B*ic) >> 12) * ONE_THIRD_12B >> 12;	//
}

void park_fixed(int32_t alpha, int32_t beta, int32_t sin_theta, int32_t cos_theta, int32_t * iq, int32_t * id, uint8_t shift)
{
	*id = (alpha*cos_theta + beta*sin_theta) >> shift;
	*iq = (-alpha*sin_theta + beta*cos_theta) >> shift;
}

/* 
Inputs:
     iq, id, desired
     sth, cth measured
     IMPORTANT NOTE :
         Fixed point range of iqand id is not full.If we use 2 ^ 12, the
         true range will be(sqrt(3) / 2) * 2 ^ 12, or +/ -3547

Outputs :
     ialpha, ibeta in whichever scale iqand id were passed
*/
void inverse_park_fixed(int32_t iq, int32_t id, int32_t  sth, int32_t cth, int32_t * ialpha, int32_t* ibeta, uint8_t shift)
{
    /* 
        NOTE: If desired, it is possible to
        maintain a higher resolution of iqand id, if they are generated by
        the controller to have a higher res.This function removes the 2 ^ 12
        term from sin and cos, but not the 2 ^ n term from iq and id
    */
    *ialpha = (id * cth - iq * sth) >> shift;
    *ibeta = (id * sth + iq * cth) >> shift;
}

const int32_t two_by_three_sqrt3 = 4730;	//sqrt(3)*2/3*4096
const int32_t harmonic_mag = 788;	//first harmonic magnitude divided by 6, with scaling vactor for normalization 0-1
int32_t sin_third_harmonic_fixed(int32_t theta)
{
	theta = wrap_2pi_12b(theta);
	int32_t theta3 = wrap_2pi_12b(theta*3);
	int32_t v = sin_12b(theta)*two_by_three_sqrt3 / 4096;
	v += sin_12b(theta3)*harmonic_mag / 4096;
	return v;
}
const int32_t pi_by_3 = PI_12B/3;
const int32_t two_pi_by_3 = TWO_PI_12B/3;
void generate_3rdharmonic_waveforms(int32_t theta, int32_t m, int32_t half_period, int32_t* p_tA, int32_t* p_tB, int32_t* p_tC)
{
	*p_tA = (m*sin_third_harmonic_fixed(theta))/(1<<13) + half_period;
	*p_tB = (-m*sin_third_harmonic_fixed(theta+pi_by_3))/(1<<13) + half_period;
	*p_tC = (m*sin_third_harmonic_fixed(theta+two_pi_by_3))/(1<<13) + half_period;
}
const int32_t half_period_val = COMM_PWM_PERIOD/2;
/*sinusoidal mode, no clarke-park, direct voltage in PWM space*/
void voltage_3rd_harmonic(int32_t mag)
{
	int32_t theta_m = theta_rel_12b();	//atan2, apply align offset
	int32_t theta_elec = wrap_2pi_12b(theta_m * fds_mp->elec_conv_ratio_fixed + phase_adv_12b);

	gl_theta_rem_m = unwrap_angle_32b_overflow(theta_m, &prev_theta_m, TWO_PI_14B, &gl_theta_rotations);

	int32_t tA,tB,tC;
	generate_3rdharmonic_waveforms(theta_elec, mag, half_period_val, &tA, &tB, &tC);
	TIMER_UPDATE_DUTY(tA,tB,tC);

	if(count >= fds_mp->gl_prop_delayloop_interval)
	{
		/*This breaks if the rotor moves quickly enough. Consider reducing calculation period, downscaling
		 * the velocity estimate, or using implementation of a fixed point unwrap subroutine.*/
		v_12b = wrap_2pi_12b(theta_m - prev_mtheta_12b);	//no scaling factor. Only works if the rotor does not rotate by more than a full rotation between estimates.
		phase_adv_12b = v_12b/fds_mp->gl_prop_delay_const_12b;
		prev_mtheta_12b = theta_m;
		count = 0;
	}
	count++;
}

void volage_pure_sinusoidal(int32_t m)
{
	int32_t theta_m = theta_rel_12b();	//atan2, apply align offset
	int32_t theta_elec = wrap_2pi_12b(theta_m * fds_mp->elec_conv_ratio_fixed + phase_adv_12b);

	gl_theta_rem_m = unwrap_angle_32b_overflow(theta_m, &prev_theta_m, TWO_PI_14B, &gl_theta_rotations);

	int32_t theta2 = wrap_2pi_12b(theta_elec+pi_by_3);
	int32_t theta3 = wrap_2pi_12b(theta_elec+two_pi_by_3);

	int32_t tA,tB,tC;
	tA = m*sin_12b(theta_elec)/(1<<13) + half_period_val;
	tB = -m*sin_12b(theta2)/(1<<13) + half_period_val;
	tC = m*sin_12b(theta3)/(1<<13) + half_period_val;

	TIMER_UPDATE_DUTY(tA,tB,tC);


	if(count >= fds_mp->gl_prop_delayloop_interval)
	{
		/*This breaks if the rotor moves quickly enough. Consider reducing calculation period, downscaling
		 * the velocity estimate, or using implementation of a fixed point unwrap subroutine.*/
		v_12b = wrap_2pi_12b(theta_m - prev_mtheta_12b);	//no scaling factor. Only works if the rotor does not rotate by more than a full rotation between estimates.
		phase_adv_12b = v_12b/fds_mp->gl_prop_delay_const_12b;
		prev_mtheta_12b = theta_m;
		count = 0;
	}
	count++;
}

/*A.K.A. Inverse Clarke*/
void svm_fixed(int32_t alpha, int32_t beta, int32_t period_cnt, int32_t* p_tA, int32_t* p_tB, int32_t* p_tC)
{
	int32_t pwm_half_period = period_cnt;
	int32_t alpha_sqrt3 = (alpha * SQRT_3_12B) >> 12;
	int32_t sector = 0;
	if (beta >= 0) // //quadrant 1 or two
	{
		if (alpha >= 0) // quadrant 1
		{
			if (beta <= alpha_sqrt3) // sector 1
				sector = 1;
			else// sector 2
				sector = 2;
		}
		else// quadrant 2
		{
			if (beta <= -alpha_sqrt3) // sector 2
				sector = 3;
			else// sector 3
				sector = 2;
		}
	}
	else
	{	//// quadrant 3 or 4
		if (alpha >= 0) // quadrant 4
		{
			if (beta < -alpha_sqrt3)
				sector = 5;
			else
				sector = 6;
		}
		else// quadrant 3
		{
			if (beta < alpha_sqrt3)
				sector = 5;
			else
				sector = 4;
		}
	}
	
	int32_t beta_by_sqrt3, two_by_sqrt3_beta, t1, t2, t3, t4, t5, t6, tA, tB, tC;
	switch (sector)
	{
		case 1:
		{
			beta_by_sqrt3 = (ONE_BY_SQRT_3_12B * beta) >> 12;
			two_by_sqrt3_beta = (TWO_BY_SQRT_3_12B * beta) >> 12;

			t1 = ((alpha - beta_by_sqrt3) * pwm_half_period) >> 12;
			t2 = ((two_by_sqrt3_beta)*pwm_half_period) >> 12;
			tA = ((pwm_half_period - t1 - t2) >> 1);
			tB = tA + t1;
			tC = tB + t2;
			break;
		}

		case 2:
		{
			beta_by_sqrt3 = (ONE_BY_SQRT_3_12B * beta) >> 12;

			t2 = (((alpha + beta_by_sqrt3) * pwm_half_period) >> 12);
			t3 = (((-alpha + beta_by_sqrt3) * pwm_half_period) >> 12);
			tB = ((pwm_half_period - t2 - t3)>> 1);
			tA = tB + t3;
			tC = tA + t2;
			break;
		}
		case 3:
		{
			beta_by_sqrt3 = (ONE_BY_SQRT_3_12B * beta) >> 12;
			two_by_sqrt3_beta = (TWO_BY_SQRT_3_12B * beta) >> 12;

			t3 = (((two_by_sqrt3_beta)*pwm_half_period) >> 12);
			t4 = (((-alpha - beta_by_sqrt3) * pwm_half_period) >> 12);
			tB = ((pwm_half_period - t3 - t4) >> 1);
			tC = tB + t3;
			tA = tC + t4;
			break;
		}

		case 4:
		{
			beta_by_sqrt3 = (ONE_BY_SQRT_3_12B * beta) >> 12;
			two_by_sqrt3_beta = (TWO_BY_SQRT_3_12B * beta) >> 12;

			t4 = (((-alpha + beta_by_sqrt3) * pwm_half_period) >> 12);
			t5 = (((-two_by_sqrt3_beta) * pwm_half_period) >> 12);
			tC = ((pwm_half_period - t4 - t5) >> 1);
			tB = tC + t5;
			tA = tB + t4;
			break;
		}
		case 5:
		{
			beta_by_sqrt3 = (ONE_BY_SQRT_3_12B * beta) >> 12;

			t5 = (((-alpha - beta_by_sqrt3) * pwm_half_period) >> 12);
			t6 = (((alpha - beta_by_sqrt3) * pwm_half_period) >> 12);
			tC = ((pwm_half_period - t5 - t6) >> 1);
			tA = tC + t5;
			tB = tA + t6;
			break;
		}

		case 6:
		{
			beta_by_sqrt3 = (ONE_BY_SQRT_3_12B * beta) >> 12;
			two_by_sqrt3_beta = (TWO_BY_SQRT_3_12B * beta) >> 12;

			t6 = (((-two_by_sqrt3_beta) * pwm_half_period) >> 12);
			t1 = (((alpha + beta_by_sqrt3) * pwm_half_period) >> 12);
			tA = ((pwm_half_period - t6 - t1) >> 1);
			tC = tA + t1;
			tB = tC + t6;
			break;
		}
		default :
		{
			tA = 0;
			tB = 0;
			tC = 0;
		}
	};

	int32_t max_pwm = period_cnt - 5;
	if(fds_mp->is_flipped == 0)
	{
		if(tA > max_pwm)
			*p_tA = max_pwm;
		else if(tA < 0)
			*p_tA = 0;
		else
			*p_tA = tA;

		if(tB > max_pwm)
			*p_tB = max_pwm;
		else if(tB < 0)
			*p_tB = 0;
		else
			*p_tB = tB;

		if(tC > max_pwm)
			*p_tC = max_pwm;
		else if(tC < 0)
			*p_tC = 0;
		else
			*p_tC = tC;
	}
	else
	{
		if(tA > max_pwm)
			*p_tB = max_pwm;
		else if(tA < 0)
			*p_tB = 0;
		else
			*p_tB = tA;

		if(tB > max_pwm)
			*p_tA = max_pwm;
		else if(tB < 0)
			*p_tA = 0;
		else
			*p_tA = tB;

		if(tC > max_pwm)
			*p_tC = max_pwm;
		else if(tC < 0)
			*p_tC = 0;
		else
			*p_tC = tC;
	}
}

/*
 *
 * */
void get_current_cal_offsets(void)
{
	TIMER_UPDATE_DUTY(500,500,500);
	current_input_offset_A = 0;
	current_input_offset_B = 0;
	current_input_offset_C = 0;
	int i;
	int num_samples = 300;
	for(i=0;i<num_samples;i++)
	{
		int32_t ia = dma_adc_raw[0];
		int32_t ib = dma_adc_raw[1];
		current_input_offset_A += ia;
		current_input_offset_B += ib;
		HAL_Delay(1);
	}
	current_input_offset_A /= num_samples;
	current_input_offset_B /= num_samples;
}

/*
 * Saturate output
 * */
int32_t saturate(int32_t v, int32_t sat)
{
	if(v > sat)
		return sat;
	if(v < -sat)
		return -sat;
	return v;
}

/*
 *
 * */
int32_t check_overflow32(int32_t a, int32_t b)
{

	if(a > 0)
	{
		int32_t upperbound = (MAX_INT32 - a);
		if(b > upperbound)
		{
			return 1;
		}
		else
			return 0;
	}
	else if (a < 0)
	{
		int32_t lowerbound = (MIN_INT32 - a);
		if(b < lowerbound)
		{
			return -1;
		}
		else
			return 0;
	}
	else
		return 0;
}

/*
 *
 * */
int32_t unwrap_angle_32b_overflow(int32_t theta, int32_t * prev_theta, int32_t two_pi_fixed, int32_t * ovfl_cnt)
{
	int ovfl = check_overflow32(theta, -*prev_theta);
	if(ovfl < 0)
	{
		*prev_theta = *prev_theta - ovfl*two_pi_fixed/2;
	}
	else
	{
		*prev_theta = *prev_theta - ovfl*two_pi_fixed;
	}
	*ovfl_cnt -= ovfl;
	int32_t dif_wrapped = wrap_2pi_fixed(theta - *prev_theta, two_pi_fixed);
	int32_t unwrapped = *prev_theta + dif_wrapped;
	*prev_theta = unwrapped;
	return unwrapped;
}



