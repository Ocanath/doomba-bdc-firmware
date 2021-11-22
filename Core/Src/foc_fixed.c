#include "foc_fixed.h"
#include "mag-encoder.h"
#include "init.h"
#include "CAN.h"

/*Some constants for FOC math*/
#define SQRT_3_12B			7094	//(sqrt(3))*2^12
#define TWO_BY_SQRT_3_12B	4730	//(2/sqrt(3))*2^12
#define	ONE_BY_SQRT_3_12B	2365	//(1/sqrt(3))*2^12
#define ONE_THIRD_12B		1365

/*Constant for overflow. This times overflow_count + return value is the gigantic unwrapped angle.
 *
 * The remainder will range +/-half_overflow_val
 *
 * The true output is overflow_val*rot + rem
 *
 * half_overflow_val of 41721*PI_12B will net you
 * */
const int32_t half_overflow_val = PI_12B*41721;	// multiple of PI which is used for unwrapping with overflow.
const int32_t overflow_val = 2*half_overflow_val;

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
int32_t gl_theta_rem_m = 0;
int32_t gl_theta_rotations = 0;
int32_t gl_dtheta = 0;

/*Previous Theta*/
static int32_t prev_theta_m;


/*
 * x is state variable for integral delay
 */
int32_t ctl_PI_fixed(int32_t i_q_ref, int32_t i_q_meas, fixed_PI_params_t * p)
{
	int32_t err = i_q_ref - i_q_meas;
	int32_t u = p->x + (p->kp_m*err)/p->kp_div; //multiplying then dividing by scaled gains has the effect of leftshifting 12, then rightshifting 12
	p->x = p->x + (p->ki_m*err)/p->ki_div;
	int32_t sat = p->x_sat;
	if(p->x > sat)
		p->x = sat;
	if(p->x < -sat)
		p->x = -sat;
	return u >> 12;
}

/*
 * */
void sinusoidal_fixed(int32_t vq_in, int32_t vd_in)
{
	int32_t theta_m = theta_rel_fixed();	//atan2, apply align offset
	int32_t theta_elec = wrap_2pi_12b(theta_m * elec_conv_ratio_fixed + phase_adv_12b);
	int32_t sin_val = sin_12b(theta_elec);
	int32_t cos_val = cos_12b(theta_elec);

	gl_theta_rem_m = unwrap_32_overflow(theta_m, &prev_theta_m, &gl_theta_rotations);

	int32_t iq_meas, id_meas;
	get_current(sin_val, cos_val, &iq_meas, &id_meas);
	iq_meas = -iq_meas;
	id_meas = -id_meas;	//for some reason I don't understand, my park and clarkes return a sign-flipped iq and id result. Very odd


	int32_t alpha,beta,tA,tB,tC;
	inverse_park_fixed(vq_in, vd_in, sin_val,cos_val,&alpha,&beta);
	svm_fixed(alpha,beta,TIM1->ARR,&tA,&tB,&tC);
	TIMER_UPDATE_DUTY(tA,tB,tC);

	if(count >= gl_prop_delayloop_interval)
	{
		/*This breaks if the rotor moves quickly enough. Consider reducing calculation period, downscaling
		 * the velocity estimate, or using implementation of a fixed point unwrap subroutine.*/
		v_12b = wrap_2pi_12b(theta_m - prev_mtheta_12b);	//no scaling factor. Only works if the rotor does not rotate by more than a full rotation between estimates.
		phase_adv_12b = v_12b/gl_prop_delay_const_12b;
		prev_mtheta_12b = theta_m;
		count = 0;

		gl_dtheta = v_12b;
	}
	count++;

	gl_iq = iq_meas;
}

/*
 * */
void foc_fixed(int32_t iq_targ, int32_t id_targ)
{
	int32_t theta_m = theta_rel_fixed();	//atan2, apply align offset
	int32_t theta_elec = wrap_2pi_12b(theta_m * elec_conv_ratio_fixed + phase_adv_12b);
	int32_t sin_val = sin_12b(theta_elec);
	int32_t cos_val = cos_12b(theta_elec);

	gl_theta_rem_m = unwrap_32_overflow(theta_m, &prev_theta_m, &gl_theta_rotations);


	int32_t iq_meas, id_meas;
	get_current(sin_val, cos_val, &iq_meas, &id_meas);
	iq_meas = -iq_meas;
	id_meas = -id_meas;	//for some reason I don't understand, my park and clarkes return a sign-flipped iq and id result. Very odd

	int32_t vq = ctl_PI_fixed(iq_targ, iq_meas, &iq_pi);
	int32_t vd = ctl_PI_fixed(id_targ, id_meas, &id_pi);

	vq = saturate(vq,MAX_VQ);
	vd = saturate(vd,MAX_VQ);


	int32_t alpha,beta,tA,tB,tC;
	inverse_park_fixed(vq, vd, sin_val,cos_val,&alpha,&beta);
	svm_fixed(alpha,beta,COMM_PWM_PERIOD,&tA,&tB,&tC);
	TIMER_UPDATE_DUTY(tA,tB,tC);

	if(count >= gl_prop_delayloop_interval)
	{
		/*This breaks if the rotor moves quickly enough. Consider reducing calculation period, downscaling
		 * the velocity estimate, or using implementation of a fixed point unwrap subroutine.*/
		v_12b = wrap_2pi_12b(theta_m - prev_mtheta_12b);	//no scaling factor. Only works if the rotor does not rotate by more than a full rotation between estimates.
		phase_adv_12b = v_12b/gl_prop_delay_const_12b;
		prev_mtheta_12b = theta_m;
		count = 0;

		gl_dtheta = v_12b;
	}
	count++;

	gl_iq = iq_meas;
}

void get_current(int32_t sth, int32_t cth, int32_t * iq, int32_t * id)
{
	int32_t ia = (int32_t)( (hadc1.Instance->JDR1 << 1) + 1) - current_input_offset_A;
	int32_t ib = (int32_t)( (hadc1.Instance->JDR2 << 1) + 1) - current_input_offset_B;
	int32_t ic = (int32_t)( (hadc1.Instance->JDR3 << 1) + 1) - current_input_offset_C;
	ia = (ia*curr_scf[0]) >> 13;
	ib = (ib*curr_scf[1]) >> 13;
	ic = (ic*curr_scf[2]) >> 13;	//this is now equal to the current in amps * 4096
	int32_t ialpha,ibeta;
	clarke_fixed(ia,ib,ic,&ialpha,&ibeta);
	park_fixed(ialpha,ibeta,sth,cth,iq,id);
}

void clarke_fixed(int32_t ia, int32_t ib, int32_t ic, int32_t * alpha, int32_t * beta)
{
	*alpha = ia;
	*beta = (ONE_BY_SQRT_3_12B*ia + TWO_BY_SQRT_3_12B*ib) >> 12;

	/*This is the same as the original clarke, EXCEPT that it uses all 3 current measurements. Idk if it will do better or worse*/
//	*alpha = (ONE_THIRD_12B * (ia*2 - ib - ic) ) >> 12;	//this is the same as (2/3)*(ia - ib/2 - ic/2). We just do some shuffling about of factors of 2
//	*beta = ((SQRT_3_12B*ib - SQRT_3_12B*ic) >> 12) * ONE_THIRD_12B >> 12;	//
}

void park_fixed(int32_t alpha, int32_t beta, int32_t sin_theta, int32_t cos_theta, int32_t * iq, int32_t * id)
{
	*id = (alpha*cos_theta + beta*sin_theta) >> 12;
	*iq = (-alpha*sin_theta + beta*cos_theta) >> 12;
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
void inverse_park_fixed(int32_t iq, int32_t id, int32_t  sth, int32_t cth, int32_t * ialpha, int32_t* ibeta)
{
    /* 
        NOTE: If desired, it is possible to
        maintain a higher resolution of iqand id, if they are generated by
        the controller to have a higher res.This function removes the 2 ^ 12
        term from sin and cos, but not the 2 ^ n term from iq and id
    */
    *ialpha = (id * cth - iq * sth) >> 12;
    *ibeta = (id * sth + iq * cth) >> 12;
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

/*
 *
 * */
void get_current_cal_offsets(void)
{
	TIMER_UPDATE_DUTY(0,0,0);
	current_input_offset_A = 0;
	current_input_offset_B = 0;
	current_input_offset_C = 0;
	int i;
	int num_samples = 300;
	for(i=0;i<num_samples;i++)
	{
		current_input_offset_A += (hadc1.Instance->JDR1 << 1) + 1;
		current_input_offset_B += (hadc1.Instance->JDR2 << 1) + 1;
		current_input_offset_C += (hadc1.Instance->JDR3 << 1) + 1;
		HAL_Delay(1);
	}
	current_input_offset_A /= num_samples;
	current_input_offset_B /= num_samples;
	current_input_offset_C /= num_samples;
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

/*This function may only work if theta is bounded from -pi_12 to pi_12. Which it is, so it's fine
 * Stores the total elapsed radians, in 1lsb = 1/4096*pi rad
 *
 * A vishan running at max speed will overflow this function in about 1600 years
 *
 *
 * */
int64_t unwrap_64(int32_t theta, int64_t * prev_theta)
{
	return 0;
}

/*
 * A vishan running at max speed should overflow this function once every 100 seconds
 * */
int32_t unwrap_32(int32_t dif_wrapped, int32_t * prev_theta)
{
	//this is meant to be highly optimized, so dif_wrapped will be coming in pre-computed
	int32_t unwrapped = *prev_theta + dif_wrapped;
	*prev_theta = unwrapped;
	return unwrapped;
}

/*
 * A vishan running at max speed will overflow this function in 1.244 million years, if
 * the half overflow value is set to PI_12B*41721 (+/-30 bit range) and the overflow count is used
 *
 * A vishan running at max speed will overflow the remainder output of this function in about 25 seconds.
 * It is therefore acceptable to disregard the overflow count for most motors, and use only the remainder,
 * if the motor is geared down and the output of the gearbox is range-limited
 *
 *
 * Some notes on this.
 *
 * 1. This slightly underperforms compared to a straight up 64bit unwrap wrt. range, but should significantly outperform it wrt. speed
 * 	It is at best 1-2 cycles ( >= ) more than a 32bit unwrap,
 * 	and at worst 4-6 cycles more than a 32bit unwrap, and the
 * 	worst case only occurs every 25 seconds for the fastest
 * 	motor I currently use.
 *
 * 2. You should be able to downsample the call to this, as long as the downsample is frequent enough to count rotations.
 *
 * The logic behind this strategy warrants some explanation:
	 * Assumptions:
	 * 		The input dif_wrapped is bounded by -pi12 to pi12 (required, not enforced in this function)
	 * 		We must always set overflow wrap val to a multiple of pi12
	 * 		When it overflows half_overflow, it cannot exceed it by any multiple of PI12 greater than 1
	 *
	 * 		Therefore it is always safe to simply check against half overflow, and then subract twice overflow to
	 * 		wrap the remainder.
	 *
 * 		This function must be sampling the encoder signal fast enough that it does not miss full rotations.
 * 		It should compute fast enough that it can be called once per FOC loop without any degradation in FOC performance
 *
 *	ex of use:
 *		unwrap_32_overflow(theta_m, &prev_theta_m, &rot);
 * 		int64_t full_angle = overflow_val*rot + rem;
 * */
int32_t unwrap_32_overflow(int32_t theta, int32_t * prev_theta, int32_t * ovfl_cnt)
{
	int32_t dif_wrapped = wrap_2pi_12b(theta - *prev_theta);
	int32_t unwrapped = *prev_theta + dif_wrapped;

	if(unwrapped >= half_overflow_val)
	{
		unwrapped = unwrapped - overflow_val;
		*ovfl_cnt = *ovfl_cnt + 1;
	}
	else if(unwrapped <= -half_overflow_val)
	{
		unwrapped = unwrapped + overflow_val;
		*ovfl_cnt = *ovfl_cnt - 1;
	}

	*prev_theta = unwrapped;
	return unwrapped;
}
