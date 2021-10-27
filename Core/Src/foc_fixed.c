#include "foc_fixed.h"
#include "mag-encoder.h"
#include "init.h"
#include "settings.h"
#include "CAN.h"
//static const int32_t SQRT_3 = int32(sqrt(3) * 2 ^ n);
//static const int32_t 	TWO_BY_SQRT_3 = int32((2 / sqrt(3)) * 2 ^ n);
//static const int32_t ONE_BY_SQRT_3 = int32((1 / sqrt(3)) * 2 ^ n);

/*Some constants for FOC math*/
#define SQRT_3_12B			7094
#define TWO_BY_SQRT_3_12B	4730
#define	ONE_BY_SQRT_3_12B	2365

/*Constant for overflow. This times overflow_count + return value is the gigantic unwrapped angle.
 * NOTE 1: THIS MUST BE A MULTIPLE OF TWO_PI_12b OR THE OUTPUT WILL NOT BE CORRECT
 * NOTE 2: IF THIS IS EQUAL TO TWO_PI_12b, overflow_count will be equal to ROTATIONS
 *
 * (in theory. Testing in matlab has proven confusing).
 * */
#define OVERFLOW_WRAP_VAL	1073757392

/*Static globals for current inputs*/
static int32_t current_input_offset_A = 0;
static int32_t current_input_offset_B = 0;
static int32_t current_input_offset_C = 0;

/*Static globals for sinusoidal*/
static int32_t phase_adv_12b = 0;
static int32_t v_12b = 0;
static int32_t prev_mtheta_12b = 0;
uint32_t count = 0;

/*Structs and variables for FOC controllers*/
typedef struct fixed_PI_params_t
{
	int32_t kp_m;
	int32_t kp_div;
	int32_t ki_m;
	int32_t ki_div;
	int32_t x;
	int32_t x_sat;
}fixed_PI_params_t;
fixed_PI_params_t g_iq = {
		.kp_m = 12288,
		.kp_div = 10,
		.ki_m = 4096,
		.ki_div = 1000,
		.x = 0,
		.x_sat = 2000
};
fixed_PI_params_t g_id = {
		.kp_m = 3,
		.kp_div = 100,
		.ki_m = 3,
		.ki_div = 100,
		.x = 0,
		.x_sat = 819
};



/*LOG*/
#define LOG_SIZE 3500
static volatile int32_t iq_log[LOG_SIZE] = {0};
static int32_t iq_log_idx = 0;
static uint32_t current_samp_count = 0;
static const uint32_t current_interval = 10;

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
	return u;
}

void sinusoidal_fixed(int32_t iq_in, int32_t id_in)
{
	int32_t theta_m = theta_rel_fixed();	//atan2, apply align offset
	int32_t theta_elec = wrap_2pi_12b(theta_m * elec_conv_ratio_fixed + phase_adv_12b);
	int32_t sin_val = sin_fixed(theta_elec);
	int32_t cos_val = cos_fixed(theta_elec);

	int32_t iq_meas, id_meas;
	get_current(sin_val, cos_val, &iq_meas, &id_meas);

	int32_t vq_ref = ctl_PI_fixed(iq_in, iq_meas, &g_iq);
	int32_t vd_ref = ctl_PI_fixed(id_in, id_meas, &g_iq);
//	vd_ref = 0;
//	vq_ref = 819;
	if(vq_ref > 1228)
		vq_ref = 1228;
	if(vq_ref < -1228)
		vq_ref = -1228;
	vd_ref = 0;
/*
	if(current_samp_count > current_interval)
	{
		if(iq_log_idx < LOG_SIZE)
			iq_log[iq_log_idx++] = iq_meas;
		else
		{
			TIMER_UPDATE_DUTY(0,0,0);
			for(int i = 0; i < LOG_SIZE; i++)
			{
				int32_fmt_t fmt;
				fmt.v = iq_log[i];
				HAL_UART_Transmit(&huart2,fmt.d,sizeof(int32_t),5);
			}
			while(1)
			{
				HAL_GPIO_TogglePin(STAT_PORT,STAT_PIN);
				HAL_Delay(50);
			}
			return;
		}
		current_samp_count = 0;
	}
	current_samp_count++;
*/



	int32_t alpha,beta,tA,tB,tC;
	inverse_park_fixed(vq_ref, vd_ref, sin_val,cos_val,&alpha,&beta);
	svm_fixed(alpha,beta,TIM1->ARR,&tA,&tB,&tC);
	TIMER_UPDATE_DUTY(tA,tB,tC);


//	if(count >= gl_prop_delayloop_interval)
//	{
//		/*This breaks if the rotor moves quickly enough. Consider reducing calculation period, downscaling
//		 * the velocity estimate, or using implementation of a fixed point unwrap subroutine.*/
//		v_12b = wrap_2pi_12b(theta_m - prev_mtheta_12b);	//no scaling factor. Only works if the rotor does not rotate by more than a full rotation between estimates.
//		phase_adv_12b = v_12b/gl_prop_delay_const_12b;
//		prev_mtheta_12b = theta_m;
//		count = 0;
//	}
//	count++;


}

void get_current(int32_t sth, int32_t cth, int32_t * iq, int32_t * id)
{
	int32_t ia = (int32_t)hadc1.Instance->JDR1 - current_input_offset_A;
	int32_t ib = (int32_t)hadc1.Instance->JDR2 - current_input_offset_B;
	int32_t ic = (int32_t)hadc1.Instance->JDR3 - current_input_offset_C;
	int32_t ialpha,ibeta;
	clarke_fixed(ia,ib,ic,&ialpha,&ibeta);
	park_fixed(ialpha,ibeta,sth,cth,iq,id);
}

void clarke_fixed(int32_t ia, int32_t ib, int32_t ic, int32_t * alpha, int32_t * beta)
{
	*alpha = ia;
	*beta = (ONE_BY_SQRT_3_12B*ia + TWO_BY_SQRT_3_12B*ib) >> 12;
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
void svm_fixed(int32_t alpha, int32_t beta, int32_t period_cnt, int32_t* tA, int32_t* tB, int32_t* tC)
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
	
	int32_t beta_by_sqrt3, two_by_sqrt3_beta, t1, t2, t3, t4, t5, t6;
	switch (sector)
	{
		case 1:
		{
			beta_by_sqrt3 = (ONE_BY_SQRT_3_12B * beta) >> 12;
			two_by_sqrt3_beta = (TWO_BY_SQRT_3_12B * beta) >> 12;

			t1 = ((alpha - beta_by_sqrt3) * pwm_half_period) >> 12;
			t2 = ((two_by_sqrt3_beta)*pwm_half_period) >> 12;
			*tA = ((pwm_half_period - t1 - t2) >> 1);
			*tB = *tA + t1;
			*tC = *tB + t2;
			break;
		}

		case 2:
		{
			beta_by_sqrt3 = (ONE_BY_SQRT_3_12B * beta) >> 12;

			t2 = (((alpha + beta_by_sqrt3) * pwm_half_period) >> 12);
			t3 = (((-alpha + beta_by_sqrt3) * pwm_half_period) >> 12);
			*tB = ((pwm_half_period - t2 - t3)>> 1);
			*tA = *tB + t3;
			*tC = *tA + t2;
			break;
		}
		case 3:
		{
			beta_by_sqrt3 = (ONE_BY_SQRT_3_12B * beta) >> 12;
			two_by_sqrt3_beta = (TWO_BY_SQRT_3_12B * beta) >> 12;

			t3 = (((two_by_sqrt3_beta)*pwm_half_period) >> 12);
			t4 = (((-alpha - beta_by_sqrt3) * pwm_half_period) >> 12);
			*tB = ((pwm_half_period - t3 - t4) >> 1);
			*tC = *tB + t3;
			*tA = *tC + t4;
			break;
		}

		case 4:
		{
			beta_by_sqrt3 = (ONE_BY_SQRT_3_12B * beta) >> 12;
			two_by_sqrt3_beta = (TWO_BY_SQRT_3_12B * beta) >> 12;

			t4 = (((-alpha + beta_by_sqrt3) * pwm_half_period) >> 12);
			t5 = (((-two_by_sqrt3_beta) * pwm_half_period) >> 12);
			*tC = ((pwm_half_period - t4 - t5) >> 1);
			*tB = *tC + t5;
			*tA = *tB + t4;
			break;
		}
		case 5:
		{
			beta_by_sqrt3 = (ONE_BY_SQRT_3_12B * beta) >> 12;

			t5 = (((-alpha - beta_by_sqrt3) * pwm_half_period) >> 12);
			t6 = (((alpha - beta_by_sqrt3) * pwm_half_period) >> 12);
			*tC = ((pwm_half_period - t5 - t6) >> 1);
			*tA = *tC + t5;
			*tB = *tA + t6;
			break;
		}

		case 6:
		{
			beta_by_sqrt3 = (ONE_BY_SQRT_3_12B * beta) >> 12;
			two_by_sqrt3_beta = (TWO_BY_SQRT_3_12B * beta) >> 12;

			t6 = (((-two_by_sqrt3_beta) * pwm_half_period) >> 12);
			t1 = (((alpha + beta_by_sqrt3) * pwm_half_period) >> 12);
			*tA = ((pwm_half_period - t6 - t1) >> 1);
			*tC = *tA + t1;
			*tB = *tC + t6;
			break;
		}
	};
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
		current_input_offset_A += hadc1.Instance->JDR1;
		current_input_offset_B += hadc1.Instance->JDR2;
		current_input_offset_C += hadc1.Instance->JDR3;
		HAL_Delay(1);
	}
	current_input_offset_A /= num_samples;
	current_input_offset_B /= num_samples;
	current_input_offset_C /= num_samples;
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
 * A vishan running at max speed should overflow this function once every ~12 seconds
 * */
int32_t unwrap_32(int32_t dif_wrapped, int32_t * prev_theta)
{
	//this is meant to be highly optimized, so dif_wrapped will be coming in
	int32_t unwrapped = *prev_theta + dif_wrapped;
	*prev_theta = unwrapped;
	return unwrapped;
}

/*
 * A vishan running at max speed will overflow this function in about 400 years, for OVERFLOW_WRAP_VAL = 1073757392
 * The output of a vishan at max speed for the same overflow val will wrap about every 5.96 seconds
 *
 *
 * Some notes on this.
 *
 * 1. This still underperforms compared to a 64bit unwrap with respect to RANGE, simply because 64 bit
 * integers are ENORMOUS. However, 400 years and 1600 years are both absurdly large values, and this
 * should theoretically be FASTER than the 64 bit unwrap.
 *
 * 2. You can downsample the call to this. You must, however, use only the count and remainder spat out by this function in
 * a SYNCHRONIZED FASHION, or else you can have large discontinuities in your output that wouldn't otherwise show up. One
 * possibility is to call it in your velocity estimator.
 * */
int32_t unwrap_32_overflow(int32_t dif_wrapped, int32_t * prev_theta, int32_t * overflow_cnt)
{
	/*TODO: test this*/
	int32_t unwrapped = *prev_theta + dif_wrapped;
	if(unwrapped >= OVERFLOW_WRAP_VAL)
	{
		unwrapped = unwrapped - OVERFLOW_WRAP_VAL;
		*overflow_cnt = *overflow_cnt + 1;
	}
	else if(unwrapped <= -OVERFLOW_WRAP_VAL)
	{
		unwrapped = unwrapped + OVERFLOW_WRAP_VAL;
		*overflow_cnt = *overflow_cnt - 1;
	}
	*prev_theta = unwrapped;
	return unwrapped;
}
