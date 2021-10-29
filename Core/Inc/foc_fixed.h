#ifndef  FOC_FIXED_H
#define FOC_FIXED_H
#include "trig_fixed.h"

#define MAX_VQ 3546

extern int32_t current_input_offset_A;
extern int32_t current_input_offset_B;
extern int32_t current_input_offset_C;

typedef struct fixed_PI_params_t
{
	int32_t kp_m;
	int32_t kp_div;
	int32_t ki_m;
	int32_t ki_div;
	int32_t x;
	int32_t x_sat;
}fixed_PI_params_t;

#define TIMER_UPDATE_DUTY(duty1, duty2, duty3) \
		TIM1->CR1 |= TIM_CR1_UDIS; \
		TIM1->CCR1 = duty1; \
		TIM1->CCR2 = duty2; \
		TIM1->CCR3 = duty3; \
		TIM1->CR1 &= ~TIM_CR1_UDIS;

/*clarke, park, inverse park, inverse clarke (svm)*/
int32_t ctl_PI_fixed(int32_t i_q_ref, int32_t i_q_meas, fixed_PI_params_t * p);
void clarke_fixed(int32_t ia, int32_t ib, int32_t ic, int32_t * alpha, int32_t * beta);
void park_fixed(int32_t alpha, int32_t beta, int32_t sin_theta, int32_t cos_theta, int32_t * iq, int32_t * id);
void inverse_park_fixed(int32_t iq, int32_t id, int32_t  sth, int32_t cth, int32_t* ialpha, int32_t* ibeta);
void svm_fixed(int32_t alpha, int32_t beta, int32_t period_cnt, int32_t* tA, int32_t* tB, int32_t* tC);
void get_current(int32_t sth, int32_t cth, int32_t * iq, int32_t * id);
void sinusoidal_fixed(int32_t iq, int32_t id);
void get_current_cal_offsets(void);

#endif // ! FOC_FIXED_H
