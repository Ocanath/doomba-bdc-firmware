#ifndef  FOC_FIXED_H
#define FOC_FIXED_H
#include "trig_fixed.h"
#include "settings.h"

/*Define for low-noise velocity estimator, used exposed over CAN and used as a damping term*/
#define VELOCITY_ESTIMATOR_MINIMUM_PERIOD_US	1000


/*Current input offset and current scaling constants*/
extern int32_t current_input_offset_A;
extern int32_t current_input_offset_B;
extern int32_t current_input_offset_C;
extern int32_t curr_scf[3];

/*Global motor current, to share with the rest of the software*/
extern int32_t gl_iq;
extern int32_t gl_id;
extern int32_t gl_theta_rem_m;
extern int32_t gl_theta_rotations;
extern int32_t gl_dtheta_fixedpoint_rad_p_sec;
extern int32_t gl_dtheta_prev_theta;

/*Overflow value constants, for unwrap_overflow function*/
extern const int32_t half_overflow_val;
extern const int32_t overflow_val;

#define TIMER_UPDATE_DUTY(duty1, duty2, duty3) \
		TIM1->CR1 |= TIM_CR1_UDIS; \
		TIM1->CCR1 = duty1; \
		TIM1->CCR2 = duty2; \
		TIM1->CCR3 = duty3; \
		TIM1->CR1 &= ~TIM_CR1_UDIS;

/*clarke, park, inverse park, inverse clarke (svm)*/
int32_t ctl_PI_fixed(int32_t error, fixed_PI_params_t * p);
void clarke_fixed(int32_t ia, int32_t ib, int32_t ic, int32_t * alpha, int32_t * beta);
void park_fixed(int32_t alpha, int32_t beta, int32_t sin_theta, int32_t cos_theta, int32_t * iq, int32_t * id, uint8_t shift);
void inverse_park_fixed(int32_t iq, int32_t id, int32_t  sth, int32_t cth, int32_t * ialpha, int32_t* ibeta, uint8_t shift);
void svm_fixed(int32_t alpha, int32_t beta, int32_t period_cnt, int32_t* tA, int32_t* tB, int32_t* tC);
void get_current(int32_t sth, int32_t cth, int32_t * iq, int32_t * id, uint8_t shift);
void sinusoidal_fixed(int32_t vq_in, int32_t vd_in);
void foc_fixed(int32_t iq_targ, int32_t id_targ);
void get_current_cal_offsets(void);
int32_t saturate(int32_t v, int32_t sat);
int32_t unwrap_angle_32b_overflow(int32_t theta, int32_t * prev_theta, int32_t two_pi_fixed, int32_t * ovfl_cnt);
void get_dtheta(void);
int32_t ctl_PI_fixed_2(int32_t error, fixed_PI_2_params_t * p);
void openloop_fixed(int32_t angle_targ_prewrapping, int32_t vq);


#endif // ! FOC_FIXED_H
