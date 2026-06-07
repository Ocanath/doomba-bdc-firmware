#ifndef  TRIG_FIXED_H
#define TRIG_FIXED_H
#include <stdint.h>

/*Used for multiple q coeffs*/
typedef struct trig_fixed_coeffs_t
{
	int32_t half_pi_q;
	int32_t three_by_two_pi_q;
	int32_t pi_q;
	int32_t two_pi_q;
	int q;
	int32_t * coeffs;		
}trig_fixed_coeffs_t;


// _12B indicates that the constant is mutiplied by 2^12 and rounded to the nearest integer
#define HALF_PI_12B             6434	
#define THREE_BY_TWO_PI_12B     19302
#define PI_12B                  12868
#define TWO_PI_12B              25736	


#define HALF_PI_14B 			25736
#define THREE_BY_TWO_PI_14B 	77208
#define PI_14B 					51472
#define TWO_PI_14B 				102944


typedef struct unwrap_state_t
{
	int32_t unwrapped_angle;
	int32_t ovfl_cnt;
	int32_t prev_theta_wrapped;
}unwrap_state_t;

int32_t atan2_12b(int32_t y, int32_t x);
int32_t atan2_14b(int32_t y, int32_t x);
int32_t sin_12b(int32_t theta);
int32_t sin_14b(int32_t theta);
int32_t cos_12b(int32_t theta);
int32_t cos_14b(int32_t theta);
int32_t wrap_2pi_12b(int32_t in);
int32_t wrap_2pi_14b(int32_t in);
int32_t unwrap_angle_32b_overflow(int32_t theta, int32_t two_pi_fixed, unwrap_state_t * st);
int32_t sqrt_i32(int32_t v);
int64_t sqrt_i64(int64_t v);

int32_t wrap_2pi_q(int32_t in, int32_t two_pi_q);
int32_t atan2_q(int32_t y, int32_t x, const trig_fixed_coeffs_t * coefs);
int32_t cos_poly_q(int32_t theta, const trig_fixed_coeffs_t * coefs);
int32_t sin_poly_q(int32_t theta, const trig_fixed_coeffs_t * coefs);
void sin_cos_poly_q(int32_t theta, const trig_fixed_coeffs_t * coefs, int32_t * sin, int32_t * cos);


#endif // ! TRIG_FIXED_H

