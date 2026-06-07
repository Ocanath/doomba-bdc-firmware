#include "stdint.h"
#include "stdio.h"

#define MAX_INT32 	0x7FFFFFFF
#define MIN_INT32	-0x80000000


#define TWO_PI_12B              25736	
#define PI_12B                  12868
#define PI_14B 					51472
#define TWO_PI_14B 				102944

int32_t wrap_2pi_q(int32_t in, int32_t two_pi_fixed)
{
	int32_t pi_fixed = two_pi_fixed / 2;
    int32_t result = ((in + pi_fixed) % two_pi_fixed) - pi_fixed;
    if (in < -pi_fixed) //if( (in + pi_fixed) < 0 )
        return two_pi_fixed + result;
    else
        return result;
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
	'rewrap' the angle by unwrapping it in rotations but rewrapping it to the maximum possible multiple of pi with the given radix in a 32 bit container
 */
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
	int32_t dif_wrapped = wrap_2pi_q(theta - *prev_theta, two_pi_fixed);
	int32_t unwrapped = *prev_theta + dif_wrapped;
	*prev_theta = unwrapped;
	return unwrapped;
}

/*
Take an angle and unwrap it in rotations, but rewrap it in some larger multiple of rotations
 */
int32_t rewrap_angle_32b(int32_t theta, int32_t * prev_theta, int32_t two_pi_fixed, int32_t multiple, int32_t * ovfl_cnt)
{
	int32_t dif_wrapped = wrap_2pi_q(theta - *prev_theta, two_pi_fixed);
	int32_t unwrapped = wrap_2pi_q(*prev_theta + dif_wrapped, two_pi_fixed*multiple);
	*prev_theta = unwrapped;
	return unwrapped;
}


int64_t unwrap_counted_wrapped_angle(int32_t angle, int32_t two_pi, int32_t count)
{
	// int64_t output = angle + (MAX_INT32 - MAX_INT32 % two_pi)
}

int main(void)
{
	int32_t prev_theta = (0x7FFFFFFF-4*TWO_PI_14B);
	int32_t ovfl_cnt = 0;
	// for(int64_t i = -0x7FFFFFFFFFFFFFFF; i < 0x7FFFFFFFFFFFFFFF; i+= 1000000)
	printf("theta, megawrapped, upperbound, lowerbound, count, time\n");
	const int32_t upperbound = (0x7FFFFFFF - (0x7FFFFFFF % PI_14B));
	const int32_t lowerbound = MIN_INT32 + (MIN_INT32 % PI_14B);//MIN_INT32 - (MIN_INT32 % PI_14B);//(-4294967296 - );
	for(int64_t i = 0; i < 8*TWO_PI_14B; i+= 1)
	{
		int32_t theta = wrap_2pi_q((int32_t)i, TWO_PI_14B);
		int32_t unwrapped_theta = unwrap_angle_32b_overflow(theta, &prev_theta, TWO_PI_14B, &ovfl_cnt);
		printf("%ld, %ld, %ld, %ld, %ld, %lld\n", theta, unwrapped_theta, upperbound, lowerbound, ovfl_cnt, i);
	}
}

