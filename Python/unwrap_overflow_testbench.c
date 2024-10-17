#include "stdint.h"
#include "stdio.h"

#define MAX_INT32 	0x7FFFFFFF
#define MIN_INT32	-0x80000000


#define TWO_PI_12B              25736	
#define PI_12B                  12868
#define PI_14B 					51472
#define TWO_PI_14B 				102944

int32_t wrap_2pi_fixed(int32_t in, int32_t two_pi_fixed)
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

int32_t unwrap_angle_32b_overflow(int32_t theta, int32_t * prev_theta, int32_t two_pi_fixed, int32_t * ovfl_cnt)
{
	int ovfl = check_overflow32(theta, -*prev_theta);
	*prev_theta = *prev_theta - ovfl*two_pi_fixed;
	*ovfl_cnt -= ovfl;
	int32_t dif_wrapped = wrap_2pi_fixed(theta - *prev_theta, two_pi_fixed);
	int32_t unwrapped = *prev_theta + dif_wrapped;
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
	printf("Unwrapped, 2^31-1, modulo wrapper bound, count, time\n");
	const int32_t upperbound = 0x7FFFFFFF;
	for(int64_t i = 0; i < 8*TWO_PI_14B; i+= 1)
	{
		int32_t theta = wrap_2pi_fixed((int32_t)i, TWO_PI_14B);
		int32_t unwrapped_theta = unwrap_angle_32b_overflow(theta, &prev_theta, TWO_PI_14B, &ovfl_cnt);
		// if( i > 0x7FFFFFFF && i % 1000000 == 0)
		// printf("%d, %d, %d, %lld\n", unwrapped_theta, theta, ovfl_cnt, i);
		printf("%ld, %ld, %ld, %ld, %lld\n", unwrapped_theta, upperbound, 0x7FFFFFFF - (0x7FFFFFFF % PI_14B), ovfl_cnt, i);
	}
}

