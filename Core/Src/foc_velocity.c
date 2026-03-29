#include "foc_velocity.h"

static uint32_t prev_us_dtheta = 0;
static int32_t v_uS_conv = 1000000 / (1 << (14 - DEFAULT_V_REPORT_RADIX));

int32_t gl_dtheta_prev_theta = 0;

void get_dtheta(uint32_t t_us)
{
	uint32_t dif = t_us - prev_us_dtheta;	//elapsed time since last calculation
	if(dif >= VELOCITY_ESTIMATOR_MINIMUM_PERIOD_US)
	{
		//C language hack: take advantage of unsigned subtraction being well defined to deal with potential
		//integer overflow here. The only risk here is that casting from uint32_t back to int32_t is implementation 
		//defined in C99/C11 - but GCC uses 2's complement, as does ARM, so this is well defined even on C99/11 - just
		//not necessarily well defined by the C standard. And it is well defined in C23. Plus, this is the fastest way
		//to properly compute dtheta in an overflow case.
		int32_t dtheta = (int32_t)((uint32_t)gl_dp.theta_rem_m - (uint32_t)gl_dtheta_prev_theta);
		gl_dtheta_prev_theta = gl_dp.theta_rem_m;
		prev_us_dtheta = t_us;
		gl_dp.dtheta_fixedpoint_rad_p_sec = ((dtheta * v_uS_conv) / (int32_t)dif);
	}
}
