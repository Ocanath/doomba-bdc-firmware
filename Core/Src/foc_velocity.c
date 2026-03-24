#include "foc_velocity.h"

static uint32_t prev_us_dtheta = 0;
static int32_t v_uS_conv = 1000000 >> (14 - DEFAULT_V_REPORT_RADIX);

int32_t gl_dtheta_prev_theta = 0;

void get_dtheta(uint32_t t_us)
{
	uint32_t dif = t_us - prev_us_dtheta;	//elapsed time since last calculation
	if(dif >= VELOCITY_ESTIMATOR_MINIMUM_PERIOD_US)
	{
		int32_t dtheta = gl_dp.theta_rem_m - gl_dtheta_prev_theta;
		gl_dtheta_prev_theta = gl_dp.theta_rem_m;
		prev_us_dtheta = t_us;
		gl_dp.dtheta_fixedpoint_rad_p_sec = ((dtheta * v_uS_conv) / (int32_t)dif);
	}
}
