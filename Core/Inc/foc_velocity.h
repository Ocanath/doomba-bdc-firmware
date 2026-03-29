#ifndef FOC_VELOCITY_H
#define FOC_VELOCITY_H
#include <stdint.h>
#include "dartt_mctl_params.h"

#define VELOCITY_ESTIMATOR_MINIMUM_PERIOD_US	1000
#define DEFAULT_V_REPORT_RADIX	4	//report_velocity = true_velocity_radians_per_second * 2^DEFAULT_V_REPORT_RADIX

extern int32_t gl_dtheta_prev_theta;	//exposed for unit testing and for no other reason

void get_dtheta(uint32_t t_us);

#endif /* FOC_VELOCITY_H */
