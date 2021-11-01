/*
 * calibration_fixed.h
 *
 *  Created on: Oct 31, 2021
 *      Author: Ocanath
 */

#ifndef INC_CALIBRATION_FIXED_H_
#define INC_CALIBRATION_FIXED_H_
#include "foc_commutation.h"

int32_t run_v_fixed(int32_t v, uint32_t run_time, int64_t * dist);
void auto_calibrate_fixed(void);

#endif /* INC_CALIBRATION_FIXED_H_ */
