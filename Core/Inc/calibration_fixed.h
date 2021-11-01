/*
 * calibration_fixed.h
 *
 *  Created on: Oct 31, 2021
 *      Author: Ocanath
 */

#ifndef INC_CALIBRATION_FIXED_H_
#define INC_CALIBRATION_FIXED_H_
#include "foc_commutation.h"

void buffer_data(uint8_t * d, int size_d, uint8_t * buf, int * bidx);
int32_t run_v_fixed(int32_t v, uint32_t run_time, int64_t * dist);
void auto_calibrate_fixed(void);
void current_sensors_fitness_test(void);
void get_cursensor_calibrations(int32_t test_voltage, int32_t w[NUM_CURRENT_SENSORS]);

#endif /* INC_CALIBRATION_FIXED_H_ */
