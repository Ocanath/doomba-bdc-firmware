/*
 * calibration.h
 *
 *  Created on: Apr 29, 2019
 *      Author: Ocanath
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_
#include "init.h"
#include "foc_commutation.h"

void print_float(float v);
void print_str(const char * str);

float check_encoder_region();
void check_encoder_region_2(float start_offset);
float check_align_offset(uint32_t run_time, float tau_ref);
float sweep_align(float align_neg, float align_pos);
void manual_align_calib();
void test_foc();


#endif /* CALIBRATION_H_ */
