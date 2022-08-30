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

typedef enum {CAL_ERR_TIMEOUT, CAL_SUCCESS} regioncheck_retv;

void print_float(float v);
void print_str(const char * str);

float check_encoder_region(float start_step, uint32_t settle_time, float settle_error, float reverse_error, uint32_t timeout, uint8_t * status);
uint8_t check_encoder_region_2(uint32_t track_time, uint32_t timeout);

float check_align_offset(uint32_t run_time, float tau_ref);
void manual_align_calib();
void test_foc();
void force_encoder_region();
uint8_t verify_encoder_region(float tau, float diff_thresh, int pass_count_thresh, uint32_t timeout);
void auto_calibrate_motor(void);

int32_t elock_angle(int32_t vq, int32_t theta);
int32_t get_rotorlock_offsetdeviation(int32_t vq, int32_t theta, uint32_t init_settling_time_ms, int num_samples);
int32_t openloop_eang_pctl(int32_t ang, uint32_t exp_ts);

#endif /* CALIBRATION_H_ */
