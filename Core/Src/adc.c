/*
 * adc.c
 *
 *  Created on: Nov 17, 2017
 *      Author: Ocanath
 */

#include "adc.h"
#include "dartt_mctl_params.h"
#include "foc_fixed.h"
#include "fds.h"

#define BUFFER_SIZE 10

uint16_t dma_adc_raw[NUM_ADC] = {0};	//initialized, DMA access array

/*
 * Take ~100 samples of the current sensor and obtain the average. Dump that value to the filesystem midpoint current sensor calibration values.
 */
void get_currensensor_zeropoint(void)
{
	uint64_t avg_a = 0;
	uint64_t avg_b = 0;
	uint64_t avg_c = 0;
	TIMER_UPDATE_DUTY(0,0,0);	//short all 3 phase to ground to guarantee zero current
	HAL_Delay(10);	//in case you were spinning before.

	uint32_t num_samples = 0;
	for(uint32_t start_ts = gl_dp.tick; gl_dp.tick - start_ts < 100;)
	{
		avg_a += ((uint64_t)ADC1->JDR1);
		avg_b += ((uint64_t)ADC1->JDR2);
		avg_c += ((uint64_t)ADC1->JDR3);
		num_samples++;
		HAL_Delay(1);
	}
	avg_a /= num_samples;
	avg_b /= num_samples;
	avg_c /= num_samples;

	fds_mp->phase_a_mid = avg_a;
	fds_mp->phase_b_mid = avg_b;
	fds_mp->phase_c_mid = avg_c;
}

/*
 * Check the filesystem values for the phases. If they're zero (first flash) or -1 (firmware update to erased memory),
 * initialize them.
 */
void check_currentsensor_calibration(void)
{
	int32_t vals[] = {fds_mp->phase_a_mid, fds_mp->phase_b_mid, fds_mp->phase_c_mid};
	for(int i = 0; i < sizeof(vals)/sizeof(int32_t); i++)
	{
		if(vals[i] == -1 || vals[i] == 0)
		{
			get_currensensor_zeropoint();	//obtain new values
			m_write_flash((uint64_t*)fds_mp, sizeof(fds_motor_params_t)/sizeof(uint64_t));	//save them
			return;
		}
	}
}
