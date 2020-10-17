/*
 * adc.c
 *
 *  Created on: Nov 17, 2017
 *      Author: Ocanath
 */

#include "adc.h"

#define BUFFER_SIZE 10

uint16_t dma_adc_raw[NUM_ADC] = {0};	//initialized, DMA access array
int bemf_adc_map[3] = {ADC_CHAN_BEMF_C,ADC_CHAN_BEMF_B,ADC_CHAN_BEMF_A};

int zero_cross_flag[NUM_ADC] = {0};
int below_thresh[3] = {0};
int below_thresh_prev[3] = {0};

int gl_bemf_adc_record_idx = 0;
int gl_adc_capture_flag = 0;

float iA_filt = 0;
uint8_t shunt_state;


float adc_to_V(int adc)
{
	return ((float)adc) *( 3.3 / ((float)0xFFF) ) ;
}
