/*
 * adc.c
 *
 *  Created on: Nov 17, 2017
 *      Author: Ocanath
 */

#include "adc.h"

#define BUFFER_SIZE 10

uint16_t dma_adc_raw[NUM_ADC] = {0};	//initialized, DMA access array


float adc_to_V(int adc)
{
	return ((float)adc) *( 3.3 / ((float)0xFFF) ) ;
}
