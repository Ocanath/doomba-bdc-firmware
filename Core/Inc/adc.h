/*
 * adc.h
 *
 *  Created on: Nov 17, 2017
 *      Author: Ocanath
 */

#ifndef ADC_H_
#define ADC_H_
#include "init.h"

#define NUM_CURRENT_SENSORS 	3
#define NUM_ADC 				4

#define RISING 1
#define FALLING 0

#define ADC_CHAN_BEMF_C 3
#define ADC_CHAN_BEMF_B 4
#define ADC_CHAN_BEMF_A 5

#define ADC_CHAN_CURRENT_A 0
#define ADC_CHAN_CURRENT_B 1
#define ADC_CHAN_CURRENT_C 2

extern uint16_t dma_adc_raw[NUM_ADC];


float adc_to_V(int adc);

#endif /* ADC_H_ */
