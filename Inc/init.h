/*
 * init.h
 *
 *  Created on: Mar 28, 2018
 *      Author: Ocanath
 */

#ifndef INIT_H_
#define INIT_H_
#include "main.h"
#include "stm32f3xx_hal.h"
#include "settings.h"

#define STAT_PORT GPIOC
#define STAT_PIN GPIO_PIN_13

#define CAL_PORT GPIOC
#define CAL_PIN GPIO_PIN_14

#define ENABLE_PORT GPIOC
#define ENABLE_PIN GPIO_PIN_15


ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;



void SystemClock_Config(void);
 void MX_GPIO_Init(void);
 void MX_DMA_Init(void);
 void MX_I2C1_Init(void);
 void MX_ADC1_Init(void);
 void MX_TIM1_Init(void);
 void MX_USART2_UART_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


#endif /* INIT_H_ */
