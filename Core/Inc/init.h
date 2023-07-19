/*
 * init.h
 *
 *  Created on: Mar 28, 2018
 *      Author: Ocanath
 */

#ifndef INIT_H_
#define INIT_H_
#include "main.h"
#include "stm32g4xx_hal.h"
#include "settings.h"

#define STAT_PORT GPIOC
#define STAT_PIN GPIO_PIN_13

#define CAL_PORT GPIOC
#define CAL_PIN GPIO_PIN_14

#define ENABLE_PORT GPIOC
#define ENABLE_PIN GPIO_PIN_15


extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern FDCAN_HandleTypeDef hfdcan1;

extern SPI_HandleTypeDef hspi1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_ADC1_Init(void);
void MX_FDCAN1_Init(void);
void MX_TIM1_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_TIM2_Init(void);
void MX_SPI1_Init(void);


#endif /* INIT_H_ */
