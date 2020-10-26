/*
 * nfault_handler.c
 *
 *  Created on: Oct 25, 2020
 *      Author: Ocanath Robotman
 */
#include "nfault_handler.h"
#include "foc_commutation.h"

typedef struct nfault_handle_flags_t
{
	uint8_t fe_flag;	//falling edge
	uint32_t ts;
	uint8_t ts_flag;
}nfault_handle_flags_t;

//uint8_t gl_fault_flag = 0;
nfault_handle_flags_t gl_nfault_flags = {0};

/**
 * @brief  EXTI line detection callback.
 * @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line.
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == NFAULT_Pin)
	{
		gl_nfault_flags.fe_flag = 1;
	}
}

void clear_latched_nfault(void)
{
	if(gl_nfault_flags.fe_flag)
	{
		if(HAL_GPIO_ReadPin(NFAULT_GPIO_Port,NFAULT_Pin) == 1)
		{
			gl_nfault_flags.fe_flag = 0;
			gl_nfault_flags.ts_flag = 0;
		}
		else
		{
			if(gl_nfault_flags.ts_flag == 0)
			{
				gl_nfault_flags.ts = HAL_GetTick();
				gl_nfault_flags.ts_flag = 1;
			}
			else if (HAL_GetTick() - gl_nfault_flags.ts > 10)
			{
				HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 0);
				unwrap( theta_rel_rad(), &theta_enc_prev);
				HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, 1);
				gl_nfault_flags.ts_flag = 0;
				gl_nfault_flags.fe_flag = 0;
			}
		}
	}
}
