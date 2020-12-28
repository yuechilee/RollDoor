/**
  ******************************************************************************
  * @file    ADC/ADC_DMA_Transfer/Src/stm32f0xx_it.c
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"

/** @addtogroup STM32F0xx_HAL_Examples
  * @{
  */

/** @addtogroup ADC_DMA_Transfer
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern ADC_HandleTypeDef             AdcHandle;
extern TIM_HandleTypeDef    TimHandle;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}


void DMA1_Channel1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(AdcHandle.DMA_Handle);
}

void TIMx_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimHandle);
}

void EXTI4_15_IRQHandler(void)
{
  //HAL_GPIO_EXTI_IRQHandler(EXTI_CTRL_PIN);
	  /* EXTI line interrupt detected */
  if(__HAL_GPIO_EXTI_GET_IT(W_OPEN) != 0x00u){ 
    __HAL_GPIO_EXTI_CLEAR_IT(W_OPEN);
    HAL_GPIO_EXTI_Callback(W_OPEN);
		
  }else if(__HAL_GPIO_EXTI_GET_IT(W_STOP) != 0x00u){ 
    __HAL_GPIO_EXTI_CLEAR_IT(W_STOP);
    HAL_GPIO_EXTI_Callback(W_STOP);
		
  }else if(__HAL_GPIO_EXTI_GET_IT(W_CLOSE) != 0x00u){ 
    __HAL_GPIO_EXTI_CLEAR_IT(W_CLOSE);
    HAL_GPIO_EXTI_Callback(W_CLOSE);
  }

}


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
