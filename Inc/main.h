
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx_nucleo.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
	/* Definition for TIMx clock resources */
#define TIMx                           TIM16
#define TIMx_CLK_ENABLE()              __HAL_RCC_TIM16_CLK_ENABLE()

	/* Definition for TIMx's NVIC */
#define TIMx_IRQn                      TIM16_IRQn
#define TIMx_IRQHandler                TIM16_IRQHandler


#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
