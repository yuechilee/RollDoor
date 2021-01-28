/**
  ******************************************************************************
  * @file    ADC/ADC_DMA_Transfer/Inc/main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
//#include "stm32f0xx_nucleo.h"
#include "stdio.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor ADCx instance used and associated
   resources */
	 
//==========================TIMx==================================================//
/* Definition for TIMx clock resources */
#define TIMx                           TIM16
#define TIMx_CLK_ENABLE()              __HAL_RCC_TIM16_CLK_ENABLE()

/* Definition for TIMx's NVIC */
#define TIMx_IRQn                      TIM16_IRQn
#define TIMx_IRQHandler                TIM16_IRQHandler
//==========================TIMx end===============================================//

//==========================ADCx==================================================//
/* Definition for ADCx clock resources */
#define ADCx                            ADC1
#define ADCx_CLK_ENABLE()               __HAL_RCC_ADC1_CLK_ENABLE()
#define ADCx_CHANNEL_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOC_CLK_ENABLE()

#define DMAx_CHANNELx_CLK_ENABLE()      __HAL_RCC_DMA1_CLK_ENABLE()

#define ADCx_FORCE_RESET()              __HAL_RCC_ADC1_FORCE_RESET()
#define ADCx_RELEASE_RESET()            __HAL_RCC_ADC1_RELEASE_RESET()

/* Definition for ADCx Channel Pin */
#define ADCx_CHANNEL_PIN_CLK_ENABLE()   __HAL_RCC_GPIOC_CLK_ENABLE()
#define ADCx_CHANNEL_PIN                GPIO_PIN_0
#define ADCx_CHANNEL_GPIO_PORT          GPIOA

/* Definition for ADCx's Channel */
#define ADCx_CHANNEL                    ADC_CHANNEL_3
//==========================ADCx end===============================================//

//==========================USARTx==================================================//
/* Definition for USARTx clock resources */
#define USARTx                           USART1
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_9
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_AF                     GPIO_AF1_USART1
#define USARTx_RX_PIN                    GPIO_PIN_10
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_AF                     GPIO_AF1_USART1

/* Size of Trasmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE
  
/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */
//========================USARTx end=================================================//

//==========================EXTI ==================================================//
//EXTI param
#define EXTI_CTRL_PIN														W_OPEN | W_STOP | W_CLOSE		//[YC001]
#define EXTI_CTRL_PORT				                  PORT_Control_1
#define EXTI_CTRL_PORT_2			                  PORT_Control_2
#define EXTI_CTRL_GPIO_CLK_ENABLE()  	          GPIOC_CLK_ENABLE()   
#define EXTI_CTRL_GPIO_CLK_DISABLE()            GPIOC_CLK_DISABLE()  
#define EXTI_CTRL_LOCK_CLK_ENABLE()  	          GPIOF_CLK_ENABLE()   
#define EXTI_CTRL_LOCK_CLK_DISABLE()            GPIOF_CLK_DISABLE()  
//#define WIRE_CTRL_LINE                          EXTI_CTRL_PIN
//#define EXTI_CTRL_EXTI_IRQn                     EXTI4_15_IRQn
//==========================EXTI end===============================================//

//==========================I2Cx==========================//
#define I2Cx                            I2C1
#define RCC_PERIPHCLK_I2Cx              RCC_PERIPHCLK_I2C1
#define RCC_I2CxCLKSOURCE_SYSCLK        RCC_I2C1CLKSOURCE_SYSCLK
#define I2Cx_CLK_ENABLE()               __HAL_RCC_I2C1_CLK_ENABLE()
#define I2Cx_SDA_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE() 

#define I2Cx_FORCE_RESET()              __HAL_RCC_I2C1_FORCE_RESET()
#define I2Cx_RELEASE_RESET()            __HAL_RCC_I2C1_RELEASE_RESET()

/* Definition for I2Cx Pins */
#define I2Cx_SCL_PIN                    GPIO_PIN_6
#define I2Cx_SCL_GPIO_PORT              GPIOB
#define I2Cx_SDA_PIN                    GPIO_PIN_7
#define I2Cx_SDA_GPIO_PORT              GPIOB
#define I2Cx_SCL_SDA_AF                 GPIO_AF1_I2C1

/* Size of Transmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */
//==========================I2Cx end==========================//

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

//========================================//
/*****************PIN Def******************/
//========================================//
/******************Port A******************/
//#define	xxx   	           GPIO_PIN_0
//#define	xxx   	           GPIO_PIN_1
//#define	xxx   	           GPIO_PIN_2
//#define	xxx   	           GPIO_PIN_3
//#define	xxx   	           GPIO_PIN_4
//#define	xxx   	           GPIO_PIN_5
#define	MOS_ACT   	         GPIO_PIN_6
//#define	xxx   	           GPIO_PIN_7
#define	Buzz   	             GPIO_PIN_8
//#define	xxx   	           GPIO_PIN_9
//#define	xxx   	           GPIO_PIN_10
//#define	xxx   	           GPIO_PIN_11
//#define	xxx   	           GPIO_PIN_12
//#define	xxx   	           GPIO_PIN_13
//#define	xxx   	           GPIO_PIN_14
//#define	xxx   	           GPIO_PIN_15

#define PORT_Motor_MOS       GPIOA
#define Motor_MOS_CLK_ENABLE()    __HAL_RCC_GPIOA_CLK_ENABLE()  
#define Motor_MOS_CLK_DISABLE()   __HAL_RCC_GPIOA_CLK_DISABLE()

#define PORT_Buzzer          GPIOA
#define Buzzer_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()  
#define Buzzer_CLK_DISABLE()      __HAL_RCC_GPIOA_CLK_DISABLE()

/******************Port B******************/
#define	RLY_DIR   	         GPIO_PIN_0
#define	RLY_ACT   	         GPIO_PIN_1
#define	RL_POS   	           GPIO_PIN_2
#define	W_ONEKEY   	         GPIO_PIN_3     //Renote = OSC
//#define	xxx   	           GPIO_PIN_4
//#define	xxx   	           GPIO_PIN_5
//#define	xxx   	           GPIO_PIN_6
//#define	xxx   	           GPIO_PIN_7
//#define	xxx   	           GPIO_PIN_8
//#define	xxx   	           GPIO_PIN_9
#define	RL_ACT   	           GPIO_PIN_10
#define	RL_TIME   	         GPIO_PIN_11
//#define	xxx   	           GPIO_PIN_12
#define	W_IR   	             GPIO_PIN_13
//#define	xxx   	           GPIO_PIN_14
#define	W_SMK   	           GPIO_PIN_15

#define PORT_ONEKEY          GPIOB
#define ONEKEY_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()  
#define ONEKEY_CLK_DISABLE()      __HAL_RCC_GPIOB_CLK_DISABLE()

#define PORT_IR     	       GPIOB
#define IR_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()  
#define IR_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()

#define PORT_SMK     	       GPIOB
#define SMK_CLK_ENABLE()          __HAL_RCC_GPIOB_CLK_ENABLE()  
#define SMK_CLK_DISABLE()         __HAL_RCC_GPIOB_CLK_DISABLE()

#define PORT_Control_2       GPIOB
#define Control_2_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()  
#define Control_2_CLK_DISABLE()   __HAL_RCC_GPIOB_CLK_DISABLE()

#define PORT_Motor_Out       GPIOB
#define Motor_Out_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()  
#define Motor_Out_CLK_DISABLE()   __HAL_RCC_GPIOB_CLK_DISABLE()

#define PORT_Status_Out      GPIOB
#define Status_Out_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()  
#define Status_Out_CLK_DISABLE()  __HAL_RCC_GPIOB_CLK_DISABLE()

/******************Port C******************/
//#define	xxx   	           GPIO_PIN_0
#define	TEST_PIN             GPIO_PIN_1
//#define	xxx   	           GPIO_PIN_2
//#define	xxx   	           GPIO_PIN_3
//#define	xxx   	           GPIO_PIN_4
//#define	xxx   	           GPIO_PIN_5
//#define	xxx   	           GPIO_PIN_6
#define	EEPROM_SEL   	       GPIO_PIN_7
//#define	xxx   	           GPIO_PIN_8
//#define	xxx   	           GPIO_PIN_9
#define	W_OPEN   	           GPIO_PIN_10
#define	W_STOP   	           GPIO_PIN_11
#define	W_CLOSE  	           GPIO_PIN_12
//#define	xxx   	           GPIO_PIN_13
//#define	xxx   	           GPIO_PIN_14
//#define	xxx   	           GPIO_PIN_15

#define PORT_STOP            GPIOC
#define STOP_CLK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()  
#define STOP_CLK_DISABLE()          __HAL_RCC_GPIOC_CLK_DISABLE()

#define PORT_OPEN            GPIOC
#define OPEN_CLK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()  
#define OPEN_CLK_DISABLE()          __HAL_RCC_GPIOC_CLK_DISABLE()

#define PORT_CLOSE           GPIOC
#define CLOSE_CLK_ENABLE()          __HAL_RCC_GPIOC_CLK_ENABLE()  
#define CLOSE_CLK_DISABLE()         __HAL_RCC_GPIOC_CLK_DISABLE()

#define PORT_Control_1       GPIOC
#define Control_1_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()  
#define Control_1_CLK_DISABLE()     __HAL_RCC_GPIOC_CLK_DISABLE()

#define PORT_EE_SEL          GPIOC
#define EE_SEL_CLK_ENABLE()         __HAL_RCC_GPIOC_CLK_ENABLE()  
#define EE_SEL_CLK_DISABLE()        __HAL_RCC_GPIOC_CLK_DISABLE()

#define PORT_TEST            GPIOC
#define TEST_CLK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()  
#define TEST_CLK_DISABLE()          __HAL_RCC_GPIOC_CLK_DISABLE()

/******************Port D******************/
//#define	xxx   	           GPIO_PIN_2

//#define GPIOD_CLK_ENABLE()           __HAL_RCC_GPIOD_CLK_ENABLE()  
//#define GPIOD_CLK_DISABLE()          __HAL_RCC_GPIOD_CLK_DISABLE()

/******************Port F******************/
//#define	xxx   	           GPIO_PIN_0
//#define	xxx   	           GPIO_PIN_1
//#define	xxx   	           GPIO_PIN_4
//#define	xxx                GPIO_PIN_5
//#define	xxx   	           GPIO_PIN_6
#define	RM_LOCK  	           GPIO_PIN_7

#define PORT_LOCK            GPIOF
#define LOCK_CLK_ENABLE()           __HAL_RCC_GPIOF_CLK_ENABLE()  
#define LOCK_CLK_DISABLE()          __HAL_RCC_GPIOF_CLK_DISABLE()

//========================================//
/*************** PIN Def End***************/
//========================================//

//GPIO CLK EN/DISABLE
#define GPIOA_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()  
#define GPIOA_CLK_DISABLE()          __HAL_RCC_GPIOA_CLK_DISABLE()
#define GPIOB_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()  
#define GPIOB_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()
#define GPIOC_CLK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()  
#define GPIOC_CLK_DISABLE()          __HAL_RCC_GPIOC_CLK_DISABLE()
#define GPIOD_CLK_ENABLE()           __HAL_RCC_GPIOD_CLK_ENABLE()  
#define GPIOD_CLK_DISABLE()          __HAL_RCC_GPIOD_CLK_DISABLE()
#define GPIOE_CLK_ENABLE()           __HAL_RCC_GPIOD_CLK_ENABLE()  
#define GPIOE_CLK_DISABLE()          __HAL_RCC_GPIOD_CLK_DISABLE()
#define GPIOF_CLK_ENABLE()           __HAL_RCC_GPIOD_CLK_ENABLE()  
#define GPIOF_CLK_DISABLE()          __HAL_RCC_GPIOD_CLK_DISABLE()


//==========End==========//
#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
