/**
  ******************************************************************************
  * @file    GPIO/GPIO_IOToggle/Src/main.c
  * @author  MCD Application Team
  * @brief   This example describes how to configure and use GPIOs through
  *          the STM32F0xx HAL API.
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

/** @addtogroup STM32F0xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
TIM_HandleTypeDef    TimHandle;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int pin_state;
int st_w_open;
int st_w_stop;
int st_w_close;
int st_w_ir;
int st_w_smk;
int st_w_onekey;

int st_rm_open;
int st_rm_stop;
int st_rm_close;
int st_rm_osc;
int st_rm_lock;

/* Prescaler declaration */
uint32_t uwPrescalerValue = 0;

static GPIO_InitTypeDef  GPIO_InitStruct;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* This sample code shows how to use GPIO HAL API to toggle LED2 IOs
    in an infinite loop. */

  /* STM32F0xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 48 MHz */
  SystemClock_Config();
  
  /* -1- Enable each GPIO Clock (to be able to program the configuration registers) */
  LED2_GPIO_CLK_ENABLE();
	GPIOA_CLK_ENABLE();
	GPIOB_CLK_ENABLE();
	GPIOC_CLK_ENABLE();
	GPIOD_CLK_ENABLE();
	
  /* -2- Configure IOs in output push-pull mode to drive external LEDs */
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStruct.Pin = Buzz;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
	GPIO_InitStruct.Pin = RL_ACT | RL_TIME | RL_POS | RLY_ACT | RLY_DIR;
  HAL_GPIO_Init(RL_GPIO_PORT, &GPIO_InitStruct);
	
  GPIO_InitStruct.Pin = MOS_ACT;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(GPIOC, MOS_ACT, GPIO_PIN_SET);	//1:OFF, 0:0N
	HAL_GPIO_WritePin(GPIOB, RLY_DIR, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, RLY_ACT, GPIO_PIN_RESET);	//1:ON, 0:0FF

  HAL_GPIO_WritePin(RL_GPIO_PORT, RL_ACT, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RL_GPIO_PORT, RL_TIME, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RL_GPIO_PORT, RL_POS, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOA, Buzz, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, Buzz, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, Buzz, GPIO_PIN_RESET);


 /*##-1- Configure the TIM peripheral #######################################*/
  /* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
  uwPrescalerValue = (uint32_t)(SystemCoreClock / 40000) - 1;

  /* Set TIMx instance */
  TimHandle.Instance = TIMx;

  /* Initialize TIMx peripheral as follows:
       + Period = 10000 - 1
       + Prescaler = (SystemCoreClock/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Init.Period            = 10000 - 1;
  TimHandle.Init.Prescaler         = uwPrescalerValue;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_DOWN;
  TimHandle.Init.RepetitionCounter = 0;
  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	
	HAL_TIM_Base_Init(&TimHandle);
	HAL_TIM_Base_Start_IT(&TimHandle);

//避免GPIO接收到供電瞬間的不穩定狀態
	HAL_Delay(1000); 

/* -3- Toggle IOs in an infinite loop */
  while (1)
  {		
		st_w_open  = HAL_GPIO_ReadPin(GPIOC, W_OPEN);
		st_w_stop  = HAL_GPIO_ReadPin(GPIOC, W_STOP);
		st_w_close = HAL_GPIO_ReadPin(GPIOC, W_CLOSE);
		
		st_rm_open  = HAL_GPIO_ReadPin(GPIOD, RM_OPEN);
		st_rm_stop  = HAL_GPIO_ReadPin(GPIOB, RM_STOP);
		st_rm_close = HAL_GPIO_ReadPin(GPIOB, RM_CLOSE);
		
		
//線控器
		if((st_w_open == GPIO_PIN_RESET)	||
		   (st_rm_open == GPIO_PIN_SET)){
			HAL_GPIO_WritePin(GPIOB, RLY_DIR, GPIO_PIN_RESET);
			HAL_Delay(250);
			HAL_GPIO_WritePin(GPIOB, RLY_ACT, GPIO_PIN_SET);
			HAL_Delay(250);
			HAL_GPIO_WritePin(GPIOC, MOS_ACT, GPIO_PIN_RESET);	//0:H 1:L

			//HAL_GPIO_WritePin(RL_GPIO_PORT, RL_ACT, GPIO_PIN_RESET);
			//HAL_GPIO_WritePin(RL_GPIO_PORT, RL_TIME, GPIO_PIN_SET);
			//HAL_GPIO_WritePin(RL_GPIO_PORT, RL_POS, GPIO_PIN_RESET);
			
		}
		
		if((st_w_stop == GPIO_PIN_RESET)	||
		   (st_rm_stop == GPIO_PIN_SET)){
			HAL_GPIO_WritePin(GPIOC, MOS_ACT, GPIO_PIN_SET);			
			HAL_Delay(250);
			HAL_GPIO_WritePin(GPIOB, RLY_ACT, GPIO_PIN_RESET);
			HAL_Delay(250);
			HAL_GPIO_WritePin(GPIOB, RLY_DIR, GPIO_PIN_RESET);		
			
			
			//HAL_GPIO_WritePin(RL_GPIO_PORT, RL_ACT, GPIO_PIN_SET);
			//HAL_GPIO_WritePin(RL_GPIO_PORT, RL_TIME, GPIO_PIN_RESET);
			//HAL_GPIO_WritePin(RL_GPIO_PORT, RL_POS, GPIO_PIN_RESET);
		}
		
		if((st_w_close == GPIO_PIN_RESET)	||
		   (st_rm_close == GPIO_PIN_SET)){
			HAL_GPIO_WritePin(GPIOB, RLY_DIR, GPIO_PIN_SET);
			HAL_Delay(250);
			HAL_GPIO_WritePin(GPIOB, RLY_ACT, GPIO_PIN_SET);		
			HAL_Delay(250);
			HAL_GPIO_WritePin(GPIOC, MOS_ACT, GPIO_PIN_RESET);
			
			//HAL_GPIO_WritePin(RL_GPIO_PORT, RL_ACT, GPIO_PIN_RESET);
			//HAL_GPIO_WritePin(RL_GPIO_PORT, RL_TIME, GPIO_PIN_RESET);
			//HAL_GPIO_WritePin(RL_GPIO_PORT, RL_POS, GPIO_PIN_SET);
		}

//線控器
	/* 	if(st_w_open == GPIO_PIN_SET){
			HAL_GPIO_WritePin(GPIOC, MOS_ACT, GPIO_PIN_RESET);	//0:H 1:L
			HAL_GPIO_WritePin(GPIOB, RLY_DIR, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, RLY_ACT, GPIO_PIN_SET);
			
			HAL_GPIO_WritePin(RL_GPIO_PORT, RL_ACT, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RL_GPIO_PORT, RL_TIME, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RL_GPIO_PORT, RL_POS, GPIO_PIN_RESET);
		}
		
		if(st_w_stop == GPIO_PIN_SET){
			HAL_GPIO_WritePin(GPIOC, MOS_ACT, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, RLY_DIR, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, RLY_ACT, GPIO_PIN_RESET);		
			
			HAL_GPIO_WritePin(RL_GPIO_PORT, RL_ACT, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RL_GPIO_PORT, RL_TIME, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RL_GPIO_PORT, RL_POS, GPIO_PIN_RESET);
		}
		
		if(st_w_close == GPIO_PIN_SET){
			HAL_GPIO_WritePin(GPIOC, MOS_ACT, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, RLY_DIR, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, RLY_ACT, GPIO_PIN_SET);		
			
			HAL_GPIO_WritePin(RL_GPIO_PORT, RL_ACT, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RL_GPIO_PORT, RL_TIME, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RL_GPIO_PORT, RL_POS, GPIO_PIN_SET);
		} */

//線控器
/*		
		if(st_w_open == GPIO_PIN_SET){
			HAL_GPIO_WritePin(RL_GPIO_PORT, RL_ACT, GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(RL_GPIO_PORT, RL_ACT, GPIO_PIN_RESET);
		}
		
		if(st_w_stop == GPIO_PIN_SET){
			HAL_GPIO_WritePin(RL_GPIO_PORT, RL_TIME, GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(RL_GPIO_PORT, RL_TIME, GPIO_PIN_RESET);
		}
		
		if(st_w_close == GPIO_PIN_SET){
			HAL_GPIO_WritePin(RL_GPIO_PORT, RL_POS, GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(RL_GPIO_PORT, RL_POS, GPIO_PIN_RESET);
		}
*/
//發射器
/*
		if(st_rm_stop == GPIO_PIN_SET){
			HAL_GPIO_WritePin(RL_GPIO_PORT, RL_ACT, GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(RL_GPIO_PORT, RL_ACT, GPIO_PIN_RESET);
		}
		
		if((st_w_stop == GPIO_PIN_SET)||
			 (st_rm_stop == GPIO_PIN_SET)){
			HAL_GPIO_WritePin(RL_GPIO_PORT, RL_TIME, GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(RL_GPIO_PORT, RL_TIME, GPIO_PIN_RESET);
		}
		
		if((st_w_close == GPIO_PIN_SET)||
		   (st_rm_close == GPIO_PIN_SET)){
			HAL_GPIO_WritePin(RL_GPIO_PORT, RL_POS, GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(RL_GPIO_PORT, RL_POS, GPIO_PIN_RESET);
		}
*/	
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI/2)
  *            SYSCLK(Hz)                     = 48000000
  *            HCLK(Hz)                       = 48000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 8000000
  *            PREDIV                         = 1
  *            PLLMUL                         = 12
  *            Flash Latency(WS)              = 1
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* No HSE Oscillator on Nucleo, Activate PLL with HSI/2 as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1)!= HAL_OK)
  {
    Error_Handler();
  }
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
