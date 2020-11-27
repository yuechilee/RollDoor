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
//Relay control pin
int pin_state;
int st_w_open;
int st_w_stop;
int st_w_close;
int st_w_ir;
int st_w_smk;
int st_w_onekey;
int st_rm_lock;


int Enable = 1;
int Disable = 0;

//Time variable
char TM_OPEN = 0;
char TM_CLOSE = 0;

int Remote_cmd = 0;
int RLY_Delay_ms = 100;
int Tim_Delay_ms;
int Remot_cmd_get = 0;

/* Prescaler declaration */
uint32_t uwPrescalerValue = 0;

static GPIO_InitTypeDef  GPIO_InitStruct;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void EXTI4_15_IRQHandler_Config(void);
static void MotorRelay_out_config(void);
static void StatusRelay_out_config(void);
static void TIMx_Config(uint16_t time_ms, int iwork);

/* Private functions ---------------------------------------------------------*/
void Door_Up(void);
void Door_Stop(void);
void Door_Close(void);


int main(void)
{
	HAL_Init();

/* Configure the system clock to 48 MHz */
	SystemClock_Config();

/* -2- Configure EXTI_Line4_15 (connected to PC.13 pin) in interrupt mode */
  EXTI4_15_IRQHandler_Config();
	
/* -1- Enable each GPIO Clock (to be able to program the configuration registers) */
	GPIOA_CLK_ENABLE();
	GPIOB_CLK_ENABLE();
	GPIOC_CLK_ENABLE();
	GPIOD_CLK_ENABLE();

/* -2- Configure IOs in output push-pull mode to drive external LEDs */
	MotorRelay_out_config();
	StatusRelay_out_config();
	
	//buzz config.
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	GPIO_InitStruct.Pin = Buzz;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	//buzz config...end
	
	//Timer config
	TIMx_Config(50,Enable);

	
	//用途:避免開機的暫態影響GPIO判讀
	Door_Stop();
	HAL_Delay(500); 

	
/* -3- Toggle IOs in an infinite loop */
  while(1)
  {
		if(Remot_cmd_get == 1){
			Remot_cmd_get = 0;
			switch(Remote_cmd){
				case 0:
					Door_Stop();
					//TIMx_Config(50,Disable);
					HAL_TIM_Base_Stop_IT(&TimHandle);
					break;
				
				case 1:
					Door_Up();
					TIMx_Config(600,Enable);
					//HAL_TIM_Base_Start_IT(&TimHandle);
					break;
				
				case 2:
					Door_Close();
					TIMx_Config(600,Enable);
					//HAL_TIM_Base_Start_IT(&TimHandle);
					break;
			}
		}
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

void Door_Up(void){
			HAL_GPIO_WritePin(GPIOB, RLY_DIR, GPIO_PIN_RESET);
			HAL_Delay(RLY_Delay_ms);
			HAL_GPIO_WritePin(GPIOB, RLY_ACT, GPIO_PIN_SET);
			HAL_Delay(RLY_Delay_ms);
			HAL_GPIO_WritePin(GPIOC, MOS_ACT, GPIO_PIN_RESET);	//0:H 1:L
}

void Door_Stop(void){
			HAL_GPIO_WritePin(GPIOC, MOS_ACT, GPIO_PIN_SET);			
			HAL_Delay(RLY_Delay_ms);
			HAL_GPIO_WritePin(GPIOB, RLY_ACT, GPIO_PIN_RESET);
			HAL_Delay(RLY_Delay_ms);
			HAL_GPIO_WritePin(GPIOB, RLY_DIR, GPIO_PIN_RESET);		

}
void Door_Close(void){
			HAL_GPIO_WritePin(GPIOB, RLY_DIR, GPIO_PIN_SET);
			HAL_Delay(RLY_Delay_ms);
			HAL_GPIO_WritePin(GPIOB, RLY_ACT, GPIO_PIN_SET);		
			HAL_Delay(RLY_Delay_ms);
			HAL_GPIO_WritePin(GPIOC, MOS_ACT, GPIO_PIN_RESET);

}

static void MotorRelay_out_config(void){
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	
	//Motor control relays config.
	GPIO_InitStruct.Pin = RLY_ACT | RLY_DIR;
	HAL_GPIO_Init(RL_GPIO_PORT, &GPIO_InitStruct);
	
	//POWER MOSFET config.
	GPIO_InitStruct.Pin = MOS_ACT;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	//Initial condition setting.
	HAL_GPIO_WritePin(GPIOC, MOS_ACT, GPIO_PIN_SET);	//1:OFF, 0:0N
	HAL_GPIO_WritePin(GPIOB, RLY_DIR, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, RLY_ACT, GPIO_PIN_RESET);	//1:ON, 0:0FF
}

static void StatusRelay_out_config(void){
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	GPIO_InitStruct.Pin = RL_ACT | RL_TIME | RL_POS;
	HAL_GPIO_Init(RL_GPIO_PORT, &GPIO_InitStruct);
		
	//Initial condition setting.
	HAL_GPIO_WritePin(RL_GPIO_PORT, RL_ACT, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RL_GPIO_PORT, RL_TIME, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RL_GPIO_PORT, RL_POS, GPIO_PIN_RESET);
}

//EXIT Configures
static void EXTI4_15_IRQHandler_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOC clock */
	WIRE_CTRL_GPIO_CLK_ENABLE();

  /* Configure PC.13 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;//GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = WIRE_CTRL_PIN;
  HAL_GPIO_Init(WIRE_CTRL_PORT, &GPIO_InitStructure);

  /* Enable and set EXTI line 4_15 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

//EXTI line detection callbacks
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
  {
		case W_OPEN:
			Remote_cmd = 1;
			Remot_cmd_get = 1;
			break;
		case W_STOP:
			Remote_cmd = 0;
			Remot_cmd_get = 1;
			break;
		case W_CLOSE:
			Remote_cmd = 2;
			Remot_cmd_get = 1;
			break;
		//default:
			
	}
}

//TIM handle
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_Base_Stop_IT(&TimHandle);
	
	Remote_cmd = 0;
	Remot_cmd_get = 1;	
}

static void TIMx_Config(uint16_t time_s, int iwork)
{	
	uint16_t TMB = time_s;
	/*##-1- Configure the TIM peripheral #######################################*/
	/* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
	uwPrescalerValue = (uint32_t)(SystemCoreClock / 1000) - 1;

	/* Set TIMx instance */
	TimHandle.Instance = TIMx;

	/* Initialize TIMx peripheral as follows:
	+ Period = 10000 - 1
	+ Prescaler = (SystemCoreClock/10000) - 1
	+ ClockDivision = 0
	+ Counter direction = Up
	*/
	if(TMB > 655){
		TMB = 655;
	}
	
	TimHandle.Init.Period            = (time_s*100) - 1;
	TimHandle.Init.Prescaler         = uwPrescalerValue;
	TimHandle.Init.ClockDivision     = 0;
	TimHandle.Init.CounterMode       = TIM_COUNTERMODE_DOWN;
	TimHandle.Init.RepetitionCounter = 0;
	TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	HAL_TIM_Base_Init(&TimHandle);
	
	if(iwork == 1){
		HAL_TIM_Base_Start_IT(&TimHandle);
	}else if(iwork == 0){
		HAL_TIM_Base_Stop_IT(&TimHandle);
	}

}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
