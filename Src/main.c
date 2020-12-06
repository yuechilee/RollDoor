/**

  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define Enable 1
#define Disable	0

TIM_HandleTypeDef    TimHandle;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
	//Boolean
bool ST_BTN;												//(Remote) controller trigger(0:standby, 1:cmd trigger
bool Open_IT;												//Interrupt in open
bool Close_IT;											//Interrupt in close 2-1
bool Close_IT2;											//Interrupt in close 2-2
bool Close_Segment_Flg = TRUE;			//2-seg close requie?

	//8-bits
uint8_t ACT_Door = 0;								//Controller's cmd (0:Stop /1:Open /2:Close)
uint8_t ST_Door = 0;								//Operating status (0:Stop or standby /1:Opening /2:Closing)
uint8_t ST_Close;										//Recode the 2-seg close cmd

	//16-bits
uint16_t TM_MAX = 150;							//Operate maximum time.(TM_MAX/10 = xx.x sec.)
uint16_t TM_OPEN = 0;								//Time: Door open
uint16_t TM_CLOSE = 0;							//Time: Door close
uint16_t CloseTM1 = 100;							//TIme: Door close segment 2-1
uint16_t CloseTM2 = 0;							//TIme: Door close segment 2-2; Set in Init()
uint16_t OpenTM_Remain = 0;					//Remain time while interrupt in open
uint16_t CloseTM_Remain = 0;				//Remain time while interrupt in close


//===================================//

uint32_t RLY_Delay_ms = 10;
uint8_t Remote_flg = 0;
uint8_t Door_st = 0;

/* Prescaler declaration */
uint32_t uwPrescalerValue = 0;

static GPIO_InitTypeDef  GPIO_InitStruct;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void EXTI4_15_IRQHandler_Config(void);
static void MotorRelay_out_config(void);
static void StatusRelay_out_config(void);
static void TIMx_Config(void);

/* Private functions ---------------------------------------------------------*/
void Door_Open(void);
void Door_Stop(void);
void Door_Close(void);		
void Door_manage(void);		//Operating time calculate
void PWR_CTRL(void);		//Power ON to motor

void Delay_ms(int32_t nms);

static void Error_Handler(void);
static uint16_t	TIMDEC(uint16_t TIMB);

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

	//GPIO_InitStruct.Pin = Buzz;
	//HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	//buzz config...end
	TIMx_Config();


	TM_OPEN = 0;
	TM_CLOSE = 0;
	CloseTM2 = TM_MAX - CloseTM1;		//section_time_2 of close operation
	ST_Close = 1;
	
	//用途:避免開機的暫態影響GPIO判讀
	Door_Stop();	
	Delay_ms(500);

	//EXTI enable
	HAL_TIM_Base_Start_IT(&TimHandle);
/* -3- Toggle IOs in an infinite loop */
  while(1)
  {	
		Door_manage();
		PWR_CTRL();
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

void PWR_CTRL(void){
	if(TM_CLOSE > 0 && TM_OPEN > 0){
		//Empty
		//Avoid both the timer work at the same time.
	}else{	
		if(Close_Segment_Flg == FALSE){		//1-segment mode
			if(TM_OPEN > 0){
				Door_Open();
			}else if(TM_CLOSE > 0){				
				Door_Close();
			}else{
				Door_Stop();
			}
		}else{
			if(TM_OPEN > 0){
				Door_Open();
			}else if(TM_CLOSE > 0){				
				Door_Close();
			}else{
				if(TM_OPEN == 0 && TM_CLOSE == 0){
					Door_Stop();
					if(ST_Door == 1){
						ST_Door = 0;
						Open_IT = FALSE;
						Close_IT = FALSE;
						Close_IT2 = FALSE;
					}else if(ST_Door == 2){
						ST_Door = 0;
						if(Open_IT == TRUE){
							ST_Close = 1;
						}else if(ST_Close == 1){
							ST_Close = 2;
						}else if(ST_Close == 2){
							ST_Close = 1;
						}
						Open_IT = FALSE;
						Close_IT = FALSE;
						Close_IT2 = FALSE;

					}
				}
			}
		}
	}
}

void Door_manage(void){
	if(ST_BTN == TRUE){
		ST_BTN = FALSE;
		if(Close_Segment_Flg == FALSE){				//無兩段式關門
			switch(ACT_Door){
				case 0:	//Stop
					TM_OPEN = 0;
					TM_CLOSE = 0;
					break;
				
				case 1:	//Open
					TM_OPEN = TM_MAX;
					TM_CLOSE = 0;
					break;
				
				case 2:	//Close
					TM_OPEN = 0;
					TM_CLOSE = TM_MAX;
					break;
				
				default:
					TM_OPEN = 0;
					TM_CLOSE = 0;
			}
		}else if(Close_Segment_Flg == TRUE){
			switch(ACT_Door){
				case 0:	//Stop
					if(ST_Door == 1){
						OpenTM_Remain = TM_OPEN;
						TM_OPEN = 0;
						if(OpenTM_Remain > 0){
							Open_IT = TRUE;
						}else if(OpenTM_Remain == 0){
							Open_IT = FALSE;
							Close_IT = FALSE;
							Close_IT2 = FALSE;
						}

					}else if(ST_Door == 2){
						if(ST_Close == 1){
							CloseTM_Remain = TM_CLOSE;
							if(CloseTM_Remain > 0){
								Close_IT = TRUE;
							}else if(CloseTM_Remain == 0){
								Close_IT = FALSE;
								ST_Close = 2;		//等待第二段close指令
							}
						}else if(ST_Close == 2){
							CloseTM_Remain = TM_CLOSE;
							if(CloseTM_Remain > 0){
								Close_IT2 = TRUE;
							}else if(CloseTM_Remain == 0){
								Close_IT2 = FALSE;
								ST_Close = 1;
							}
						}
						TM_CLOSE = 0;
					}
					ST_Door = 0;
					break;
				
				case 1:	//Open
					ST_Door = 1;
					Open_IT = FALSE;
					Close_IT = FALSE;
					Close_IT2 = FALSE;
					TM_OPEN = TM_MAX;
					TM_CLOSE = 0;
					break;
					
				case 2:	//Close
					ST_Door = 2;
					TM_OPEN =0;
					if(Open_IT == TRUE){
						TM_CLOSE = TM_MAX;
					}else if(ST_Close == 1){
						if(Close_IT == FALSE){
							TM_CLOSE = CloseTM1;
						}else{
							TM_CLOSE = CloseTM_Remain;
						}
					}else if(ST_Close == 2){
						if(Close_IT2 == FALSE){
							TM_CLOSE = CloseTM2;
						}else{
							TM_CLOSE = CloseTM_Remain;
						}
					}else{
						//Empty
					}
					break;
				
				default:
					//Empty
					break;
			}
		}
	}
	
	if(TM_OPEN == 0 && TM_CLOSE == 0){
		ACT_Door = 0;
	}
}	
/*//20201206 disable the older management design
void Door_manage(void){
		if(Remote_flg == 1){
			Remote_flg = 0;
			if(TM_OPEN > 0){
						Door_Open();
			}else if(TM_CLOSE > 0){				
						Door_Close();
			}else{	
						Door_Stop();
			}
		}else{
			if((Door_st > 0) 	&& 
				 (TM_OPEN == 0 && TM_CLOSE == 0)){
				Door_st = 0;
				Door_Stop();
			}
		}
}
*/


void Door_Open(void){
			HAL_GPIO_WritePin(GPIOB, RLY_DIR, GPIO_PIN_RESET);
			Delay_ms(RLY_Delay_ms);
			HAL_GPIO_WritePin(GPIOB, RLY_ACT, GPIO_PIN_SET);
			Delay_ms(RLY_Delay_ms);
			HAL_GPIO_WritePin(GPIOC, MOS_ACT, GPIO_PIN_RESET);	//0:H 1:L
}

void Door_Stop(void){
			HAL_GPIO_WritePin(GPIOC, MOS_ACT, GPIO_PIN_SET);			
			Delay_ms(RLY_Delay_ms);
			HAL_GPIO_WritePin(GPIOB, RLY_ACT, GPIO_PIN_RESET);
			Delay_ms(RLY_Delay_ms);
			HAL_GPIO_WritePin(GPIOB, RLY_DIR, GPIO_PIN_RESET);		

}
void Door_Close(void){
			HAL_GPIO_WritePin(GPIOB, RLY_DIR, GPIO_PIN_SET);
			Delay_ms(RLY_Delay_ms);
			HAL_GPIO_WritePin(GPIOB, RLY_ACT, GPIO_PIN_SET);		
			Delay_ms(RLY_Delay_ms);
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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	switch(GPIO_Pin)
  {
		case W_STOP:
			//TM_OPEN = 0;
			//TM_CLOSE = 0;
			ST_BTN = TRUE;
			ACT_Door = 0;
			break;
		
		case W_OPEN:
			if(ACT_Door == 0){
				//TM_OPEN = 10;
				//TM_CLOSE = 0;
				ST_BTN = TRUE;
				ACT_Door = 1;
			}
			break;

		case W_CLOSE:
			if(ACT_Door == 0){
				//TM_OPEN = 0;
				//TM_CLOSE = 10;
				ST_BTN = TRUE;
				ACT_Door = 2;
			}
			break;
		
		default:
				break;
	}

}

/*
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
  {
		case W_STOP:
			TM_OPEN = 0;
			TM_CLOSE = 0;
			Remote_flg = 1;
			Door_st = 0;
			break;
		
		case W_OPEN:
			if(Door_st != 1 && Door_st == 0){
				TM_OPEN = 10;
				TM_CLOSE = 0;
				Remote_flg = 1;
				Door_st = 1;
			}
			break;

		case W_CLOSE:
			if(Door_st != 2 && Door_st == 0){
				TM_OPEN = 0;
				TM_CLOSE = 10;
				Remote_flg = 1;
				Door_st = 2;
			}
			break;
		
		default:
				break;
	}
}
*/

//TIM handle
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//HAL_TIM_Base_Stop_IT(&TimHandle);
	TM_OPEN = TIMDEC(TM_OPEN);
	TM_CLOSE = TIMDEC(TM_CLOSE);
}

//1s timer
static void TIMx_Config(void)
{	
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

	
	TimHandle.Init.Period            = (1*100) - 1;
	TimHandle.Init.Prescaler         = uwPrescalerValue;
	TimHandle.Init.ClockDivision     = 0;
	TimHandle.Init.CounterMode       = TIM_COUNTERMODE_DOWN;
	TimHandle.Init.RepetitionCounter = 0;
	TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	HAL_TIM_Base_Init(&TimHandle);

}

static uint16_t	TIMDEC(uint16_t TIMB){
	uint8_t TIM_Buf = TIMB;
	if(TIMB == 0) return TIMB;
	
	TIM_Buf--;
	
	return TIM_Buf;
}

void Delay_ms(int32_t nms) 
 {
  int32_t temp; 
  SysTick->LOAD = 8000*nms; 
  SysTick->VAL=0X00;//清空計數器 
  SysTick->CTRL=0X01;//使能，減到零是無動作，採用外部時鐘源 
  do 
  { 
       temp=SysTick->CTRL;//讀取當前倒計數值 
  }
     while((temp&0x01)&&(!(temp&(1<<16))));//等待時間到達 
     
     SysTick->CTRL=0x00; //關閉計數器 
     SysTick->VAL =0X00; //清空計數器 
 } 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
