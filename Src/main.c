/*******************************************************************************
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  *******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define Enable 1
#define Disable	0
#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  32)   /* Definition of ADCx conversions data table size */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Handle Declaration ---------------------------------------------------------------*/
ADC_HandleTypeDef         AdcHandle;	// ADC handle declaration
ADC_ChannelConfTypeDef    sConfig;		// ADC channel configuration structure declaration
TIM_HandleTypeDef         TimHandle;	
UART_HandleTypeDef        UartHandle;	// UART handler declaration
static GPIO_InitTypeDef   GPIO_InitStruct;

/* Private functions ---------------------------------------------------------*/
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


/* Private macro -------------------------------------------------------------*/
//��q���}�����:
//bool Close_Segment_Flg = FALSE; 		//�L:FALSE	
bool Close_Segment_Flg = TRUE;        //��:TRUE

//�`������(���ɴ���)
bool Cycle_test = TRUE;                //��:TRUE
//bool Cycle_test = FALSE;             //�L:FALSE

//�۰������\��
//bool Flag_AutoClose = TRUE;				//��:TRUE
bool Flag_AutoClose = FALSE;			//�L:FALSEE

//�T�ʥ\��
//bool Flag_Func_JOG = TRUE;			//��:TRUE
bool Flag_Func_JOG = FALSE;				//�L:FALSEE

//���F�B���V
//bool Flag_Motor_Direction = TRUE;		//�_��:TRUE
bool Flag_Motor_Direction = FALSE;		//�n��:FALSE

	
//�}�����̱`�B��ɶ�
uint32_t TM_MAX = 600;                  //�}�����̪��B��ɶ� TM_MAX * 100ms

//�۰���������ɶ�
uint32_t Time_Auto_Close = 100;			// n * 0.1sec.

//�ө��B��ɶ�
uint32_t Time_Light = 50;				// n * 0.1sec

//�ݾ��q��
float V_Stby = 0.3;						//�ݾ��q��(��0���즸�Ұʰ���),��ĳ��0.3~0.5

//�T�ʧP�w����
uint16_t Conti_times = 50;				//�T�ʧP�w����, �j��:�T��, �p��:�@��

//����3�v��
float Slope_Open = 1.5;					//�����v��(�i�p��):�}��(�V�p�V�F��),��ĳ>1
float Slope_Close = 1.5;				//�����v��(�i�p��):����(�V�p�V�F��),��ĳ>1


/* Private variables ---------------------------------------------------------*/
	//Boolean
bool ST_BTN;                            //(Remote) controller trigger(0:standby, 1:cmd trigger
bool Open_IT;                           //Interrupt in open
bool Close_IT;                          //Interrupt in close 2-1
bool Close_IT2;                         //Interrupt in close 2-2
bool Op_Flag = FALSE;
bool Anti_flg2 = TRUE;
bool AClose_Flg = FALSE;				// �۰������ʧ@�X��
bool Wait_flg;
bool Lock_CTRL = FALSE;					//��q�ʧ@�X��
bool Flag_JOG = FALSE;					//�T�ʰʧ@�X��
	
	//8-bits
uint8_t ACT_Door = 0;                   //Controller's cmd (0:Stop /1:Open /2:Close)
uint8_t ST_Door = 0;                    //Operating status (0:Stop or standby /1:Opening /2:Closing)
uint8_t ST_Door_buf;
uint8_t ST_Close;                       //Recode the 2-seg close cmd
uint8_t	ST_Anti;
uint8_t Vop_Cnt;
uint8_t OverSlope_Times = 0;
uint8_t OS_Occur_Times = 2;
uint8_t Cycle_jumper;
uint8_t ST_Press;
	//16-bits

uint16_t Vadc_buf;
uint16_t Calc_Times;
uint16_t Voc_amt;
uint16_t TM_Printf = 10;
uint16_t iWeight = 1000;
static uint16_t aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];	  //Variable containing ADC conversions data

	//32-bits
uint32_t RLY_Delay_ms = 20;			   //Relay_Delay_time(*1ms)
uint32_t uwPrescalerValue = 0;         // Prescaler declaration

uint32_t TM_OPEN = 0;                   //Time: Door open
uint32_t TM_CLOSE = 0;                  //Time: Door close
uint32_t TM_AntiDly;
uint32_t Time_AntiDly = 20;
uint32_t TM_AntiDly2;
uint32_t TM_AntiDly4;
uint32_t Time_AntiDly4 = 1;
uint32_t TM_EndDetec;
uint32_t TM_DoorOperateDly = 20;        //��찻������ɶ�(*100ms)
uint32_t OpenTM_Remain = 0;             //��q���}���Ѿl�ɶ�
uint32_t CloseTM_Remain = 0;            //��q�������Ѿl�ɶ�
uint32_t TM_DLY;						//cycle-test���ݬ��(*100ms)
uint32_t TM_Light_Off = 0;
uint32_t TM_Auto_Close = 0;
uint32_t CloseTM1,CloseTM2;


uint32_t Cycle_times_up = 0;
uint32_t Cycle_times_down = 0;

uint32_t Ver_date = 20210116;

uint16_t CNT_Conti_Press = 0;

uint16_t Tim_cnt_1s = 0;
uint16_t Tim_cnt_100ms = 0;
uint16_t Tim_cnt_10ms = 0;


	//Float
float Voc_base,Voc_base_;
float Voc_base_2;
float Vo1,Vo2;
float V_Diff,V_Diff_1,V_Diff_2;
float V_Slope;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Door_Open(void);
void Door_Stop(void);
void Door_Close(void);		
void Light_ON(void);		
void Light_OFF(void);		
void Door_manage(void);                                    //Operating time calculate
void PWR_CTRL(void);                                       //Power ON to motor
void Delay_ms(int32_t nms);
static void SystemClock_Config(void);
static void EXTI4_15_IRQHandler_Config(void);
static void TIMx_Config(void);
static void ADC_Config(void);
static void Uart_Config(void);
static void Error_Handler(void);

static void CLOCK_Enable(void);

static void MotorRelay_out_config(void);
static void StatusRelay_out_config(void);
static void Anti_Pressure(void);
static void Anti_Pressure_2(void);
static void Anti_Pressure_3(void);
static void Anti_Pressure_4(void);
static void OpEnd_Detect(void);			//Door unload detect
static void Buzzer_Config(void);
static void Ext_CNTER(void);			//Door unload detect
static uint32_t	TIMDEC(uint32_t TIMB);
static uint16_t ADC_Calculate(void);

/* Private functions ---------------------------------------------------------*/



//Variable to ADC conversion
//uint16_t i,j;
uint16_t adc_32_amnt = 0;
uint16_t adc_32_ave;
float Voc,Voc_;
float iWeight_;
float Voc_adc;

// Main Loop
int main(void)
{
  /* Configure HAL */
  HAL_Init();

  /* Configure the system clock to 48 MHz */
  SystemClock_Config();

  /* Configure LED2 */
  //BSP_LED_Init(LED2);

  /* Configure */
  ADC_Config();
  EXTI4_15_IRQHandler_Config();
  TIMx_Config();
  Uart_Config();

	
	
  /* Enable each GPIO Clock */
  CLOCK_Enable();

  /* Configure IOs in output push-pull mode to drive Relays */
  MotorRelay_out_config();
  StatusRelay_out_config();
  Buzzer_Config();	//No used


  TM_OPEN = 0;
  TM_CLOSE = 0;
  CloseTM2 = TM_MAX - CloseTM1;		//section_time_2 of close operation
  ST_Close = 1;
	
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);	
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);  

  //�γ~:�קK�}�����ȺA�v�TGPIO�PŪ
  Door_Stop();	
  Delay_ms(500);

	//EXTI enable
  if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  
  if(V_Stby == 0){
	V_Stby = ADC_Calculate() *(3.3/4095);
  }
	
	Cycle_jumper = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7);
	
	
  printf("\n\rVer_date.: %d", Ver_date);

  if(Cycle_test == TRUE && Cycle_jumper == 1){
		TM_MAX = 600;
		TM_OPEN = TM_MAX;
		Wait_flg = TRUE;
		ACT_Door = 1;
		V_Stby = 0.3;
		printf("\n\r�`������:��\n");
  }
  
    
	//while (1){
	//}
  /* Infinite Loop */
  while (1)
  { 
		
		if(Cycle_test == FALSE || Cycle_jumper == 0){
			Door_manage();
			PWR_CTRL(); 
			//Anti_Pressure_3();
			
		}else{
			
			if(TM_OPEN == 0 && TM_CLOSE == 0 && Wait_flg == TRUE){
				TM_DLY = 300;
				Wait_flg = FALSE;
			}
			
			if(TM_OPEN == 0 && ACT_Door == 1 && TM_DLY == 0){
				Door_Stop();
				TM_CLOSE = TM_MAX;
				ACT_Door = 2;
				Wait_flg = TRUE;
				Cycle_times_down++;
				Ext_CNTER();
				printf("\n\rNoise test 1");
			}else if(TM_CLOSE == 0 && ACT_Door == 2 && TM_DLY == 0){
				Door_Stop();
				TM_OPEN = TM_MAX;
				ACT_Door = 1;
				Wait_flg = TRUE;
				Cycle_times_up++;
				
				Ext_CNTER();
				//HAL_GPIO_WritePin(PORT_Status_Out, RL_TIME, GPIO_PIN_SET);
				//Delay_ms(RLY_Delay_ms);
				//HAL_GPIO_WritePin(PORT_Status_Out, RL_TIME, GPIO_PIN_RESET);
				printf("\n\rNoise test 2");

			}
			
			Door_manage();
			PWR_CTRL(); 
			//OpEnd_Detect();
			//Anti_Pressure_3();
			
			if(TM_Printf == 0){
				printf("\n\r==============���Ascan1===================");			
				printf("\n\r==============�`������===================");			
				Voc_ = ADC_Calculate() *(3.3/4095);		
				printf("\n\r�ثe�q���� = %f V",Voc_);
				printf("\n\r�ݾ��q��   = %f V",V_Stby);
				printf("\n\rACT_Door = %d",ACT_Door);
				if(TM_OPEN > 0){
					printf("\n\n\r�}���Ѿl�ɶ� = %d ms",TM_OPEN);
					printf("\n\r�}������ = %d\n",Cycle_times_down);
				}
				if(TM_CLOSE > 0){
					printf("\n\n\r�����Ѿl�ɶ� = %d ms",TM_CLOSE);
					printf("\n\r�������� = %d\n",Cycle_times_down);
				}
				if(TM_DLY > 0){
					printf("\n\rTM_DLY = %d",TM_DLY);
				}
					TM_Printf= 10;
				}	
		}

		if(TM_Printf == 0 && (Cycle_test == FALSE || Cycle_jumper == 0)){
			printf("\n\r==============���Ascan2===================");			
			
			if(TM_OPEN > 0 || TM_CLOSE > 0){
				Voc_ = ADC_Calculate() *(3.3/4095);		
				printf("\n\r�ثe�q���� = %f V",Voc_);
				printf("\n\r�ݾ��q��   = %f V",V_Stby);
			}
			
			printf("\n\n\r�ثe�����A = %d",ST_Door);
			
			if(TM_OPEN > 0){
				printf("\n\n\r�}���Ѿl�ɶ� = %d ms",TM_OPEN);
			}
			if(TM_CLOSE > 0){
				printf("\n\n\r�����Ѿl�ɶ� = %d ms",TM_CLOSE);
			}
			
			if(TM_Light_Off > 0){
				//printf("\n\rOPEN_IT= %d",Open_IT);
				printf("\n\n\r�ө������ɶ� = %d ms",TM_Light_Off);
			}
			
			if(TM_Auto_Close > 0){
				//printf("\n\rOPEN_IT= %d",Open_IT);
				printf("\n\n\r�������ݮɶ� = %d ms",TM_Auto_Close);
			}
			if(ST_Anti == 2){
				printf("\n\n\r�������A = %d",ST_Anti);
				printf("\n\r�ܤƲv��� = %f",V_Slope);
				printf("\n\r�ܤƲv = %f",V_Diff);
			}else if(ST_Anti ==3 && ST_Door == 2){
				printf("\n\n\r�������A = %d",ST_Anti);
				printf("\n\n\r��������Ĳ�o: �j��}����");
			}else if(ST_Anti ==3 && ST_Door == 1){
				printf("\n\n\r�������A = %d",ST_Anti);
				printf("\n\n\r�}������Ĳ�o: ����");
			}
						
			if(Close_Segment_Flg == TRUE){		//1-segment mode
				//printf("\n\rOPEN_IT= %d",Open_IT);
				printf("\n\n\r��q���������A");
				printf("\n\rST_CLOSE= %d",ST_Close);
			}
			printf("\n\r");

			TM_Printf = 10;
		}
  }
}

int fputc(int ch, FILE *f){
	HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 1000);
	return ch;
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
void SystemClock_Config(void)
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
    /* Initialization Error */
    while(1); 
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
}

static void CLOCK_Enable(void){
	GPIOA_CLK_ENABLE();
	GPIOB_CLK_ENABLE();
	GPIOC_CLK_ENABLE();
	GPIOD_CLK_ENABLE();
	//GPIOE_CLK_ENABLE();
	//GPIOF_CLK_ENABLE();
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED2 on*/
//  BSP_LED_On(LED2);
  while (1)
  {
  }
}

static void Ext_CNTER(void){
	printf("\n\r----EXT_CNT");
	HAL_GPIO_WritePin(PORT_Status_Out, RL_TIME, GPIO_PIN_SET);
	Delay_ms(RLY_Delay_ms);
	HAL_GPIO_WritePin(PORT_Status_Out, RL_TIME, GPIO_PIN_RESET);
}

void PWR_CTRL(void){
	if(TM_CLOSE > 0 && TM_OPEN > 0){
		//Empty
		//Avoid both the timer work at the same time.		
		Door_Stop();
		printf("\n\n\r===========NG===========");
		printf("\n\rTime_Open =%d",TM_OPEN);
		printf("\n\rTime_Close=%d",TM_CLOSE);
		TM_OPEN = 0;
		TM_CLOSE = 0;
	}else{	
		ST_Door_buf = ST_Door;
		if(Close_Segment_Flg == FALSE){		//1-segment mode
			if(TM_OPEN > 0){
				Door_Open();
				OpEnd_Detect();
			}else if(TM_CLOSE > 0){	
				Door_Close();
				OpEnd_Detect();
			}else{
				Door_Stop();
				ST_Door = 0;
				Op_Flag = FALSE;
				if(ST_Door == 1){// && ST_Anti > 0){      //20201227_OC_Detect
					ST_Anti = 0;                         		//20201227_OC_Detect
				}else if(ST_Door == 2 && ST_Anti < 3){ 		//20201227_OC_Detect
					ST_Anti = 0;                         		//20201227_OC_Detect
				}                                      		//20201227_OC_Detect
				
				//if(ST_Anti < 3){
					//ST_Door = 0;
				//}
			}
		}else{
			if(TM_OPEN > 0){
				Door_Open();
				OpEnd_Detect();
			}else if(TM_CLOSE > 0){				
				Door_Close();
				OpEnd_Detect();
			}else{
				if(TM_OPEN == 0 && TM_CLOSE == 0){
					Op_Flag = FALSE;
					Door_Stop();
					if(ST_Door == 1){
						ST_Door = 0;
						Open_IT = FALSE;
						Close_IT = FALSE;
						Close_IT2 = FALSE;
						
						if(ST_Anti > 0){                       //20201227_OC_Detect
							ST_Anti = 0;                         //20201227_OC_Detect
						}                                      //20201227_OC_Detect

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
						
						if(ST_Anti < 3){                       //20201227_OC_Detect
							ST_Anti = 0;                         //20201227_OC_Detect
						}                                      //20201227_OC_Detect

					}
				}
			}
		}
	}
	
	if(TM_Light_Off > 0){
		Light_ON();
	}else{
		Light_OFF();
	}
}

void Door_manage(void){
	if(ST_BTN == TRUE){							//����U�F���O
		ST_BTN = FALSE;
		if(Close_Segment_Flg == FALSE){			//��q������:�L
			switch(ACT_Door){					//���O�P�_
				case 0:							//���O=����
					ST_Door = 0;
					ST_Anti = 0;
					TM_OPEN = 0;
					TM_CLOSE = 0;
					TM_Light_Off = Time_Light;
					break;
				
				case 1:							//���O=�}��
					ST_Door = 1;
					TM_CLOSE = 0;
					TM_OPEN = TM_MAX;
					//TM_Light_Off = TM_MAX + Time_Light;
					TM_AntiDly = Time_AntiDly;
					TM_EndDetec = 10;
					break;
				
				case 2:							//���O=����
					ST_Door = 2;
					if(ST_Anti == 3){ //20201227_OC_Detect
						break;
					}
					TM_OPEN = 0;
					TM_CLOSE = TM_MAX;
					//TM_Light_Off = TM_MAX + Time_Light;
					TM_AntiDly = Time_AntiDly;	//20201227_OC_Detect
					TM_EndDetec = 10;
					AClose_Flg = FALSE;
					TM_Auto_Close = 0;
					break;
				
				default:
					TM_OPEN = 0;
					TM_CLOSE = 0;
			}
//			printf("\n\r������O = %d",ACT_Door);
		}else if(Close_Segment_Flg == TRUE){	//��q������:��
			switch(ACT_Door){
			//-------------------���O=����--------------------//
				case 0:
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

					}else if(ST_Door == 2){			//�e���A:����
						CloseTM_Remain = TM_CLOSE;
						if(ST_Close == 1){
								//CloseTM_Remain = TM_CLOSE;
								if(CloseTM_Remain > 0){
									Close_IT = TRUE;
								}else if(CloseTM_Remain == 0){
									Close_IT = FALSE;
									ST_Close = 2;		//���ݲĤG�qclose���O
								}
						}else if(ST_Close == 2){
								//CloseTM_Remain = TM_CLOSE;
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
			//-------------------���O=���� End----------------//
			//-------------------���O=�}��--------------------//
				case 1:
					ST_Door = 1;
					Open_IT = FALSE;
					Close_IT = FALSE;
					Close_IT2 = FALSE;
					TM_CLOSE = 0;
					TM_OPEN = TM_MAX;
					break;
			//-------------------���O=�}�� End----------------//
			//-------------------���O=����--------------------//
				case 2:
					if(ST_Anti == 3){	 //20201227_OC_Detect
						break;
					}
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
					TM_AntiDly = 10;	//20201227_OC_Detect
					break;
			//-------------------���O=���� End-----------------//
			// ----- Else ----- //
				default:
					TM_OPEN = 0;
					TM_CLOSE = 0;
					//break;
			}
		}
	}
	
	/*if(TM_OPEN == 0 && TM_CLOSE == 0){
		ST_Door = 0;
	}*/
	if(Cycle_test == TRUE && Cycle_jumper == 1){
	
	}else{
		// �}���᩵��ɶ����L�۰�����
		if(Flag_AutoClose == TRUE){										//�۰������\��: ON
			if(Close_Segment_Flg == FALSE){								//��q�}���\��:�L
				if(	ST_Door == 0 		&& 
					ST_Door_buf == 1 	&&								//�P�_�����e�����A�O�_���}��
					ST_Anti < 3){										//����3���\��3���Ұ�
					printf("\n\r�۰������X�� & ���ݮɶ��]��");
					TM_Auto_Close = Time_Auto_Close;					//�]�w�۰������˼Ʈɶ�
					AClose_Flg = TRUE;									//�۰������X��:ON
				}else if(AClose_Flg == TRUE && TM_Auto_Close == 0){		//�۰������˼Ʈɶ�����
					printf("\n\r�۰��������ݮɶ���F");
					printf("\n\r�����ɶ��]��");
					AClose_Flg = FALSE;									//�۰������X��:OFF
					TM_CLOSE = TM_MAX;									//�����ɶ��]�w
				}
			}
		}
	}
	//�]�w�}���P�����ɪ��ө�
	//�ʧ@�᩵��Time_Light�ɶ���A�����ө�
	if(TM_OPEN > 0){
		TM_Light_Off = TM_OPEN + Time_Light;
	}else if(TM_CLOSE > 0){
		TM_Light_Off = TM_CLOSE + Time_Light;
	}
}	


//******************Relay control******************//
void Door_Close(void){
//	printf("\n\r----CLOSE_Relay");
	if(Flag_Motor_Direction == TRUE){
		HAL_GPIO_WritePin(PORT_Motor_Out, RLY_DIR, GPIO_PIN_RESET);
	}else{
		HAL_GPIO_WritePin(PORT_Motor_Out, RLY_DIR, GPIO_PIN_SET);
	}
	Delay_ms(RLY_Delay_ms);
	HAL_GPIO_WritePin(PORT_Motor_Out, RLY_ACT, GPIO_PIN_SET);
	Delay_ms(RLY_Delay_ms);
	HAL_GPIO_WritePin(PORT_Motor_MOS, MOS_ACT, GPIO_PIN_RESET);	//0:H 1:L
}

void Door_Stop(void){
//	printf("\n\r----STOP_Relay");
	HAL_GPIO_WritePin(PORT_Motor_MOS, MOS_ACT, GPIO_PIN_SET);			
	Delay_ms(RLY_Delay_ms);
	HAL_GPIO_WritePin(PORT_Motor_Out, RLY_ACT, GPIO_PIN_RESET);
	Delay_ms(RLY_Delay_ms);
	HAL_GPIO_WritePin(PORT_Motor_Out, RLY_DIR, GPIO_PIN_RESET);		

}

void Door_Open(void){
//	printf("\n\r----OPEN_Relay");
	if(Flag_Motor_Direction == TRUE){
		HAL_GPIO_WritePin(PORT_Motor_Out, RLY_DIR, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(PORT_Motor_Out, RLY_DIR, GPIO_PIN_RESET);
	}
		Delay_ms(RLY_Delay_ms);
		HAL_GPIO_WritePin(PORT_Motor_Out, RLY_ACT, GPIO_PIN_SET);		
		Delay_ms(RLY_Delay_ms);
		HAL_GPIO_WritePin(PORT_Motor_MOS, MOS_ACT, GPIO_PIN_RESET);

}

// �ө�:ON
void Light_ON(void){
	HAL_GPIO_WritePin(PORT_Status_Out, RL_ACT, GPIO_PIN_SET);
	//printf("\n\rLight ON!");
}

// �ө�:OFF
void Light_OFF(void){
	HAL_GPIO_WritePin(PORT_Status_Out, RL_ACT, GPIO_PIN_RESET);
	//printf("\n\rLight OFF!");
}

//******************Relay control end******************//

static void OpEnd_Detect(void){
	if(TM_EndDetec == 0){
		if(Op_Flag == FALSE){
			TM_DoorOperateDly = 5;	//Delay 0.5 second.
			Op_Flag = TRUE;
		}else{
			Voc = ADC_Calculate() *(3.3/4095);		
			if(Voc <= V_Stby && TM_DoorOperateDly == 0){
				printf("\n\n\r�����-����B��!\n\n");
				TM_OPEN = 0;
				TM_CLOSE = 0;
				TM_Light_Off = Time_Light;
				ST_Anti = 0;
				Op_Flag = FALSE;
			}
		}
	}
}

static void MotorRelay_out_config(void){
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	
	//Motor control relays config.
	GPIO_InitStruct.Pin = RLY_ACT | RLY_DIR;
	HAL_GPIO_Init(PORT_Motor_Out, &GPIO_InitStruct);
	
	//POWER MOSFET config.
	GPIO_InitStruct.Pin = MOS_ACT;
	HAL_GPIO_Init(PORT_Motor_MOS, &GPIO_InitStruct);
	
	//Initial condition setting.
	HAL_GPIO_WritePin(PORT_Motor_MOS, MOS_ACT, GPIO_PIN_SET);	//1:OFF, 0:0N
	HAL_GPIO_WritePin(PORT_Motor_Out, RLY_DIR, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PORT_Motor_Out, RLY_ACT, GPIO_PIN_RESET);	//1:ON, 0:0FF
}

static void StatusRelay_out_config(void){
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	GPIO_InitStruct.Pin = RL_ACT | RL_TIME | RL_POS;
	HAL_GPIO_Init(PORT_Status_Out, &GPIO_InitStruct);
		
	//Initial condition setting.
	HAL_GPIO_WritePin(PORT_Status_Out, RL_ACT, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PORT_Status_Out, RL_TIME, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PORT_Status_Out, RL_POS, GPIO_PIN_RESET);
}

//EXIT Configures
static void EXTI4_15_IRQHandler_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOC clock */
	EXTI_CTRL_GPIO_CLK_ENABLE();
	EXTI_CTRL_LOCK_CLK_ENABLE();

  /* Configure PC.13 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;//GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = EXTI_CTRL_PIN;
  HAL_GPIO_Init(EXTI_CTRL_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = RM_LOCK;
  HAL_GPIO_Init(PORT_LOCK, &GPIO_InitStructure);

  /* Enable and set EXTI line 4_15 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
  //HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

//EXTI line detection callbacks
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	printf("\n\r������O: ");
	CNT_Conti_Press = 0;
	Flag_JOG = FALSE;
	ST_Door_buf = ST_Door;
	
	switch(GPIO_Pin){
		case W_STOP:
			if(Lock_CTRL == TRUE)	break;
			ST_BTN = TRUE;
			ACT_Door = 0;
			ST_Anti = 0;
			
			if(AClose_Flg == TRUE){
				printf("\n\r���s���J���������ɶ�: %d ms", Time_Auto_Close);
				TM_Auto_Close = Time_Auto_Close;					//�]�w�۰������˼Ʈɶ�
			}
			printf("\n\rSTOP!\n");
			break;
		
		case W_OPEN:
			if(Lock_CTRL == TRUE)	break;
			
			if(Flag_Func_JOG == TRUE){
				//************JOG detect************//
				//�T�ʰ���
				ST_Press = HAL_GPIO_ReadPin(PORT_OPEN, W_OPEN);
				while(ST_Press == 0){
					CNT_Conti_Press++;
					printf("\n\rOPEN key.....press\n");
					ST_Press = HAL_GPIO_ReadPin(PORT_OPEN, W_OPEN);
					if(CNT_Conti_Press > Conti_times){		// ������U�ëO��: 50 times
						if(Flag_JOG == FALSE){
							//TM_DoorOperateDly = 5;
							Door_Open();
						}
						Flag_JOG = TRUE;
						printf("TM_DoorOperateDly = %d", TM_DoorOperateDly);
						//�^�¹q������, �P�_�O�_���
						Voc = ADC_Calculate() *(3.3/4095);		
						if(Voc > V_Stby ){//&& TM_DoorOperateDly == 0){	//�����^�¹q��>�ݾ��q��?
							Door_Open();
							Light_ON();
						}else{										//�^�¹q��<=�ݾ��q���ɰ���
							Door_Stop();
						}
						printf("\n\rJOG Mode:OPEN...\n");

						//OpEnd_Detect();
					}
				}
				printf("\n\rContinue = %d\n",CNT_Conti_Press);
				if(Flag_JOG == TRUE){
					Door_Stop();
					TM_Light_Off = Time_Light;
					printf("\n\rDoor Stop.....\n");
					break;
				}
				//************JOG detect end************//
			}
			if(ST_Door_buf == 2){
				printf("\n\r--------�ߧY����--------");
				Delay_ms(100);
				Door_Stop();
				Op_Flag = TRUE;				
			}
			
			ST_BTN = TRUE;
			ACT_Door = 1;
			printf("\n\rOPEN!\n");
			break;

		case W_CLOSE:			
			if(Lock_CTRL == TRUE)	break;
			
			if(Flag_Func_JOG == TRUE){
			// JOG detect
				ST_Press = HAL_GPIO_ReadPin(PORT_CLOSE, W_CLOSE);
				while(ST_Press == 0){
					CNT_Conti_Press++;
					printf("\n\rCLOSE key.....press\n");
					ST_Press = HAL_GPIO_ReadPin(PORT_CLOSE, W_CLOSE);
					if(CNT_Conti_Press > Conti_times){		// 50 times
						Flag_JOG = TRUE;
						printf("\n\rJOG Mode:CLOSE...\n");
						Door_Close();
						Light_ON();
						//OpEnd_Detect();
						//...�ثe����������_���\��,
						//�p�G����,�A�NOPEN����챱��[�J
					}
				}
				printf("\n\rContinue = %d\n",CNT_Conti_Press);
				if(Flag_JOG == TRUE){
					Door_Stop();
					TM_Light_Off = Time_Light;
					printf("\n\rDoor Stop.....\n");
					break;
				}
				// JOG end
			}
			
			if(ST_Door_buf == 1){
				printf("\n\r--------�ߧY����--------");
				Delay_ms(100);
				Door_Stop();
				Op_Flag = TRUE;
			}

			ST_BTN = TRUE;
			ACT_Door = 2;
			printf("\n\rClose!\n");
			break;
		
		case RM_LOCK:
			if(Lock_CTRL == TRUE){
				Lock_CTRL = FALSE;
				printf("\n\rUNLOCK~~~~~~!\n");
			}else{
				Lock_CTRL = TRUE;
				printf("\n\rLOCK~~~~~~!\n");
			}
			break;
		
		default:
				break;
	}

}

//	TIMx handle
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	Tim_cnt_10ms++;
	Tim_cnt_100ms++;
	Tim_cnt_1s++;
//	printf("\n\r %d",Tim_cnt_100ms);
	
	// 0.01 sec
	if(Tim_cnt_10ms == 10){
		Tim_cnt_10ms = 0;
		//TM_OPEN 					= TIMDEC(TM_OPEN);
//		printf("\n\r Tim_cnt_10ms");
	}
	
	// 0.1 sec
	if(Tim_cnt_100ms == 100){
		Tim_cnt_100ms = 0;
		TM_OPEN 					= TIMDEC(TM_OPEN);
		TM_CLOSE 					= TIMDEC(TM_CLOSE);
		TM_AntiDly  			= TIMDEC(TM_AntiDly);
		TM_AntiDly2 			= TIMDEC(TM_AntiDly2);
		TM_AntiDly4 			= TIMDEC(TM_AntiDly4);
		TM_EndDetec 			= TIMDEC(TM_EndDetec);
		TM_DoorOperateDly = TIMDEC(TM_DoorOperateDly);
		TM_Printf 				= TIMDEC(TM_Printf);
		TM_DLY 						= TIMDEC(TM_DLY);
		TM_Light_Off 			= TIMDEC(TM_Light_Off);
		TM_Auto_Close 		= TIMDEC(TM_Auto_Close);
		
//		printf("\n\r Tim_cnt_100ms");
//		printf("\n\r**************************");
	}
	
	// 1 sec
	if(Tim_cnt_1s == 100){
		Tim_cnt_1s = 0;
	
/*
	//������
		if(TM_OPEN > 0 || TM_CLOSE >0){
			V_Stby = V_Stby + 0.01;
		}else{
			V_Stby = 0.2;
		}
	//������end
*/	
//		printf("\n\r Tim_cnt_1s");
	}

}

//1s timer
static void TIMx_Config(void)
{	
	/*##-1- Configure the TIM peripheral #######################################*/
	/* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
	uwPrescalerValue = (uint32_t)(SystemCoreClock / 10000) - 1;

	/* Set TIMx instance */
	TimHandle.Instance = TIMx;

	/* Initialize TIMx peripheral as follows:
	+ Period = 10000 - 1
	+ Prescaler = (SystemCoreClock/10000) - 1
	+ ClockDivision = 0
	+ Counter direction = Up
	*/

	TimHandle.Init.Period            = (1*10) - 1;   // 1*10ms
	TimHandle.Init.Prescaler         = uwPrescalerValue;
	TimHandle.Init.ClockDivision     = 0;
	TimHandle.Init.CounterMode       = TIM_COUNTERMODE_DOWN;
	TimHandle.Init.RepetitionCounter = 0;
	TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  
	if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

static void ADC_Config(void){
	AdcHandle.Instance          = ADCx;
  if (HAL_ADC_DeInit(&AdcHandle) != HAL_OK)
  {
    /* ADC de-initialization Error */
    Error_Handler();
  }

  AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV2;      /* Synchronous clock mode, input ADC clock with prscaler 2 */

  AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;            /* 12-bit resolution for converted data */
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;           /* Right-alignment for converted data */
  AdcHandle.Init.ScanConvMode          = ADC_SCAN_DIRECTION_FORWARD;    /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
  AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;           /* EOC flag picked-up to indicate conversion end */
  AdcHandle.Init.LowPowerAutoPowerOff  = DISABLE;
  AdcHandle.Init.LowPowerAutoWait      = DISABLE;                       /* Auto-delayed conversion feature disabled */
  AdcHandle.Init.ContinuousConvMode    = ENABLE;                        /* Continuous mode enabled (automatic conversion restart after each conversion) */
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE; /* Parameter discarded because software trigger chosen */
  AdcHandle.Init.DMAContinuousRequests = ENABLE;                        /* ADC DMA continuous request to match with DMA circular mode */
  AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;      /* DR register is overwritten with the last conversion result in case of overrun */
  AdcHandle.Init.SamplingTimeCommon    = ADC_SAMPLETIME_1CYCLE_5;

  /* Initialize ADC peripheral according to the passed parameters */
  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* ### - 2 - Start calibration ############################################ */
  if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* ### - 3 - Channel configuration ######################################## */
  sConfig.Channel      = ADCx_CHANNEL;               /* Channel to be converted */
  sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* ### - 4 - Start conversion in DMA mode ################################# */
  if (HAL_ADC_Start_DMA(&AdcHandle,
                        (uint32_t *)aADCxConvertedData,
                        ADC_CONVERTED_DATA_BUFFER_SIZE
                       ) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Uart_Config(void){
	/*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance        = USARTx;

  UartHandle.Init.BaudRate   = 9600;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }  
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }
}



static void Anti_Pressure_3(void){
	switch(ST_Anti){
		case 0:
		//Stand-by
			if(TM_AntiDly > 0){
				ST_Anti = 1;
			}
			break;
		
		case 1:
		//�Ѧҭȭp��
			if(TM_AntiDly > 0){
				// Empty
				Calc_Times = 0;
				Voc_amt = 0;
			}else if(TM_AntiDly == 0 && Calc_Times < 10){
				// Ū��ADC buffer��(32������)
				Vadc_buf = ADC_Calculate();
				Voc_amt = Voc_amt + Vadc_buf;
				Calc_Times++;
			}
			
			//�p��Ѧҭ�
			if(Calc_Times == 10){
				Vo1 = (Voc_amt/10)*(3.3/4095);
				Calc_Times = 0;
				ST_Anti = 2;
				TM_AntiDly4 = Time_AntiDly4;	
				OverSlope_Times = 0;
			}
			break;
		
		case 2:
			if(ST_Door == 1){					//20210103
				V_Slope = Slope_Open;
			}else if(ST_Door == 2){
				V_Slope = Slope_Close;
			}													//20210103_end
			

			//�����B��q�y			
			if(TM_AntiDly4 == 0){// && Anti_flg2 == TRUE){
				if(Calc_Times < 10){
					// Ū��ADC buffer��(32������)
					Vadc_buf = ADC_Calculate();
					Voc_amt = Voc_amt + Vadc_buf;
					Calc_Times++;
				}
			
				if(Calc_Times == 10){
					Vo2 = (Voc_amt/10)*(3.3/4095);
					V_Diff = 10*(Vo2-Vo1)/5;  // (Vo1-Vo2)/0.5sec
					TM_AntiDly4 = Time_AntiDly4;
					Voc_amt = 0;
					
					//printf("\n\rV_Diff = %f",V_Diff);
					//printf("\n\rVo1 = %f",Vo1);
					//printf("\n\rVo2 = %f",Vo2);
					
					if(V_Diff >= V_Slope){
						OverSlope_Times++;
						Calc_Times = 0;
						printf("\n\r�������ߦ��� = %d ��", OverSlope_Times);

						/*if(OverSlope_Times == 1){
							V_Diff_1 = V_Diff;
						}else if(OverSlope_Times == 2){
							V_Diff_2= V_Diff;
						}*/
				
					}else{
						OverSlope_Times = 0;
						//printf("\n\r�������ߦ��� = %d ��", OverSlope_Times);
					}
					
					if(OverSlope_Times == OS_Occur_Times){
						ST_Anti = 3;
						TM_OPEN = 0;		//20210103
						TM_CLOSE = 0;
						//20210103//ST_Door = 0;
						TM_AntiDly2 = 10;
						
						//printf("\n\r************************");
						//printf("\n\rVo1 = %f",Vo1);
						//printf("\n\rVo2 = %f",Vo2);
						//printf("\n\r************************");
						//printf("\n\rAnti_press ratio = %f",V_Slope);
						//printf("\n\rSlope= %f",V_Diff);
						//printf("\n\rSlope1= %f",V_Diff_1);
						//printf("\n\rSlope2= %f\n",V_Diff_2);
						//printf("\n\r************************");
						printf("\n\n\r�������ߦ��ƹF %d ��", OverSlope_Times);
						printf("\n\r*********�������󦨥�-�����Ұ�********************");
						
					}else{
						ST_Anti = 2;
						Vo1 = Vo2;
						Calc_Times = 0;
						Voc_amt = 0;
						TM_AntiDly4 = 5;
					}
				}
			}
			break;
		
		case 3:
		//�O�@�ʧ@ & �}��
			if(TM_AntiDly2 == 0){			//20210103
				//printf("\n\rST_Anti = %d",ST_Anti);
				if(ST_Door == 1 && TM_OPEN == 0){
					//ST_Door = 0;
					//ST_Anti = 0;
				}else if(ST_Door == 2){
					ST_Door = 1;
					//ST_Anti = 0;
					TM_OPEN = TM_MAX;
				}
			}													//20210103_end
			break;
		
		default:
			// Empty
			break;
	}
}

static uint16_t ADC_Calculate(void){
	uint16_t   i;
	uint16_t   Voc_Buf;
	uint32_t   adc_32_amnt = 0;

	for (i=0;i<=31;i++){
		adc_32_amnt = adc_32_amnt + aADCxConvertedData[i];
	}
	
	Voc_Buf = adc_32_amnt / 32;
	
	return Voc_Buf;
}

static void Buzzer_Config(void){
	//buzz config.
	//GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	//GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
	//GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	//GPIO_InitStruct.Pin = Buzz;
	//HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	//buzz config...end
}

static uint32_t	TIMDEC(uint32_t TIMB){
	uint32_t TIM_Buf = TIMB;
	if(TIMB == 0) return TIMB;
	
	TIM_Buf--;
	
	return TIM_Buf;
}

void Delay_ms(int32_t nms) 
 {
  int32_t temp; 
  SysTick->LOAD = 8000*nms; 
  SysTick->VAL=0X00;				//�M�ŭp�ƾ� 
  SysTick->CTRL=0X01;				//�ϯ�A���s�O�L�ʧ@�A�ĥΥ~�������� 
  do 
  { 
       temp=SysTick->CTRL;	//Ū����e�˭p�ƭ� 
  }
     while((temp&0x01)&&(!(temp&(1<<16))));//���ݮɶ���F 
     
     SysTick->CTRL=0x00; 		//�����p�ƾ� 
     SysTick->VAL =0X00; 		//�M�ŭp�ƾ� 
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
