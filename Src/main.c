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
#define TRUE 1
#define FALSE	0
#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  32)   /* Definition of ADCx conversions data table size */

// =====I2C=====//
#define I2C_ADDRESS        0xA0	//0x30F
#define I2C_TIMING      0x00A51314


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Handle Declaration ---------------------------------------------------------------*/
ADC_HandleTypeDef         AdcHandle;	// ADC handle declaration
ADC_ChannelConfTypeDef    sConfig;		// ADC channel configuration structure declaration
TIM_HandleTypeDef         TimHandle;	
TIM_HandleTypeDef         TimHandle17;	
UART_HandleTypeDef        UartHandle;	// UART handler declaration
I2C_HandleTypeDef 				I2cHandle;	// I2C handler declaration

static GPIO_InitTypeDef   GPIO_InitStruct;

/* Private functions ---------------------------------------------------------*/
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


/* Private macro -------------------------------------------------------------*/
//*******�ѼƳ]�w*******//
uint8_t EE_Default = FALSE;
uint8_t Flag_WindowsDoor; 		//���������:	
uint8_t Flag_CycleTest;             //�`������(���ɴ���)
uint8_t Flag_AntiPress;			//�����\��
uint8_t Times_JOG;				//�����P�w����: �T�ʧP�w����, �j��:�T��, �p��:�@��
uint8_t Times_Remote_Lock;
uint8_t PWM_Grade;	
uint8_t Flag_Rate_Regulate;
uint8_t Flag_Buzzer;
uint8_t Flag_AutoClose;			//�۰������\��
uint8_t Flag_Func_JOG;				//�T�ʥ\��
uint8_t Flag_Motor_Direction;		//���F�B���V
uint8_t Flag_Remote_Lock;				//��q�\��
uint8_t Flag_Low_Operate;				//

uint16_t TM_MAX;                  //�}�����̪��B��ɶ� TM_MAX * 100ms
uint16_t TM_WindowsDoor_ClosePart1;			 // �������Ĥ@�q�����ɶ�: n*0.1sec
uint16_t TM_DLY;							//cycle-test���ݬ��(*100ms)
uint16_t TM_DLY_Value;
uint16_t Time_Auto_Close;			// �۰���������ɶ�: n * 0.1sec.
uint16_t Time_Light;				//�ө��B��ɶ�n * 0.1sec

float Anti_Weight_Open;					//�����v��(�i�p��):�}��(�V�p�V�F��),��ĳ>1
float Anti_Weight_Close;				//�����v��(�i�p��):����(�V�p�V�F��),��ĳ>1
float Volt_StandBy;				//�ݾ��q��(��0���즸�Ұʰ���),��ĳ��0.3~0.5

//*******�ѼƳ]�w����*******//

/* Private variables ---------------------------------------------------------*/
	//Boolean
uint8_t ST_BTN;                            //(Remote) controller trigger(0:standby, 1:cmd trigger
uint8_t Open_IT;                           //Interrupt in open
uint8_t Close_IT;                          //Interrupt in close 2-1
uint8_t Close_IT2;                         //Interrupt in close 2-2
uint8_t OpEnd_Detect_Start_Flag = FALSE;
uint8_t Anti_flg2 = TRUE;
uint8_t AClose_Flg = FALSE;				// �۰������ʧ@�X��
uint8_t Wait_flg;
uint8_t Flag_LOCK = FALSE;					//��q�ʧ@�X��,�w�]FALSE,����i�ʧ@.
uint8_t Flag_JOG;					//�T�ʰʧ@�X��
uint8_t Flag_Light;
uint8_t Flag_IR;
uint8_t Flag_SMK;
uint8_t Flag_Door_UpLimit;
uint8_t Flag_Door_DownLimit;


	//8-bits
uint8_t ACT_Door = 0;                   //Controller's cmd (0:Stop /1:Open /2:Close)
uint8_t ST_Door = 0;                    //Operating status (0:Stop or standby /1:Opening /2:Closing)
uint8_t ST_Door_buf, ST_Door_buf2;
uint8_t ST_Close;                       //Recode the 2-seg close cmd
uint8_t	ST_Anti;
uint8_t Vop_Cnt;
uint8_t OverSlope_Times = 0;
uint8_t OS_Occur_Times = 2;
uint8_t Cycle_jumper;
uint8_t ST_Press;
uint8_t aRxBuffer[256];
uint8_t PWM_Period = 100;
uint8_t PWM_Duty = 50;
uint8_t PWM_Count = 0;
uint8_t TM_IR_Lock = 0;
uint8_t Auto_Close_Mode;
uint8_t CNT_Jog_Press = 0;
uint8_t CNT_LOCK_Press = 0;
uint8_t ADC_Detect_Start_Flag = 0;
uint8_t Times_OverADC = 0;
uint8_t Times_OverADC_Target = 2;	//�����QĲ�o����
uint8_t Anti_Event = 0;				//����Ĳ�o���O 0:���`�B�� 1:�}������ 2:��������
uint8_t Anti_Event_buf = 0;				//����Ĳ�o���O 0:���`�B�� 1:�}������ 2:��������
uint8_t ST_Low_Operate = 0;	
uint8_t Buzz_Type = 0;
	//16-bits

uint16_t Vadc_buf;
uint16_t Calc_Times;
uint16_t Voc_amt;
uint16_t Vadc_amt;
uint16_t TM_Printf = 10;
static uint16_t aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];	  //Variable containing ADC conversions data
uint16_t TM_OPEN = 0;                   //Time: Door open
uint16_t TM_CLOSE = 0;                  //Time: Door close
uint16_t TM_CLOSE_b = 0;                  //Time: Door close
uint16_t TM_AntiDly;
uint16_t Time_AntiDly = 20;
uint16_t TM_AntiDly2;
uint16_t TM_AntiDly3;
uint16_t Time_AntiDly4 = 1;
uint16_t TM_EndDetec;
uint16_t TM_DoorOperateDly = 20;        //��찻������ɶ�(*100ms)
uint16_t OpenTM_Remain = 0;             //��q���}���Ѿl�ɶ�
uint16_t CloseTM_Remain = 0;            //��q�������Ѿl�ɶ�
uint16_t TM_Light_ON = 0;
uint16_t TM_Auto_Close = 0;
uint16_t CloseTM2;
uint16_t Tim_cnt_1s = 0;
uint16_t Tim_cnt_100ms = 0;
uint16_t Tim_cnt_10ms = 0;
uint16_t Tim_TEST = 0;

uint16_t ADC_OPEN_MAX = 3700;	//[???]
//uint16_t ADC_OPEN_MAX;
uint16_t ADC_OPEN_MIN;
uint16_t ADC_CLOSE_MAX = 3700;	//[???]
//uint16_t ADC_CLOSE_MAX;
uint16_t ADC_CLOSE_MIN;
uint16_t ADC_OPEN_MAX_b;
uint16_t ADC_OPEN_MIN_b;
uint16_t ADC_CLOSE_MAX_b;
uint16_t ADC_CLOSE_MIN_b;
uint16_t ADC_Anti_Max;

uint16_t TM_Anti_Occur = 0;
uint16_t TM_Save = 2*60*60;			//�B�স���x�s���g��
uint16_t TM_Buzz_ON = 0;
uint16_t TM_Buzz_OFF = 0;
uint16_t TM_ADC_Relaod = 0;
uint16_t TM_Low_Operate = 0;

	//32-bits
uint32_t RLY_Delay_ms = 20;			   //Relay_Delay_time(*1ms)
uint32_t uwPrescalerValue = 0;         // Prescaler declaration
uint32_t Cycle_times_up = 0;
uint32_t Cycle_times_down = 0;
uint32_t Ver_date = 20210228;
uint32_t REC_Operate_Times;

uint8_t TXBuf[4];
uint16_t ee_address;
uint8_t x;


	//Float
float Voc_base,Voc_base_;
float Voc_base_2;
float Vo1,Vo2;
float V_Diff,V_Diff_1,V_Diff_2;
float Anti_Weight;
float Vadc_ave;


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
static void TIM16_Config(void);
static void TIM17_Config(void);
static void ADC_Config(void);
static void Uart_Config(void);
static void I2C_Config(void);
static void Error_Handler(void);

static void CLOCK_Enable(void);

static void MotorRelay_out_config(void);
static void StatusRelay_out_config(void);
static void Anti_Pressure_5(void);
static void OpEnd_Detect(void);			//Door unload detect
static void OpEnd_Detect_2(void);			//Door unload detect
static void Buzzer_Config(void);
static void ControlBox_Config(void);
static void Ext_CNTER(void);			//Door unload detect
static uint16_t	TIMDEC(uint16_t TIMB);
static uint16_t	TIMINC(uint16_t TIMB);
static uint16_t ADC_Calculate(void);
uint16_t* BubbleSort(uint16_t arr[], uint16_t len);
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);	// Confirm the I2C R/W datas.

static void Buzz_on(void);
static void Buzz_off(void);
static void Buzz_out(uint16_t ON_Time, uint16_t OFF_Time);

static void Parameter_Load(void);	//EEPROM�Ѽ�Ū��
static void SMK_CTRL(void);	    //�ϷP����
static void Cycle_Test(void);	//
static void IR_CTRL(void);	    //
static void Light_CTRL(void);	//
static void Auto_Close_CTRL(void);	//
static void Operate_ADC_Detect(void);

//I2C Package
uint8_t I2C_TX_Buffer_u8[1];
uint8_t I2C_TX_Buffer_u16[2];
uint8_t I2C_TX_Buffer_u32[4];

static void Debug_Monitor(void);
static void Operate_Infor_Save(void);
static void CtrlBox_Light_Up(void);
static void CtrlBox_Light_Down(void);
static void CtrlBox_Light_OFF(void);



/* Private functions ---------------------------------------------------------*/


//Variable to ADC conversion
//uint16_t i,j;
uint16_t adc_32_amnt = 0;
uint16_t adc_32_ave;
float Voc,Voc_;
float iWeight_;
float Voc_adc;

uint16_t TM_Buzz;// = 30;

// Main Loop
int main(void)
{		
  /* Configure HAL */
  HAL_Init();

  /* Configure the system clock to 48 MHz */
  SystemClock_Config();

  /* Configure */
  ADC_Config();
  EXTI4_15_IRQHandler_Config();
  TIM16_Config();
  TIM17_Config();
  Uart_Config();
  I2C_Config();
	
  /* Enable each GPIO Clock */
  CLOCK_Enable();
	
  /* Configure IOs in output push-pull mode to drive Relays */
  MotorRelay_out_config();
  StatusRelay_out_config();
  Buzzer_Config();	//No used
  ControlBox_Config();	//No used

  // Parameter access
	EE_Default = FALSE;
  Parameter_Load();

  TM_OPEN = 0;
  TM_CLOSE = 0;
  CloseTM2 = TM_MAX - TM_WindowsDoor_ClosePart1;		//section_time_2 of close operation
  if(Flag_WindowsDoor == TRUE && CloseTM2 <= 0){
		Flag_WindowsDoor = FALSE;
		printf("\n\r�������\��: OFF\n");
  }
  ST_Close = 1;
	
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);	
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);  

  //�γ~:�קK�}�����ȺA�v�TGPIO�PŪ
  Door_Stop();	
  Delay_ms(500);

	//EXTI enable
	//TIM16 Enable
  if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK){
    Error_Handler();
  }

  if(Volt_StandBy == 0){
		Volt_StandBy = ADC_Calculate() *(3.3/4095) * 1.3;
  }
	
  //Cycle_jumper = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7);
	
  printf("\n\rVer_date.: %d", Ver_date);

  if(Flag_CycleTest == TRUE){
	TM_MAX = 600;
	TM_OPEN = TM_MAX;
	Wait_flg = TRUE;
	ACT_Door = 1;
	Volt_StandBy = 0.3;
	printf("\n\r�`������:��\n");
  }
	
  //�}�����ܭ�
  Buzz_on();
  Delay_ms(500);
  Buzz_off();
  
  while (1){ 
	if(Flag_CycleTest == TRUE){
		//�`������
		Cycle_Test();
		
	}else{ 
		//Main function
		Door_manage();
		Light_CTRL();
		IR_CTRL();
		SMK_CTRL();
		PWR_CTRL(); 
		Operate_ADC_Detect();
		Auto_Close_CTRL();
		Anti_Pressure_5();
		Debug_Monitor();
		Operate_Infor_Save();
	}
	
	
  }
}

static void Debug_Monitor(void){
	//�ثe���A����
	if(TM_Printf == 0){
		printf("\n\r==============���Ascan1===================");			
			Voc_ = ADC_Calculate() *(3.3/4095);		
			printf("\n\r�ثe�q���� = %f V",Voc_);
		if(TM_OPEN > 0 || TM_CLOSE > 0){
			Voc_ = ADC_Calculate() *(3.3/4095);		
			printf("\n\r�ثe�q���� = %f V",Voc_);
			printf("\n\r�ݾ��q��   = %f V",Volt_StandBy);
		}
		
			printf("\n\r***********ADC = %d",ADC_Calculate());
		
		
		printf("\n\n\r�ثe�����A = %d",ST_Door);
		
		if(TM_OPEN > 0){
			printf("\n\n\r�}���Ѿl�ɶ� = %d ms",TM_OPEN);
		}
		if(TM_CLOSE > 0){
			printf("\n\n\r�����Ѿl�ɶ� = %d ms",TM_CLOSE);
		}
		
		if(TM_Light_ON > 0){
			//printf("\n\rOPEN_IT= %d",Open_IT);
			printf("\n\n\r�ө������ɶ� = %d ms",TM_Light_ON);
		}
		
		if(TM_Auto_Close > 0){
			//printf("\n\rOPEN_IT= %d",Open_IT);
			printf("\n\n\r�������ݮɶ� = %d ms",TM_Auto_Close);
		}
		
		if(ST_Anti > 0){
			printf("\n\n\r�������A = %d",ST_Anti);
			printf("\n\n\r�������v�� = %f",Anti_Weight);
			printf("\n\n\r�����ɽu�� = %d",ADC_Anti_Max);
		}
					
		if(Flag_WindowsDoor == TRUE){		//1-segment mode
			//printf("\n\rOPEN_IT= %d",Open_IT);
			printf("\n\n\r�����������A");
			printf("ST_CLOSE= %d",ST_Close);
		}
		
		if(Anti_Event > 0){
			printf("\n");
			printf("\n\rAnti_Event = %d",Anti_Event);
			printf("\n\n\r�����}������� = %d",TM_Anti_Occur);
		}
		
		if(ADC_Detect_Start_Flag == 1){
			printf("\n");
			printf("\n\r ADC_OPEN_MAX_b = %d",ADC_OPEN_MAX_b);
			//printf("\n\r ADC_OPEN_MIN_b = %d",ADC_OPEN_MIN_b);
			printf("\n\r ADC_CLOSE_MAX_b = %d",ADC_CLOSE_MAX_b);
			//printf("\n\r ADC_CLOSE_MIN_b = %d",ADC_CLOSE_MIN_b);
		}else{// if(ADC_Detect_Start_Flag == 2){
			printf("\n");
			printf("\n\r ADC_OPEN_MAX = %d",ADC_OPEN_MAX);
			//printf("\n\r ADC_OPEN_MIN = %d",ADC_OPEN_MIN);
			printf("\n\r ADC_CLOSE_MAX = %d",ADC_CLOSE_MAX);
			//printf("\n\r ADC_CLOSE_MIN = %d",ADC_CLOSE_MIN);
		}

		if(TM_ADC_Relaod > 0){
			printf("\n\r TM_ADC_Relaod = %d",TM_ADC_Relaod);
		}
		//printf("\r\n\nAnti_Weight = %f", Anti_Weight = 1.5);
		if(ADC_Detect_Start_Flag > 0){
			printf("\n\r ADC_Detect_Start_Flag = %d",ADC_Detect_Start_Flag);
		}
		
		if(Flag_IR == TRUE){
			printf("\n\r Flag_IR = %d",Flag_IR);
		}
		if(Flag_SMK == TRUE){
			printf("\n\r Flag_SMK = %d",Flag_SMK);
		}
		//printf("\r\n\nAnti_Weight = %f", Anti_Weight = 1.5);
		
		printf("\n\r TM_Low_Operate = %d",TM_Low_Operate);		
		printf("\n\r *******ST_Low_Operate = %d",ST_Low_Operate);
		printf("\n\r PWM_Duty = %d",PWM_Duty);
		printf("\n\r PWM_Period = %d",PWM_Period);
		printf("\n\r OpEnd_Detect_Start_Flag = %d",OpEnd_Detect_Start_Flag);
		
		printf("\n\r");

		TM_Printf = 10;
	}	
}

static void Operate_Infor_Save(void){
	
	/*
	REC_Operate_Times++;
	TXBuf[0] = REC_Operate_Times;
	TXBuf[1] = REC_Operate_Times >> 8;
	TXBuf[2] = REC_Operate_Times >> 8*2;
	TXBuf[3] = REC_Operate_Times >> 8*3;
		*/
	
	if(TM_Save == 0){
		TXBuf[0] = REC_Operate_Times;
		TXBuf[1] = REC_Operate_Times >> 8;
		TXBuf[2] = REC_Operate_Times >> 8*2;
		TXBuf[3] = REC_Operate_Times >> 8*3;
		
		//
		if(HAL_I2C_Mem_Write(&I2cHandle,(uint16_t)I2C_ADDRESS, 50, I2C_MEMADD_SIZE_8BIT, (uint8_t*)TXBuf, 4, 10000) != HAL_OK){
			if(HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF){
				Error_Handler();
			}
		}
		//HAL_Delay(5);
			
		/*
		if(HAL_I2C_Mem_Read(&I2cHandle,(uint16_t)I2C_ADDRESS, 0, I2C_MEMADD_SIZE_8BIT, (uint8_t*)aRxBuffer, 256, 10000) != HAL_OK){
			if(HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF){
				Error_Handler();
			}
		}
		*/
		
		TM_Save = 2*60*60;
	}
}

//�����}�����ɪ��̤j�P�̤pADC��
static void Operate_ADC_Detect(void){
	uint16_t ADC_Tmp;
	
	//ADC_Detect_Start_Flag
	//0: ���Jbuffers
	//1: �z�LOpend_detect()�ҥ�,����ADC�ܤ�
	//2: Reload�����ܼ�
	
	if(ADC_Detect_Start_Flag == 0){
		ADC_Tmp = ADC_Calculate();
		ADC_OPEN_MAX_b = ADC_Tmp;
		ADC_OPEN_MIN_b = ADC_Tmp;
		ADC_CLOSE_MAX_b = ADC_Tmp;
		ADC_CLOSE_MIN_b = ADC_Tmp;
	
	}else if(ADC_Detect_Start_Flag == 1){	//���찻���}�l�Y����
		if(ST_Door == 1){
			ADC_Tmp = ADC_Calculate();
			if(ADC_Tmp > ADC_OPEN_MAX_b){
				ADC_OPEN_MAX_b = ADC_Tmp;
			}else if(ADC_Tmp < ADC_OPEN_MIN_b){
				ADC_OPEN_MIN_b = ADC_Tmp;
			}
		}else if(ST_Door == 2){
			ADC_Tmp = ADC_Calculate();
			if(ADC_Tmp > ADC_CLOSE_MAX_b){
				ADC_CLOSE_MAX_b = ADC_Tmp;
			}else if(ADC_Tmp < ADC_CLOSE_MIN_b){
				ADC_CLOSE_MIN_b = ADC_Tmp;
			}
		}
		
	}else if(ADC_Detect_Start_Flag == 2){
		printf("\n\n\r TM_ADC_Relaod = %d", TM_ADC_Relaod);
		printf("\n\n\r Anti_Event_buf = %d", Anti_Event_buf);
		printf("\n\n\r Flag_Door_UpLimit = %d", Flag_Door_UpLimit);
		printf("\n\n\r Flag_Door_DownLimit = %d", Flag_Door_DownLimit);
		if(TM_ADC_Relaod == 0 && Anti_Event_buf == 0){
			if(Flag_Door_UpLimit == TRUE){
				ADC_OPEN_MAX= ADC_OPEN_MAX_b;
				ADC_OPEN_MIN = ADC_OPEN_MIN_b;
			}else if(Flag_Door_DownLimit == TRUE){
				ADC_CLOSE_MAX= ADC_CLOSE_MAX_b;
				ADC_CLOSE_MIN = ADC_CLOSE_MIN_b;
			}
		}else{
			ADC_Detect_Start_Flag = 0;
		}
		
		//�����ѦҭȤU��
		if(ADC_OPEN_MAX < 1000){
			ADC_OPEN_MAX = 3700;
		}
		if(ADC_CLOSE_MAX < 1000){
			ADC_CLOSE_MAX = 3700;
		}
		
		ADC_Detect_Start_Flag = 0;
	}
}

static void Cycle_Test(void){
	if(TM_OPEN == 0 && TM_CLOSE == 0 && Wait_flg == TRUE){
		TM_DLY = TM_DLY_Value;
		Wait_flg = FALSE;
	}
	
	if(TM_OPEN == 0 && ACT_Door == 1 && TM_DLY == 0){
		Door_Stop();
		TM_CLOSE = TM_MAX;
		ACT_Door = 2;
		Wait_flg = TRUE;
		REC_Operate_Times++;
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
	Operate_Infor_Save();

	
	if(TM_Printf == 0){
		printf("\n\r==============�`������-���Ascan2===================");			
		Voc_ = ADC_Calculate() *(3.3/4095);		
		printf("\n\r�ثe�q���� = %f V",Voc_);
		printf("\n\r�ݾ��q��   = %f V",Volt_StandBy);
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

int fputc(int ch, FILE *f){
	HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 1000);
	return ch;
}

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
	GPIOF_CLK_ENABLE();
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
		//Avoid both of OPEN and CLOSE have be trigger simultaneously.		
		Door_Stop();
		TM_OPEN = 0;
		TM_CLOSE = 0;
		printf("\n\n\rNG:�B��ɶ��P��>0");

	}else{	
		ST_Door_buf = ST_Door;
		if(Flag_WindowsDoor == FALSE){		//���`�}�����Ҧ�
			if(TM_OPEN > 0){
				Door_Open();
			}else if(TM_CLOSE > 0){	
				Door_Close();
			}else{
				Door_Stop();
				OpEnd_Detect_Start_Flag = FALSE;
				ST_Door = 0;
			}
			
		}else{	//��q�������Ҧ�
			if(TM_OPEN > 0){
				Door_Open();
			}else if(TM_CLOSE > 0){				
				Door_Close();
			}else if(TM_OPEN == 0 && TM_CLOSE == 0){
				OpEnd_Detect_Start_Flag = FALSE;
				Door_Stop();
				if(ST_Door == 1){
					Open_IT = FALSE;
					Close_IT = FALSE;
					Close_IT2 = FALSE;

				}else if(ST_Door == 2){
					if(Open_IT == TRUE){
						ST_Close = 1;
					}else if(ST_Close == 1){	//�Ĥ@�q����
						ST_Close = 2;
					}else if(ST_Close == 2){	//�ĤG�q����
						ST_Close = 1;
					}
					Open_IT = FALSE;
					Close_IT = FALSE;
					Close_IT2 = FALSE;

				}
				ST_Door = 0;
			}
		}
	}
	
	//��찻��
	if(TM_OPEN > 0 || TM_CLOSE > 0){
		OpEnd_Detect();
		OpEnd_Detect_2();
	}else if(TM_CLOSE == 0 && TM_CLOSE_b != 0){
			printf("\n\n\rHelloxxx");
			printf("\n\n\rST_Close = %d", ST_Close);
			if(ST_Close == 1){
				Flag_Door_UpLimit   = FALSE;
				Flag_Door_DownLimit = TRUE;
				ADC_Detect_Start_Flag = 2;		//�B�൲���åB�x�sAD��: Operate_ADC_Detect
			}else if(ST_Close == 2){
				ADC_Detect_Start_Flag = 0;
			}			
	}else{
		ADC_Detect_Start_Flag =0;
	}
	TM_CLOSE_b = TM_CLOSE;
	
	//�}���O������
	if(TM_Light_ON > 0){
		Light_ON();
	}else{
		Light_OFF();
	}
	
	if(TM_OPEN > 0){
		CtrlBox_Light_Up();
	}else if(TM_CLOSE > 0){
		CtrlBox_Light_Down();
	}else{
		CtrlBox_Light_OFF();
	}
	
	if(TM_Buzz > 0){
		if(Buzz_Type == 1){
			Buzz_out(3,3);
		}else if(Buzz_Type == 2){
			Buzz_out(5,5);
		}else{
			//TBD
		}
	}else{
		Buzz_off();
	}
		
}

void Door_manage(void){	
	if(ST_BTN == TRUE){							//����U�F���O
		ST_BTN = FALSE;
		if(Flag_WindowsDoor == FALSE){			//��q������:�L
			switch(ACT_Door){					//���O�P�_
				case 0:							//���O=����
					ST_Door = 0;
					ST_Anti = 0;
					TM_OPEN = 0;
					TM_CLOSE = 0;
					break;
				
				case 1:							//���O=�}��
					ST_Door = 1;
					TM_CLOSE = 0;
					TM_OPEN = TM_MAX;
					TM_AntiDly = Time_AntiDly;
					TM_EndDetec = 10;
					break;
				
				case 2:							//���O=����
					if(Anti_Event == 2){ 		//����������: break
						break;
					}
					ST_Door = 2;
					TM_OPEN = 0;
					TM_CLOSE = TM_MAX;
					TM_AntiDly = Time_AntiDly;
					TM_EndDetec = 10;
					AClose_Flg = FALSE;
					//TM_Auto_Close = 0;
					break;
				
				default:
					ST_Door = 0;
					TM_OPEN = 0;
					TM_CLOSE = 0;
			}
		}else if(Flag_WindowsDoor == TRUE){	//��q������:��
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
					ST_Anti = 0;
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
					TM_AntiDly = Time_AntiDly;
					TM_EndDetec = 10;
					break;
			//-------------------���O=�}�� End----------------//
			//-------------------���O=����--------------------//
				case 2:
					if(Anti_Event == 2){ 		//����������: break
						break;
					}
					ST_Door = 2;
					TM_OPEN =0;
					if(Open_IT == TRUE){
							TM_CLOSE = TM_MAX;
					}else if(ST_Close == 1){
							if(Close_IT == FALSE){
								TM_CLOSE = TM_WindowsDoor_ClosePart1;
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
					TM_AntiDly = Time_AntiDly;
					TM_EndDetec = 10;
					AClose_Flg = FALSE;
					break;
			//-------------------���O=���� End-----------------//
			// ----- Else ----- //
				default:
					ST_Door = 0;
					TM_OPEN = 0;
					TM_CLOSE = 0;
					//break;
			}
		}
		if(TM_OPEN > 0 || TM_CLOSE > 0){
			TM_ADC_Relaod = 50;
			TM_Low_Operate = 0;
		}
	}
}	

static void Auto_Close_CTRL(void){
	//�۰������\��
	if(Flag_SMK == FALSE){
		// �}���᩵��ɶ��g�L�۰�����
		if(Flag_AutoClose == 1){
			if(AClose_Flg == TRUE && TM_Auto_Close == 0){		//�۰������˼Ʈɶ�����
				printf("\n\r�۰��������ݮɶ���F");
				printf("\n\r�����ɶ��]��");
				AClose_Flg = FALSE;									//�۰������X��:OFF
				TM_CLOSE = TM_MAX;									//�����ɶ��]�w
			}
		}else if(Flag_AutoClose == 2){							//�۰������\��: ON
			if(Flag_WindowsDoor == FALSE){								//��q�����\��:�L
				if(	ST_Door == 0 && ST_Door_buf == 1){					//�P�_������e�����A�O�_���}��
					printf("\n\r�۰������X�� & ���ݮɶ��]��");
					TM_Auto_Close = Time_Auto_Close;					//�]�w�۰������˼Ʈɶ�
					AClose_Flg = TRUE;									//�۰������X��:ON
				}else if(AClose_Flg == TRUE && TM_Auto_Close == 0){		//�۰������˼Ʈɶ�����
					printf("\n\r�۰��������ݮɶ���F");
					printf("\n\r�����ɶ��]��");
					AClose_Flg = FALSE;									//�۰������X��:OFF
					TM_CLOSE = TM_MAX;									//�����ɶ��]�w
				}
			}else{
				//empty
			}
		}else{
			//Empty
		}
	}
}

static void Light_CTRL(void){
	//�]�w�}���ɪ��ө�
	//�ʧ@�᩵��TM_Light�ɶ���A�����ө�
	if(TM_OPEN > 0){
		TM_Light_ON = TM_OPEN + Time_Light;
	}
}

static void IR_CTRL(void){
	//����~�u����Ĳ�o��30����ӺX��
	if(TM_IR_Lock == 0 && Flag_IR == TRUE){
		Flag_IR = FALSE;
	}
}

//******************Relay control******************//
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
	
	//MOSFET switch ON
	//HAL_GPIO_WritePin(PORT_Motor_MOS, MOS_ACT, GPIO_PIN_RESET);
	if (HAL_TIM_Base_Start_IT(&TimHandle17) != HAL_OK){
		/* Starting Error */
		Error_Handler();
	}	
}

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
	
	//MOSFET switch ON
	//HAL_GPIO_WritePin(PORT_Motor_MOS, MOS_ACT, GPIO_PIN_RESET);
	if (HAL_TIM_Base_Start_IT(&TimHandle17) != HAL_OK){
		/* Starting Error */
		Error_Handler();
	}	
}

void Door_Stop(void){
//	printf("\n\r----STOP_Relay");
	
	if (HAL_TIM_Base_Stop_IT(&TimHandle17) != HAL_OK){
		/* Starting Error */
		Error_Handler();
	}	
	HAL_GPIO_WritePin(PORT_Motor_MOS, MOS_ACT, GPIO_PIN_SET);			
	
	Delay_ms(RLY_Delay_ms);
	HAL_GPIO_WritePin(PORT_Motor_Out, RLY_ACT, GPIO_PIN_RESET);
	Delay_ms(RLY_Delay_ms);
	HAL_GPIO_WritePin(PORT_Motor_Out, RLY_DIR, GPIO_PIN_RESET);		
	
	HAL_GPIO_WritePin(PORT_TEST, TEST_PIN, GPIO_PIN_SET);

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

//�L���������찻��
static void OpEnd_Detect(void){
	if(TM_EndDetec == 0 && Flag_WindowsDoor == FALSE){
		if(OpEnd_Detect_Start_Flag == FALSE){                  //����
			TM_DoorOperateDly = 5;                             //Delay 0.5 second��}�l�T�{�O�_�쭭��
			OpEnd_Detect_Start_Flag = TRUE;
			ADC_Detect_Start_Flag = 1;		                   //ADC�B����^���}�l:Operate_ADC_Detect
		}else{
			Voc = ADC_Calculate() *(3.3/4095);		
			if(Voc <= Volt_StandBy && TM_DoorOperateDly == 0){
				printf("\n\n\r�����-����B��!\n\n");
				
				//�P�_�O�_�}�����,�åB�]�w�ө��ɶ�
				if(TM_OPEN > 0){
					//�]�w�ө�����ɶ�
					TM_Light_ON = Time_Light;
					if(Flag_AutoClose == 1){
						TM_Auto_Close = Time_Auto_Close;					//�]�w�۰������˼Ʈɶ�
						AClose_Flg = TRUE;									//�۰������X��:ON
					}
					
					//�Y�O��������Ĳ�o,��_���ݾ����`�B��
					Anti_Event_buf = Anti_Event;
					if(Anti_Event == 2){
						Anti_Event = 0;
					}
				}
								
				//����X�а���
				if(TM_OPEN > 0){
					Flag_Door_UpLimit   = TRUE;
					Flag_Door_DownLimit = FALSE;
				}else if(TM_CLOSE > 0){
					Flag_Door_UpLimit   = FALSE;
					Flag_Door_DownLimit = TRUE;
				}
				
				//�B�স��
				if(TM_OPEN > 0){
					REC_Operate_Times++;
				}
				
				if(Flag_WindowsDoor == TRUE){
					if(TM_OPEN > 0){
						ST_Close = 1;
					}
				}
				
				TM_OPEN = 0;
				TM_CLOSE = 0;
				ST_Anti = 0;
				OpEnd_Detect_Start_Flag = FALSE;		
				ADC_Detect_Start_Flag = 2;		//�B�൲���åB�x�sAD��: Operate_ADC_Detect
			}
		}
	}
}

//�����������찻��
static void OpEnd_Detect_2(void){
	if(TM_EndDetec == 0 && Flag_WindowsDoor == TRUE){
		if(OpEnd_Detect_Start_Flag == FALSE){                  //����
			TM_DoorOperateDly = 5;                             //Delay 0.5 second��}�l�T�{�O�_�쭭��
			OpEnd_Detect_Start_Flag = TRUE;
			ADC_Detect_Start_Flag = 1;		                   //ADC�B����^���}�l:Operate_ADC_Detect
		}else if(TM_OPEN > 0){
			Voc = ADC_Calculate() *(3.3/4095);		
			if(Voc <= Volt_StandBy && TM_DoorOperateDly == 0){
				printf("\n\n\r�����-����B��!\n\n");
				
				//�P�_�O�_�}�����,�åB�]�w�ө��ɶ�
				if(TM_OPEN > 0){
					//�]�w�ө�����ɶ�
					TM_Light_ON = Time_Light;
					if(Flag_AutoClose == 1){
						TM_Auto_Close = Time_Auto_Close;					//�]�w�۰������˼Ʈɶ�
						AClose_Flg = TRUE;									//�۰������X��:ON
					}
					
					//�Y�O��������Ĳ�o,��_���ݾ����`�B��
					Anti_Event_buf = Anti_Event;
					if(Anti_Event == 2){
						Anti_Event = 0;
					}
				}
								
				//����X�а���
				Flag_Door_UpLimit   = TRUE;
				Flag_Door_DownLimit = FALSE;
				
				//�B�স��
				REC_Operate_Times++;
				
				ST_Close = 1;

				TM_OPEN = 0;
				TM_CLOSE = 0;
				ST_Anti = 0;
				OpEnd_Detect_Start_Flag = FALSE;		
				ADC_Detect_Start_Flag = 2;		//�B�൲���åB�x�sAD��: Operate_ADC_Detect
			}
		}else if(TM_CLOSE > 0){			
			Voc = ADC_Calculate() *(3.3/4095);		
			if(Voc <= Volt_StandBy && TM_DoorOperateDly == 0){
				printf("\n\n\r�����-����B��!\n\n");
												
				//����X�а���
				Flag_Door_UpLimit   = FALSE;
				Flag_Door_DownLimit = TRUE;
				
				TM_OPEN = 0;
				TM_CLOSE = 0;
				ST_Anti = 0;
				OpEnd_Detect_Start_Flag = FALSE;
				
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
	
	
	GPIO_InitStruct.Pin = TEST_PIN;
	HAL_GPIO_Init(PORT_TEST, &GPIO_InitStruct);

	
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
	EXTI_CTRL_GPIO2_CLK_ENABLE();
	EXTI_CTRL_LOCK_CLK_ENABLE();

  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;//GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = EXTI_CTRL_PIN;
  HAL_GPIO_Init(EXTI_CTRL_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = RM_LOCK;
  HAL_GPIO_Init(PORT_LOCK, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = EXTI_CTRL_PIN_2;
  HAL_GPIO_Init(EXTI_CTRL_PORT_2, &GPIO_InitStructure);
	
  /* Enable and set EXTI line 4_15 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
  //HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

//�~�����_:���䰻��
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	printf("\n\r������O: ");
	CNT_Jog_Press = 0;
	CNT_LOCK_Press = 0;
	Flag_JOG = FALSE;
	ST_Door_buf = ST_Door;
	
	switch(GPIO_Pin){
		case W_STOP:
			if(Flag_SMK  == TRUE)	break;
			if(Flag_LOCK == TRUE){			//��q�\��ON: ���ʧ@
				Buzz_Type = 2;	    // ���ﾹON/OFF 1��
				TM_Buzz = 2;	    // ���ﾹ�B�@ 4 sec.
				break;
			}
			ST_BTN = TRUE;
			ACT_Door = 0;
			ST_Anti = 0;
			ADC_Detect_Start_Flag = 0;
			
			//�۰��������ݮɶ����J
			if(AClose_Flg == TRUE){
				printf("\n\r���s���J���������ɶ�: %d ms", TM_Auto_Close);
				TM_Auto_Close = Time_Auto_Close;					//�]�w�۰������˼Ʈɶ�
			}
			
			//�Ѱ����~�u��w
			if(Flag_IR == TRUE){	
				Flag_IR = FALSE;
				TM_IR_Lock = 0;
			}
			
			Anti_Event = 0;
			printf("\n\rSTOP!\n");
			break;
		
		case W_OPEN:
			if(Flag_SMK  == TRUE)	break;
			if(Flag_LOCK == TRUE){			//��q�\��ON: ���ʧ@
				Buzz_Type = 2;	    // ���ﾹON/OFF 1��
				TM_Buzz = 2;	    // ���ﾹ�B�@ 4 sec.
				break;
			}
			
			//�T�ʥ\��
			if(Flag_Func_JOG == TRUE){
				//�T�ʰ���
				ST_Press = HAL_GPIO_ReadPin(PORT_OPEN, W_OPEN);
				while(ST_Press == 0){
					CNT_Jog_Press++;
					printf("\n\rOPEN key.....press\n");
					ST_Press = HAL_GPIO_ReadPin(PORT_OPEN, W_OPEN);
					if(CNT_Jog_Press > Times_JOG){		// ������U�ëO��: 50 times
						if(Flag_JOG == FALSE){
							//TM_DoorOperateDly = 5;
							Door_Open();
						}
						Flag_JOG = TRUE;
						printf("\n\rJOG Mode:OPEN...\n");
						Door_Open();
						Light_ON();
						
					}
				}
				printf("\n\rContinue = %d\n",CNT_Jog_Press);
				if(Flag_JOG == TRUE){
					Door_Stop();
					TM_Light_ON = Time_Light;
					printf("\n\rDoor Stop.....\n");
					break;
				}
			}
			
			//�ϦV���O����
			if(ST_Door_buf == 2){
				printf("\n\r--------�ߧY����--------");
				Door_Stop();
				Delay_ms(100);
				OpEnd_Detect_Start_Flag = TRUE;				
			}
			//***����P�w END***//
			
			ST_BTN = TRUE;
			ACT_Door = 1;
			printf("\n\rOPEN!\n");
			break;

		case W_CLOSE:			
			if(Flag_SMK   == TRUE)	break;		//�����P����Ĳ�o
			if(Anti_Event == 2)     break;      //����������
			if(Flag_LOCK  == TRUE){			//��q�\��ON: ���ʧ@
				Buzz_Type = 2;	    // ���ﾹON/OFF 1��
				TM_Buzz = 2;	    // ���ﾹ�B�@ 4 sec.
				break;
			}
			if(Flag_IR    == TRUE){				//���~�uĲ�o: ���ʧ@
				Buzz_Type = 1;	//���ﾹON/OFF 0.5��
				TM_Buzz   = 10;	//���ﾹ�B�@ 10 sec.
				break;
			}
			
			if(Flag_Func_JOG == TRUE){
			// JOG detect
				ST_Press = HAL_GPIO_ReadPin(PORT_CLOSE, W_CLOSE);
				while(ST_Press == 0){
					CNT_Jog_Press++;
					printf("\n\rCLOSE key.....press\n");
					ST_Press = HAL_GPIO_ReadPin(PORT_CLOSE, W_CLOSE);
					if(CNT_Jog_Press > Times_JOG){		// 50 times
						Flag_JOG = TRUE;
						printf("\n\rJOG Mode:CLOSE...\n");
						Door_Close();
						//Light_ON();
						//OpEnd_Detect();
						//...�ثe����������_���\��,
						//�p�G����,�A�NOPEN����챱��[�J
					}
				}
				printf("\n\rContinue = %d\n",CNT_Jog_Press);
				if(Flag_JOG == TRUE){
					Door_Stop();
					printf("\n\rDoor Stop.....\n");
					break;
				}
				// JOG end
			}
			
			//***����P�w***//
			if(ST_Door_buf == 1){
				printf("\n\r--------�ߧY����--------");
				Door_Stop();
				Delay_ms(100);
				OpEnd_Detect_Start_Flag = TRUE;
			}
			//***����P�w END***//

			ST_BTN = TRUE;
			ACT_Door = 2;
			printf("\n\rClose!\n");
			break;
		
		case RM_LOCK:
			printf("\n\rLock!\n");
			if(Flag_Remote_Lock == TRUE){	//��q�\��}��?
				
				ST_Press = HAL_GPIO_ReadPin(PORT_LOCK, RM_LOCK);
				while(ST_Press == 0){
					printf("\n\rLOCK key.....press %d\n", CNT_LOCK_Press);
					CNT_LOCK_Press++;
					ST_Press = HAL_GPIO_ReadPin(PORT_LOCK, RM_LOCK);
					if(CNT_LOCK_Press > Times_Remote_Lock){		// 50 times
						if(Flag_LOCK == TRUE){
							Flag_LOCK = FALSE;
							ST_Press = 1;
							printf("\n\rUNLOCK~~~~~~!\n");
						}else{
							Flag_LOCK = TRUE;
							ST_Press = 1;
							printf("\n\rLOCK~~~~~~!\n");
						}
					}
				}
				
			}
			break;
		
		case W_IR:	//���~�u����
			if(ST_Door == 2){
				Flag_IR = TRUE;
				Door_Stop();
				Delay_ms(100);
				ST_BTN = TRUE;
				ACT_Door = 1;	 // �}��
				TM_IR_Lock = 30; // ��w�ɶ� 30 sec.
				Buzz_Type = 1;	 // ���ﾹON/OFF 0.5��
				TM_Buzz = 30;	 // ���ﾹ�B�@ 30 sec.
			}
			break;
		
		case W_SMK:
			Flag_SMK = TRUE;
			Door_Stop();
			Delay_ms(100);
			ST_BTN = TRUE;
			ACT_Door = 1;	 //�}��
			TM_Buzz = TM_MAX;

			break;
		
		default:
				break;
	}

}

//�����P������
static void SMK_CTRL(void){
	uint8_t SMK_tmp;
	
	//����������QĲ�o��,�}�lpolling SMK pin,
	//����ĵ���Ѱ��~�Ѱ����������X��.
	if(Flag_SMK == TRUE){
		SMK_tmp = HAL_GPIO_ReadPin(PORT_SMK, W_SMK);
		if(SMK_tmp == TRUE){
			Flag_SMK = FALSE;
			TM_Buzz = 0;
		}else{
			Buzz_Type = 1;
			TM_Buzz = TM_OPEN;
		}
	}
}


//	TIM14 handle
//	For PWM output
void HAL_TIM17_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{	
	if(Flag_Low_Operate == TRUE){
		if(TM_Low_Operate < 20){
			PWM_Duty = 1;
			PWM_Period = 3;
			ST_Low_Operate = 1;
		}else if(TM_Low_Operate < 50){
			PWM_Duty = 99;
			PWM_Period = 100;
			ST_Low_Operate = 2;
		}else{
			PWM_Duty = 1;
			PWM_Period = 3;
			ST_Low_Operate = 3;
		}
		
		if(ST_Low_Operate == 2){
			Volt_StandBy = ADC_Calculate() *(3.3/4095) * 1.3;
		}else{
			Volt_StandBy = ADC_Calculate() *(3.3/4095) * 1.1;
		}
		
	}else{
		switch(PWM_Grade){
			case 1:
				PWM_Duty = 1;
				PWM_Period = 2;
				break;
				
			case 2:
				PWM_Duty = 2;
				PWM_Period = 3;
				break;
				
			case 3:
				PWM_Duty = 3;
				PWM_Period = 4;
				break;
				
			case 4:
				PWM_Duty = 4;
				PWM_Period = 5;
				break;
				
			case 5:
				PWM_Duty = 6;
				PWM_Period = 7;
				break;
				
			case 6:
				PWM_Duty = 99;
				PWM_Period = 100;
				break;
				
			default:
				PWM_Duty = 1;
				PWM_Period = 2;
				break;
		}
	}
	
	PWM_Count++;	
	if(PWM_Count == PWM_Period){
		PWM_Count = 0;
	}
	
	if(PWM_Count < PWM_Duty){
		HAL_GPIO_WritePin(PORT_TEST, TEST_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PORT_Motor_MOS, MOS_ACT, GPIO_PIN_RESET);
	}else{
		HAL_GPIO_WritePin(PORT_TEST, TEST_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(PORT_Motor_MOS, MOS_ACT, GPIO_PIN_SET);
	}
	//HAL_GPIO_WritePin(PORT_TEST, TEST_PIN, GPIO_PIN_RESET);
  //HAL_GPIO_TogglePin(PORT_Motor_MOS, MOS_ACT);

}


//	TIM16 handle
//	For TIMER Down-count
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	Tim_cnt_10ms++;
	Tim_cnt_100ms++;
	Tim_cnt_1s++;
	
	// 0.01 sec
	if(Tim_cnt_10ms == 10){
		Tim_cnt_10ms = 0;
	}
	
	// 0.1 sec
	if(Tim_cnt_100ms == 100){
		TM_OPEN 	      = TIMDEC(TM_OPEN);
		TM_CLOSE 	      = TIMDEC(TM_CLOSE);
		TM_AntiDly 	      = TIMDEC(TM_AntiDly);
		TM_AntiDly2 	  = TIMDEC(TM_AntiDly2);
		TM_AntiDly3 	  = TIMDEC(TM_AntiDly3);
		TM_EndDetec 	  = TIMDEC(TM_EndDetec);
		TM_DoorOperateDly = TIMDEC(TM_DoorOperateDly);
		TM_Printf 	      = TIMDEC(TM_Printf);
		TM_DLY 	          = TIMDEC(TM_DLY);
		TM_Light_ON       = TIMDEC(TM_Light_ON);
		TM_Auto_Close     = TIMDEC(TM_Auto_Close);
		TM_Anti_Occur     = TIMDEC(TM_Anti_Occur);
		TM_Buzz_ON        = TIMDEC(TM_Buzz_ON);
		TM_Buzz_OFF        = TIMDEC(TM_Buzz_OFF);
		TM_ADC_Relaod        = TIMDEC(TM_ADC_Relaod);
		TM_Low_Operate    = TIMINC(TM_Low_Operate);
		Tim_cnt_100ms = 0;

	}
	
	// 1 sec
	if(Tim_cnt_1s == 1000){
		Tim_cnt_1s = 0;
		TM_IR_Lock 	      = TIMDEC(TM_IR_Lock);
		TM_Save 	      	= TIMDEC(TM_Save);
		TM_Buzz 	      	= TIMDEC(TM_Buzz);
	}

}

//1s timer
static void TIM16_Config(void)
{	
	uwPrescalerValue = (uint32_t)(SystemCoreClock / 10000) - 1;

	TimHandle.Instance = TIM16;


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

static void TIM17_Config(void)
{	
	uwPrescalerValue = (uint32_t)(SystemCoreClock / 1000000) - 1;

	TimHandle17.Instance = TIM17;

	TimHandle17.Init.Period            = (1*10) - 1;   // 1*10ms
	TimHandle17.Init.Prescaler         = uwPrescalerValue;
	TimHandle17.Init.ClockDivision     = 0;
	TimHandle17.Init.CounterMode       = TIM_COUNTERMODE_DOWN;
	TimHandle17.Init.RepetitionCounter = 0;
	TimHandle17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  
	if (HAL_TIM_Base_Init(&TimHandle17) != HAL_OK)
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

static void I2C_Config(void){
	/*##-1- Configure the I2C peripheral ######################################*/
  I2cHandle.Instance             = I2Cx;
  I2cHandle.Init.Timing          = I2C_TIMING;
  I2cHandle.Init.OwnAddress1     = I2C_ADDRESS;
  I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2cHandle.Init.OwnAddress2     = 0xFF;
  I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

  if(HAL_I2C_Init(&I2cHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Enable the Analog I2C Filter */
  HAL_I2CEx_ConfigAnalogFilter(&I2cHandle,I2C_ANALOGFILTER_ENABLE);
	

}

static void Uart_Config(void){
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

static void Parameter_Load(void){
	uint8_t i;
	
	if(HAL_I2C_Mem_Read(&I2cHandle,(uint16_t)I2C_ADDRESS, 0, I2C_MEMADD_SIZE_8BIT, (uint8_t*)aRxBuffer, 256, 10000) != HAL_OK){
		if(HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF){
			Error_Handler();
		}
	}
	
	EE_Default = HAL_GPIO_ReadPin(PORT_EE_SEL, EEPROM_SEL);
	
	if(EE_Default == TRUE){
		//Default parameter
		Flag_CycleTest       = FALSE;   //�`������(���ɴ���)
		Flag_WindowsDoor     = TRUE;   //�������\��
		Flag_AntiPress       = TRUE;    //�����\��
		Flag_AutoClose       = FALSE;   //�۰������\��
		Flag_Func_JOG        = FALSE;   //�T�ʥ\��
		Flag_Motor_Direction = TRUE;   //���F�B���V
		Flag_Remote_Lock     = TRUE;   //��q�\��
		Flag_Rate_Regulate   = FALSE;   //�����ճt
		Flag_Buzzer          = TRUE;    //���ﾹ
		Flag_Light           = FALSE;    //�۰ʷө�
		Flag_Low_Operate     = FALSE;  //�w�_�B & �w����
		
		TM_DLY_Value              = 300;   //�`�����ն��j�ɶ�
		TM_WindowsDoor_ClosePart1 = 70;   //������_�Ĥ@�q�����ɶ�
		TM_MAX                    = 600;   //�}�����̪��B��ɶ�
		Time_Auto_Close           = 100;   //�۰���������ɶ�
		Time_Light                = 100;   //�ө��B��ɶ�

		Volt_StandBy = 0; //�}���۰ʨM�w�ݾ���
		//Volt_StandBy = 0.3;
		Anti_Weight_Open  = 0.01;   //�����v��: �}��
		Anti_Weight_Close = 0.01;   //�����v��: ����
		
		Times_JOG        = 50;   //�T�ʧP�w����
		Times_Remote_Lock = 75;   //��q���ߦ���
		
		PWM_Grade        = 6;   //�K���t��
		Auto_Close_Mode  = 1;	 //�۰������Ҧ��]�w
		

	}else{
		//******Parameter form EEPROM*****//
		//Funtion ON/OFF
		Flag_CycleTest       = aRxBuffer[10];   //�`������(���ɴ���)
		Flag_WindowsDoor     = aRxBuffer[11];   //�������\��
		Flag_AntiPress       = aRxBuffer[12];   //�����\��
		Flag_AutoClose       = aRxBuffer[13];   //�۰������\��
		Flag_Func_JOG        = aRxBuffer[14];   //�T�ʥ\��
		Flag_Motor_Direction = aRxBuffer[15];   //���F�B���V
		Flag_Remote_Lock     = aRxBuffer[16];   //��q�\��
		Flag_Rate_Regulate   = aRxBuffer[17];   //�����ճt
		Flag_Buzzer          = aRxBuffer[18];   //���ﾹ
		Flag_Light           = aRxBuffer[19];    //�۰ʷө�
		Flag_Low_Operate     = aRxBuffer[20];  //�w�_�B & �w����
		
		TM_DLY_Value              = (uint16_t)aRxBuffer[30] | (uint16_t)aRxBuffer[31]<<8;   //�`�����ն��j�ɶ�
		TM_WindowsDoor_ClosePart1 = (uint16_t)aRxBuffer[32] | (uint16_t)aRxBuffer[33]<<8;   //������_�Ĥ@�q�����ɶ�
		TM_MAX                    = (uint16_t)aRxBuffer[34] | (uint16_t)aRxBuffer[35]<<8;   //�}�����̪��B��ɶ�
		Time_Auto_Close           = (uint16_t)aRxBuffer[36] | (uint16_t)aRxBuffer[37]<<8;   //�۰���������ɶ�
		Time_Light                = (uint16_t)aRxBuffer[38] | (uint16_t)aRxBuffer[39]<<8;   //�ө��B��ɶ�

		Volt_StandBy      = (float)aRxBuffer[40]/10;   //�ݾ��q��for���P�w�ϥ�
		Anti_Weight_Open  = (float)aRxBuffer[41]/100;   //�����v��: �}��
		Anti_Weight_Close = (float)aRxBuffer[42]/100;   //�����v��: ����
		
		Times_JOG         = aRxBuffer[43];   //�T�ʧP�w����
		Times_Remote_Lock = aRxBuffer[44];   //��q���ߦ���
		
		PWM_Grade      = aRxBuffer[45];   //�K���t��
		Auto_Close_Mode   = aRxBuffer[46];	 //�۰������Ҧ��]�w
		
	}
	
	//�����B�榸��
	REC_Operate_Times = 0;
	//REC_Operate_Times = (uint32_t)aRxBuffer[30] | (uint32_t)aRxBuffer[31]<<8 | (uint32_t)aRxBuffer[32]<<16 | (uint32_t)aRxBuffer[33]<<24;
	for(i=0;i<4;i++){
		REC_Operate_Times = REC_Operate_Times | (uint32_t)aRxBuffer[50+i]<<(8*i);
	}
}

static void Anti_Pressure_5(void){
	uint16_t TM_Buf;
	uint16_t ADC_Tmp;
	uint16_t ADC_Buf;
	
	if(Flag_AntiPress == TRUE &&
		 (ADC_OPEN_MAX != 0 && ADC_CLOSE_MAX != 0)){
		
		//printf("\r\nAnti_ST = %d",ST_Anti);
		
		switch(ST_Anti){
			case 0:
				//�ݾ�
				//���ݰ�������ɶ�>0�Y�}�l����
				if(TM_AntiDly > 0){		
					ST_Anti = 1;
					Anti_Event = 0;
					Times_OverADC = 0;
				}
				
				//�}��������, ���ݫ���ɶ��g�L
				if(Anti_Event == 1 && TM_Anti_Occur == 0){
					Anti_Event =0;
				}
				
				break;
			
			case 1:		// �L���q�y���v�]�w
				if(TM_AntiDly > 0){
					// Empty
					ADC_Tmp = 0;
				}else if(TM_AntiDly == 0){
					//�ھڨ����Ѽ��v��,�]�w�����ʧ@��
					if(TM_OPEN > 0){
						//Anti_Weight = 1 + (Anti_Weight_Open / 10);
						Anti_Weight = 1 + Anti_Weight_Open;
						ADC_Anti_Max = ADC_OPEN_MAX * Anti_Weight;
						ST_Anti = 2;
					}else if(TM_CLOSE > 0){
						//Anti_Weight = 1 + (Anti_Weight_Close / 10);
						Anti_Weight = 1 + Anti_Weight_Close;
						ADC_Anti_Max = ADC_CLOSE_MAX * Anti_Weight;
						ST_Anti = 3;
					}
					
					TM_AntiDly2 = 10; //Delay 0.1 second��}�l����
				}
				break;
			
			case 2:	//�}����������
				if(TM_AntiDly2 == 0){

					ADC_Buf = ADC_Calculate();	//Ū����eAD��

					//�P�_��eAD��	
					if(ADC_Buf >= ADC_Anti_Max){
						Times_OverADC++;				
					}else{
						Times_OverADC = 0;
					}
					
					//printf("\r\n\n ADC_Buf = %d, ADC_Anti_Max = %d",ADC_Buf,ADC_Anti_Max);
					
					if(Times_OverADC >= Times_OverADC_Target){
						ST_Anti = 4;
						Anti_Event = 1; 	// OPEN����ON
						TM_Anti_Occur = 100; //�]�w100��p��
						
						//����
						TM_OPEN = 0;
						TM_CLOSE  = 0;
						
						TM_AntiDly3 = 0; 	//�B�ఱ��,�K����,���ݵ��ݮɶ�
					}else{
						//ST_Anti = 2;
						Anti_Event = 0; 	// ���`�B��
						TM_AntiDly2 = 3;
					}
				}
				break;
			
			case 3:	//������������
				if(TM_AntiDly2 == 0){

					ADC_Buf = ADC_Calculate();	//Ū����eADC��

					
					//�p��B��q���ܤ�
										
					if(ADC_Buf >= ADC_Anti_Max){
						Times_OverADC++;				
					}else{
						Times_OverADC = 0;
					}
									
					if(Times_OverADC == Times_OverADC_Target){
						ST_Anti = 4;
						Anti_Event = 2; 	// CLOSE����ON
						//����
						TM_OPEN = 0;
						TM_CLOSE = 0;
						TM_AntiDly3 = 1;	//���y0.1���,����}��
					}else{
						//ST_Anti = 2;
						Anti_Event = 0; 	// ���`�B��
						TM_AntiDly2 = 1;
					}
				}
				break;
			
			case 4:
			//�O�@�ʧ@
				if(TM_AntiDly3 == 0){
					if(Anti_Event == 1){
						//����, �]�w���ﾹ
					}else if(Anti_Event == 2){
						//����}��,�]�w���ﾹ
						TM_OPEN = TM_MAX;
						TM_CLOSE = 0;
					}else{
						//empty
					}
					//�^�_�����ݾ�
					ST_Anti = 0;
				}
				break;
			
			default:
				// Empty
				break;
		}

		//printf("\r\nADC_Anti_Max = %d",ADC_Anti_Max);

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
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	GPIO_InitStruct.Pin = Buzzer;
	HAL_GPIO_Init(PORT_Buzzer, &GPIO_InitStruct);

	//buzz config...end
}

static void ControlBox_Config(void){
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	GPIO_InitStruct.Pin = CtrlBox_1 | CtrlBox_2;
	HAL_GPIO_Init(PORT_CtrlBox, &GPIO_InitStruct);

}

static void CtrlBox_Light_Up(void){
	HAL_GPIO_WritePin(PORT_CtrlBox, CtrlBox_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PORT_CtrlBox, CtrlBox_2, GPIO_PIN_SET);
}

static void CtrlBox_Light_Down(void){
	HAL_GPIO_WritePin(PORT_CtrlBox, CtrlBox_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PORT_CtrlBox, CtrlBox_2, GPIO_PIN_RESET);
}

static void CtrlBox_Light_OFF(void){
	HAL_GPIO_WritePin(PORT_CtrlBox, CtrlBox_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PORT_CtrlBox, CtrlBox_2, GPIO_PIN_RESET);
}



uint16_t* BubbleSort(uint16_t arr[], uint16_t len){
	int i, j, temp;
	for (i = 0; i < len - 1; ++i){          //�`��N-1��
		for (j = 0; j < len - 1 - i; ++j){  //�C���`���n���������
			if (arr[j] > arr[j + 1])       //��j�p��洫
			{
				temp = arr[j];
				arr[j] = arr[j + 1];
				arr[j + 1] = temp;
			}
		}
	}
	
	return arr;
}

static uint16_t	TIMDEC(uint16_t TIMB){
	uint16_t TIM_Buf = TIMB;
	if(TIMB == 0) return TIMB;
	
	TIM_Buf--;
	
	return TIM_Buf;
}

static uint16_t	TIMINC(uint16_t TIMB){
	uint32_t TIM_Buf = TIMB;
	if(TIMB == 0xFFFF) return TIMB;
	
	TIM_Buf++;
	
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

 //Name: Buzz_on
 //Description: Buzzer ON
static void Buzz_on(void){
	HAL_GPIO_WritePin(PORT_Buzzer, Buzzer, GPIO_PIN_SET);
}

//Name: Buzz_off
//Description: Buzzer OFF
static void Buzz_off(void){
	HAL_GPIO_WritePin(PORT_Buzzer, Buzzer, GPIO_PIN_RESET);
}

static void Buzz_out(uint16_t ON_Time, uint16_t OFF_Time){
	uint8_t ST_Buzz;
	
	ST_Buzz = HAL_GPIO_ReadPin(PORT_Buzzer, Buzzer);
	
	printf("\n\n\rBuzz_type = %d", Buzz_Type);
	
	if(ST_Buzz == 0 && TM_Buzz_OFF == 0){
		TM_Buzz_ON = ON_Time;
		Buzz_on();
	}else if(ST_Buzz == 1 && TM_Buzz_ON == 0){
		TM_Buzz_OFF = OFF_Time;
		Buzz_off();
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
