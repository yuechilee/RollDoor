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
#define I2C_ADDRESS	0xA0	//0x30F
#define I2C_TIMING	0x00A51314

//Status Relay
#define	None         0
#define	SR_Uplimit   1
#define	SR_Downlimit 2
#define	SR_MidStop   3
#define	SR_Open      4
#define	SR_Down      5
#define	SR_CmdOpen   6
#define	SR_CmdStop   7
#define	SR_CmdClose  8
#define	SR_CmdLock   9

//Bit process
#define	BIT0	0x01
#define	BIT1	0x02
#define	BIT2	0x04
#define	BIT3	0x08
#define	BIT4	0x10
#define	BIT5	0x20
#define	BIT6	0x40
#define	BIT7	0x80

#define	BSET(x,y)	(x |= y)
#define	BTST(x,y)	(x & y)
#define	BCLR(x,y)	(x &= (y ^ 0xFF))
#define	BNOT(x,y)	(x ^= y)

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
uint8_t VER1, VER2;
uint8_t	VER3 = 18;
uint8_t Maintain_Year; 	
uint8_t Maintain_Month; 	
uint8_t Maintain_Day; 	
uint8_t Warranty_Year; 	
uint8_t Warranty_Month; 	
uint8_t Warranty_Day; 	
uint8_t PN1; 	
uint8_t PN2; 	
uint8_t PN3; 	
uint8_t PN4; 	
uint8_t PN5; 	
uint8_t PN6; 	
uint8_t EE_Default = FALSE;
uint8_t Flag_WindowsDoor; 		//���������:	
uint8_t Flag_CycleTest;             //�`������(���ɴ���)
uint8_t Flag_AntiPress;			//�����\��
uint8_t Flag_AntiPress_Open;	//�����\��(�}��)
uint8_t Flag_AntiPress_Close;	//�����\��(����)
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

uint8_t Anti_Weight_Open_select;					//�����v��(�i�p��):�}��(�V�p�V�F��),��ĳ>1
float Anti_Weight_Open;					//�����v��(�i�p��):�}��(�V�p�V�F��),��ĳ>1
uint8_t Anti_Weight_Close_select;				//�����v��(�i�p��):����(�V�p�V�F��),��ĳ>1
float Anti_Weight_Close;				//�����v��(�i�p��):����(�V�p�V�F��),��ĳ>1
float Volt_StandBy_32f, Volt_StandBy_b_32f;				//�ݾ��q��(��0���즸�Ұʰ���),��ĳ��0.3~0.5
uint16_t ADC_StandBy_16u,ADC_StandBy_b_16u,ADC_Promp_16u;
uint16_t ADC_TMP_16u,ADC_OpEnd_16u;
float iWeight_Vstb_8u;

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
uint8_t Flag_No_VSB; //The default stand-by volt is ZERO.
uint8_t Flag_Relay_MidStop;
uint8_t Flag_Relay_OPEN;
uint8_t Flag_Relay_CLOSE;
uint8_t Flag_Debug_8u = FALSE;

	//8-bits
uint8_t ACT_Door = 0;                   //Controller's cmd (0:Stop /1:Open /2:Close)
uint8_t ST_Door = 0;                    //Operating status (0:Stop or standby /1:Opening /2:Closing)
uint8_t ST_ONEKEY_8u = 0;               //
uint8_t ST_Door_buf, ST_Door_buf2;
uint8_t ST_Close;                       //Recode the 2-seg close cmd
uint8_t	ST_Anti;
uint8_t ST_Press;
uint8_t aRxBuffer[256];
uint8_t PWM_Period = 100;
uint8_t PWM_Duty = 50;
uint8_t PWM_Count = 0;
uint8_t TM_IR_Lock = 0;
uint8_t CNT_Jog_Press = 0;
uint8_t CNT_LOCK_Press = 0;
uint8_t ADC_Detect_Start_Flag = 0;
uint8_t Times_OverADC = 0;
uint8_t Times_OverADC_Target = 2;	//�����QĲ�o����
uint8_t Anti_Event = 0;				//����Ĳ�o���O 0:���`�B�� 1:�}������ 2:��������
uint8_t Anti_Event_buf = 0;				//����Ĳ�o���O 0:���`�B�� 1:�}������ 2:��������
uint8_t ST_Low_Operate = 0;	

uint8_t EE_Addr_P = 0;
uint8_t ST_Save_8u = 0;
	//16-bits

uint16_t TM_Printf = 10;
static uint16_t aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];	  //Variable containing ADC conversions data
uint16_t TM_OPEN = 0;                   //Time: Door open
uint16_t TM_OPEN_Buf_16u = 0;                   //Time: Door open
uint16_t TM_CLOSE = 0;                  //Time: Door close
uint16_t TM_CLOSE_EndPart = 0;                  //Time: Door close
uint16_t TM_CLOSE_b = 0;                  //Time: Door close
uint16_t TM_AntiDly;
uint16_t Time_AntiDly = 20;
uint16_t TM_AntiDly2;
uint16_t TM_AntiDly3;
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
uint8_t TM_Vstb_Extend_8u = 0;
uint8_t Time_Vstb_Extend_8u = 0;

uint16_t ADC_OPEN_MAX_16u = 0;	//[???]
uint16_t ADC_OPEN_MIN;
uint16_t ADC_CLOSE_MAX_16u = 0;	//[???]
uint16_t ADC_CLOSE_MIN;
uint16_t ADC_OPEN_MAX_b;
uint16_t ADC_OPEN_MIN_b;
uint16_t ADC_CLOSE_MAX_b;
uint16_t ADC_CLOSE_MIN_b;
uint16_t ADC_Anti_Max;
uint16_t ADC_Anti_MAX_STD_8u;
uint16_t ADC_AVE_16u;
uint16_t ADC_EndDetect_16u;

uint16_t Door_Select_8u;

uint16_t TM_Anti_Occur = 0;
uint16_t TM_Save = 2*60*60;			//�B�স���x�s���g��
uint16_t TM_ADC_Relaod = 0;
uint16_t TM_Low_Operate = 0;

uint16_t TM_Save_1st_16u = 60;
uint8_t Flag_60s_Save = FALSE;

uint16_t TM_SAVE_Buf;

uint16_t Time_Remain_Open_16u = 0;   //�}���B��ɶ�           
uint16_t Time_Remain_Close_16u = 0;  //�����B��ɶ�

uint16_t Time_Low_Operate_Ini;
uint16_t Time_Low_Operate_Mid;


	//32-bits
uint32_t RLY_Delay_ms = 20;			   //Relay_Delay_time(*1ms)
uint32_t uwPrescalerValue = 0;         // Prescaler declaration
uint32_t Cycle_times_up = 0;
uint32_t Cycle_times_down = 0;
uint32_t Ver_date = 20210723;
uint32_t REC_Operate_Times_32u;

uint8_t TXBuf[16];

	//Float
float Voc_base,Voc_base_;
float Voc_base_2;
float Vo1,Vo2;
float V_Diff,V_Diff_1,V_Diff_2;
float Anti_Weight;
float Vadc_ave;

//===============================================================//
//�~�q��TME 
//EEPROM
uint8_t Flag_Rly_TME_A_8u;			//TME���߱���A
uint8_t Flag_Rly_TME_B_8u;			//TME���߱���B
uint8_t Flag_Rly_TME_TER_8u;		//TME�Ѱ�����
uint16_t Time_RlyEvent_TME_A_16u;	//�~�q��TME���߮ɶ�A
uint16_t Time_RlyEvent_TME_B_16u;	//�~�q��TME���߮ɶ�B
uint16_t Time_RlyEvent_TER_TME_16u;	//�~�q��TME�Ѱ��ɶ�
uint16_t Time_RlyOp_TME_16u;			//�~�q��TME�ʧ@�ɶ�

//��XRelay
uint8_t Rly_TME_A_8u;
uint8_t Rly_TME_B_8u;
uint8_t ST_RlyEvent_TME_A_8u;
uint8_t ST_RlyEvent_TME_B_8u;
uint8_t ST_RlyEvent_TER_TME_8u;
uint16_t TM_RlyEventDelay_TME_A_16u;
uint16_t TM_RlyEventDelay_TME_B_16u;
uint16_t TM_RlyEventDelay_TER_TME_16u;

//Relay��X�P�_TIMER
uint16_t TM_Relay_TME_16u;

//===============================================================//
//�~�q��ACT 
//EEPROM
uint8_t Flag_Rly_ACT_A_8u;			//ACT���߱���A
uint8_t Flag_Rly_ACT_B_8u;			//ACT���߱���B
uint8_t Flag_Rly_ACT_TER_8u;		//ACT�Ѱ�����
uint16_t Time_RlyEvent_ACT_A_16u;	//�~�q��ACT���߮ɶ�A
uint16_t Time_RlyEvent_ACT_B_16u;	//�~�q��ACT���߮ɶ�B
uint16_t Time_RlyEvent_TER_ACT_16u;	//�~�q��ACT�Ѱ��ɶ�
uint16_t Time_RlyOp_ACT_16u;			//�~�q��ACT�ʧ@�ɶ�

//��XRelay
uint8_t Rly_ACT_A_8u;
uint8_t Rly_ACT_B_8u;
uint8_t ST_RlyEvent_ACT_A_8u;
uint8_t ST_RlyEvent_ACT_B_8u;
uint8_t ST_RlyEvent_TER_ACT_8u;
uint16_t TM_RlyEventDelay_ACT_A_16u;
uint16_t TM_RlyEventDelay_ACT_B_16u;
uint16_t TM_RlyEventDelay_TER_ACT_16u;

//Relay��X�P�_TIMER
uint16_t TM_Relay_ACT_16u;
//===============================================================//
//�~�q��POS 
//EEPROM
uint8_t Flag_Rly_POS_A_8u;			//POS���߱���A
uint8_t Flag_Rly_POS_B_8u;			//POS���߱���B
uint8_t Flag_Rly_POS_TER_8u;		//POS�Ѱ�����
uint16_t Time_RlyEvent_POS_A_16u;	//�~�q��POS���߮ɶ�A
uint16_t Time_RlyEvent_POS_B_16u;	//�~�q��POS���߮ɶ�B
uint16_t Time_RlyEvent_TER_POS_16u;	//�~�q��POS�Ѱ��ɶ�
uint16_t Time_RlyOp_POS_16u;			//�~�q��POS�ʧ@�ɶ�

//��XRelay
uint8_t Rly_POS_A_8u;
uint8_t Rly_POS_B_8u;
uint8_t ST_RlyEvent_POS_A_8u;
uint8_t ST_RlyEvent_POS_B_8u;
uint8_t ST_RlyEvent_TER_POS_8u;
uint16_t TM_RlyEventDelay_POS_A_16u;
uint16_t TM_RlyEventDelay_POS_B_16u;
uint16_t TM_RlyEventDelay_TER_POS_16u;

//Relay��X�P�_TIMER
uint16_t TM_Relay_POS_16u;


//�@��: �w�ļȦs��
uint8_t TMP_Flag_LOCK_8u;
uint16_t TMP_TM_OPEN_16u;
uint16_t TMP_TM_CLOSE_16u;

//�@��
uint8_t Flag2_Door_UpLimit_8u;
uint8_t Flag2_Door_DownLimit_8u;
uint8_t Trig_RM_8u;


//���ﾹ
uint8_t ST_BUZZ_8u;
uint8_t ST_BUZZ_Buf_8u;
uint8_t ST_BUZZ_A_8u;
uint8_t TM_Buzz_ON_8u;
uint8_t TM_Buzz_ON_Buf_8u;
uint8_t TM_Buzz_OFF_8u;
uint8_t TM_Buzz_OFF_Buf_8u;
uint8_t Flag3_Door_UpLimit_8u;
uint8_t CNT_Buzz_8u;
uint16_t TM_Buzz_16u;

uint8_t W_IR_Buf0;
uint8_t W_IR_Buf1;

//Dubug
uint8_t CNT_Debug_BTN_8u = 0;

uint8_t Flag4_Door_UpLimit_8u = FALSE;
uint8_t Time_Open_Break_8u = 0;

uint8_t Flag_End;

uint8_t ST_OpEnd_8u,ST_End_Detect_P1_8u;
uint8_t Flag_Slope_Hi_8u;
uint8_t Slope_ADC_16u;
uint16_t ADC_Buf_16u;
uint16_t TM_Dly_InitSample_16u;
uint8_t CNT_NoWork_8u;

uint16_t TM_Idle_16u = 0;

int i,j;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Door_Open(void);
void Door_Stop(void);
void Door_Stop_2(void);
void Door_Close(void);		
void Door_Open_JOG(void);
void Door_Stop_JOG(void);
void Door_Close_JOG(void);		
void Door_manage(void);                                    //Operating time calculate
void PWR_CTRL(void);                                       //Power ON to motor
void Delay_ms(int32_t nms);
//static void SystemClock_Config(void);
static void EXTI4_15_IRQHandler_Config(void);
static void EXTI2_3_IRQHandler_Config(void);
static void EXTI0_1_IRQHandler_Config(void);
static void TIM16_Config(void);
static void TIM17_Config(void);
static void ADC_Config(void);
static void Uart_Config(void);
static void I2C_Config(void);
static void Error_Handler(void);

static void CLOCK_Enable(void);

static void MotorRelay_out_config(void);
static void StatusRelay_out_config(void);
static void StatusRelay_Control(void);
static void Anti_Pressure_5(void);
static void OpEnd_Detect(void);			//Door unload detect
static void OpEnd_Detect_2(void);			//Door unload detect
static void OpEnd_Detect_Close(void);
static void Buzzer_Config(void);
static void ControlBox_Config(void);
static void Ext_CNTER(void);			//RelayĲ�o�~���p�ƾ�
static uint16_t	TIMDEC(uint16_t TIMB);
static uint16_t	TIMINC(uint16_t TIMB);
static uint16_t ADC_Calculate(void);
uint16_t* BubbleSort(uint16_t arr[], uint16_t len);

static void Buzzer_CTRL(void);
static void Buzz_ON(void);
static void Buzz_OFF(void);

static void LED_CTRL(void);
static void LED_ON(void);
static void LED_OFF(void);

static void Parameter_Load(void);	//EEPROM�Ѽ�Ū��
static void Parameter_List(void);	//EEPROM�Ѽ����
static void SMK_CTRL(void);	    //�ϷP����
static void Cycle_Test(void);	//
static void IR_CTRL(void);	    //
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

static void Low_Operate_Function(void);


static void SR_CTRL_TME_A(void);
static void SR_CTRL_TME_B(void);
static void SR_TERMINATE_TME(void);

static void SR_CTRL_ACT_A(void);
static void SR_CTRL_ACT_B(void);
static void SR_TERMINATE_ACT(void);

static void SR_CTRL_POS_A(void);
static void SR_CTRL_POS_B(void);
static void SR_TERMINATE_POS(void);


static void SR_OUTPUT_CTRL(void);	//Status-Relay Output Control
static void SR_VAR_BUF(void);		//Status-Relay Variable Reset
static void SR_STVAR_RST(void);		//Status-Relay Status-Variable Reset

void Relay_TME_ON(void);
void Relay_TME_OFF(void);
void Relay_ACT_ON(void);
void Relay_ACT_OFF(void);
void Relay_POS_ON(void);
void Relay_POS_OFF(void);


static void Dubug_CTRL(void);
static void Fun_Debug_Enable(void);
static void Fun_Debug_Disable(void);

static void Fun_Break_AFT_Open(void);

static void PWM_Grade_Select(uint8_t Grade_8u);

/* Private functions ---------------------------------------------------------*/


//Variable to ADC conversion
//uint16_t i,j;
float Voc,Voc_;

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
  EXTI2_3_IRQHandler_Config();
  EXTI0_1_IRQHandler_Config();
	TIM16_Config();
  TIM17_Config();
  Uart_Config();
  I2C_Config();
	
  /* Enable each GPIO Clock */
  CLOCK_Enable();
  

  // Parameter access
  //EE_Default = FALSE;
  Parameter_Load();  
  Parameter_List();	

  /* Configure IOs in output push-pull mode to drive Relays */
  MotorRelay_out_config();
  StatusRelay_out_config();
  Buzzer_Config();	
  ControlBox_Config();

  TM_OPEN = 0;
  TM_CLOSE = 0;
  CloseTM2 = TM_MAX - TM_WindowsDoor_ClosePart1;		//section_time_2 of close operation
  if(Flag_WindowsDoor == TRUE && CloseTM2 <= 0){
		Flag_WindowsDoor = FALSE;
		printf("\n\r�������\��ĤG�q�����ɶ� = %d", CloseTM2);
		printf("\n\r�������\��: %d", Flag_WindowsDoor);
		printf("\n");
  }
  ST_Close = 1;
	
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);	
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);	
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
		
  if(Flag_CycleTest == TRUE){
		//TM_MAX = 600;
		TM_OPEN = TM_MAX;
		Wait_flg = TRUE;
		ACT_Door = 1;
		//Volt_StandBy_32f = 0.3;
		printf("\n\r�`������:��\n");
  }
		
  Rly_TME_A_8u = FALSE;
  Rly_TME_B_8u = FALSE;
  Rly_ACT_A_8u = FALSE;
  Rly_ACT_B_8u = FALSE;
  Rly_POS_A_8u = FALSE;
  Rly_POS_B_8u = FALSE;
  
  LED_OFF();
  
  //�}�����ܭ�
  ST_BUZZ_8u = 1;
  
	//???
  //Fun_Debug_Enable();
  //Flag_Debug_8u = TRUE;
  //Flag_Func_JOG = TRUE;
  
	while(1){ 
		if(Flag_CycleTest == TRUE){
			//�`������
			Cycle_Test();
			ADC_Calculate();
			Operate_Infor_Save();
		}else{ 
			//Main function
			//printf("\n\rPI_OC = % d",HAL_GPIO_ReadPin(PORT_OC,PIN_OC));
			Door_manage();
			Low_Operate_Function();
			IR_CTRL();
			SMK_CTRL();
			PWR_CTRL();
			ADC_Calculate();
			Operate_ADC_Detect();
			Auto_Close_CTRL();
			Anti_Pressure_5();
			Operate_Infor_Save();
			StatusRelay_Control();
			Buzzer_CTRL();
			LED_CTRL();
			Dubug_CTRL();
			Debug_Monitor();
		}
  }
}


static void Debug_Monitor(void){
	//�ثe���A����
	if(TM_Printf == 0){
		printf("\n\r==============���Ascan===================");			
		
		ADC_Promp_16u = ADC_AVE_16u;
		Voc_ = ADC_Promp_16u *(3.3/4096);		
		printf("\n\r �ݾ��W��V = %2.3f V/ I = %2.2f A/ AD = %d",Volt_StandBy_32f,Volt_StandBy_32f*20/1.825, ADC_StandBy_16u);
		printf("\n\r �ثe    V = %2.3f V/ I = %2.2f A/ AD = %d",Voc_,Voc_*20/1.825,ADC_Promp_16u);
		printf("\n\r �ݾ��U��V = %2.3f V/ I = %2.2f A/ AD = %d",Volt_StandBy_b_32f,Volt_StandBy_b_32f*20/1.825, ADC_StandBy_b_16u);
		
		printf("\n\n\r �������AD = %d",ADC_OpEnd_16u);
		
		printf("\n\n\r �B�স�� = %d",REC_Operate_Times_32u*2);
		
		printf("\n");
		printf("\n\r �������AD��:");
		printf("\n\r ADC_OPEN_MAX_16u = %d",ADC_OPEN_MAX_16u);
		printf("\n\r ADC_CLOSE_MAX_16u = %d",ADC_CLOSE_MAX_16u);
		
		printf("\n");
		printf("\n\r �ثe�����A = %d  (0:����/1:�}����/2:������)",ST_Door);
		
		if(TM_OPEN > 0 || TM_CLOSE > 0){
			if(Flag_Low_Operate == TRUE){
				printf("\n");
				printf("\n\r �w�B�B�બ�A = %d (%2.0f��)", ST_Low_Operate, (float)TM_Low_Operate/10);
				printf("\n\r (1:��q/2:���q/3:���q)");		
			}

			if(TM_OPEN > 0){
				printf("\n\n\r �}���Ѿl�ɶ� = %2.1f s",(float)TM_OPEN/10);
				printf("\n\r �B��t�� = %3.2f",(float)100*PWM_Duty/PWM_Period);
			}
			if(TM_CLOSE > 0){
				printf("\n\n\r �����Ѿl�ɶ� = %2.1f s",(float)TM_CLOSE/10);
				printf("\n\r �B��t�� = %3.2f",(float)100*PWM_Duty/PWM_Period);
			}
			
		}
				
		if(TM_Auto_Close > 0){
			printf("\n\n\r�������ݮɶ� = %d ms",TM_Auto_Close);
		}
		
		if(ST_Anti > 0){
			printf("\n\n\r�������A = %d",ST_Anti);
			printf("\n\n\r�������v�� = %f",Anti_Weight);
			printf("\n\n\r�����ɭ��� = %d",ADC_Anti_Max);
		}
					
		if(Flag_WindowsDoor == TRUE){		//1-segment mode
			//printf("\n\rOPEN_IT= %d",Open_IT);
			printf("\n\n\r�����������AST_Close= %d",ST_Close);
		}
		
		if(Anti_Event > 0){
			printf("\n");
			printf("\n\r ����Ĳ�o1�}��2����(%d)",Anti_Event);
			printf("\n\n\r �}����������ݮɶ�(%2.0f��)",(float)TM_Anti_Occur/10);
		}
		
		if(ADC_Detect_Start_Flag == 1){
			printf("\n");
			printf("\n\r �B��AD��(�Y��):");
			printf("\n\r ADC_OPEN_MAX_b = %d",ADC_OPEN_MAX_b);
			//printf("\n\r ADC_OPEN_MIN_b = %d",ADC_OPEN_MIN_b);
			printf("\n\r ADC_CLOSE_MAX_b = %d",ADC_CLOSE_MAX_b);
			//printf("\n\r ADC_CLOSE_MIN_b = %d",ADC_CLOSE_MIN_b);
		}

		if(TM_ADC_Relaod > 0){
			printf("\n");
			printf("\n\r TM_ADC_Relaod = %d",TM_ADC_Relaod);
		}
		//printf("\r\n\nAnti_Weight = %f", Anti_Weight = 1.5);
		if(ADC_Detect_Start_Flag > 0){
			printf("\n");
			printf("\n\r ADC_Detect_Start_Flag = %d",ADC_Detect_Start_Flag);
		}
		
		if(Flag_IR == TRUE){
			printf("\n");
			printf("\n\r //////////���~�u����Ĳ�o\\\\\\\\\\");
			printf("\n\r Flag_IR = %d",Flag_IR);
		}
		if(Flag_SMK == TRUE){
			printf("\n");
			printf("\n\r //////////�����P��������Ĳ�o\\\\\\\\\\");
			printf("\n\r Flag_SMK = %d",Flag_SMK);
		}
		
		if(ST_BUZZ_8u > 0){
			printf("\n");
			printf("\n\r ���ﾹ���A = %d",ST_BUZZ_8u);
		}

		if(OpEnd_Detect_Start_Flag == TRUE){
			printf("\n");
			printf("\n\r ���찻���}�l(%d)",OpEnd_Detect_Start_Flag);
		}
		
		printf("\n\r");
		
		if(Flag_Debug_8u == TRUE){
			TM_Printf = 10;
		}
	}	
}

static void Low_Operate_Function(void){
	if(Flag_Low_Operate == TRUE){
		if(TM_OPEN > 0 || TM_CLOSE > 0){
			if(TM_Low_Operate < Time_Low_Operate_Ini){ //��B�Ұ�
				//PWM_Duty = 1;
				//PWM_Period = 2;
				PWM_Grade_Select(0);
				ST_Low_Operate = 1;
			}else if(TM_Low_Operate < (Time_Low_Operate_Ini + Time_Low_Operate_Mid)){ //���q�[�t
				//PWM_Duty = 99;
				//PWM_Period = 100;
				PWM_Grade_Select(3);
				ST_Low_Operate = 2;
			}else{	//���q��t
				//PWM_Duty = 1;
				//PWM_Period = 2;
				PWM_Grade_Select(0);
				ST_Low_Operate = 3;
			}
			

			if(ST_Low_Operate == 2){
				Volt_StandBy_32f = Volt_StandBy_b_32f * iWeight_Vstb_8u;
			}else{
				Volt_StandBy_32f = Volt_StandBy_b_32f * 1.1;
			}

		}else{
			Volt_StandBy_32f = Volt_StandBy_b_32f * 1.3;
			
		}
	}
}

static void Operate_Infor_Save(void){
	uint8_t W_ADR, ee_addr; 
	//uint8_t Pack_Size;
	
	W_ADR = 0;
	//Pack_Size = sizeof(TXBuf);
	
	if(ST_Save_8u == 1){
		TXBuf[W_ADR++] = REC_Operate_Times_32u;
		TXBuf[W_ADR++] = REC_Operate_Times_32u >> 8;
		TXBuf[W_ADR++] = REC_Operate_Times_32u >> 8*2;
		TXBuf[W_ADR++] = REC_Operate_Times_32u >> 8*3;
		
		TXBuf[W_ADR++] = Time_Remain_Open_16u;
		TXBuf[W_ADR++] = Time_Remain_Open_16u >> 8;
		
		TXBuf[W_ADR++] = Time_Remain_Close_16u;
		TXBuf[W_ADR++] = Time_Remain_Close_16u >> 8;
		
		TXBuf[W_ADR++] = ADC_OPEN_MAX_16u;
		TXBuf[W_ADR++] = ADC_OPEN_MAX_16u >> 8;
		

		TXBuf[W_ADR++] = ADC_CLOSE_MAX_16u;
		TXBuf[W_ADR++] = ADC_CLOSE_MAX_16u >> 8;
		
		for(ee_addr = 0; ee_addr < sizeof(TXBuf); ee_addr += 8){
			HAL_I2C_Mem_Write(&I2cHandle,(uint16_t)I2C_ADDRESS, ee_addr + 200, I2C_MEMADD_SIZE_8BIT, (uint8_t*)TXBuf + ee_addr, 8, 10000);
			Delay_ms(6);
		}
		
		//HAL_I2C_Mem_Read(&I2cHandle,(uint16_t)I2C_ADDRESS, 0, I2C_MEMADD_SIZE_8BIT, (uint8_t*)aRxBuffer, 256, 10000);
		
		ST_Save_8u = 0;
	}
}

//�����}�����ɪ��̤j�P�̤pADC��
static void Operate_ADC_Detect(void){
//Description:�@�����B��q���A�åB�N�̤j�ȧ@�������\�઺�Ѧҭ�
//Timing: ���찻��OpEnd_Detect()�ݾ����A


	uint16_t ADC_Tmp;
	
	//ADC_Detect_Start_Flag
	//0: ���Jbuffers
	//1: �z�LOpend_detect()�ҥ�,����ADC�ܤ�
	//2: Reload�����ܼ�
	
	if(ADC_Detect_Start_Flag == 0){
		ADC_Tmp = ADC_AVE_16u;
		ADC_OPEN_MAX_b = ADC_Tmp;
		ADC_OPEN_MIN_b = ADC_Tmp;
		ADC_CLOSE_MAX_b = ADC_Tmp;
		ADC_CLOSE_MIN_b = ADC_Tmp;
	
	}else if(ADC_Detect_Start_Flag == 1){	//���찻���}�l�Y����
		if(ST_Door == 1 && TM_OPEN > 0){
			ADC_Tmp = ADC_AVE_16u;
			if(ADC_Tmp > ADC_OPEN_MAX_b){
				ADC_OPEN_MAX_b = ADC_Tmp;
			}else if(ADC_Tmp < ADC_OPEN_MIN_b){
				ADC_OPEN_MIN_b = ADC_Tmp;
			}
		}else if(ST_Door == 2 && TM_CLOSE_EndPart > 0){
			ADC_Tmp = ADC_AVE_16u;
			if(ADC_Tmp > ADC_CLOSE_MAX_b){
				ADC_CLOSE_MAX_b = ADC_Tmp;
			}else if(ADC_Tmp < ADC_CLOSE_MIN_b){
				ADC_CLOSE_MIN_b = ADC_Tmp;
			}
		}
		
	}else if(ADC_Detect_Start_Flag == 2){
		printf("\n\r ADC_Detect_Start_Flag = %d",ADC_Detect_Start_Flag);
		printf("\n\r TM_ADC_Relaod = %d",TM_ADC_Relaod);
		printf("\n\r Anti_Event = %d",Anti_Event);
		printf("\n\r Anti_Event_buf = %d",Anti_Event_buf);
		if(TM_ADC_Relaod == 0 && Anti_Event_buf == 0){
			if(Flag_Door_UpLimit == TRUE){
				if(ADC_OPEN_MAX_b > ADC_OPEN_MAX_16u){
					ADC_OPEN_MAX_16u = ADC_OPEN_MAX_b;
					ADC_OPEN_MIN = ADC_OPEN_MIN_b;
				}
			}else if(Flag_Door_DownLimit == TRUE){
				if(ADC_CLOSE_MAX_b > ADC_CLOSE_MAX_16u){
					ADC_CLOSE_MAX_16u = ADC_CLOSE_MAX_b;
					ADC_CLOSE_MIN = ADC_CLOSE_MIN_b;
				}
			}
		}else{
			ADC_Detect_Start_Flag = 0;
		}
		
		//�����ѦҭȤU��
		//if(ADC_OPEN_MAX_16u < 1000){
		//	ADC_OPEN_MAX_16u = 3700;
		//}
		//if(ADC_CLOSE_MAX_16u < 1000){
		//	ADC_CLOSE_MAX_16u = 3700;
		//}
		
		Anti_Event_buf = 0;
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
		//REC_Operate_Times_32u++;
		Ext_CNTER();
		printf("\n\rNoise test 1");
	}else if(TM_CLOSE == 0 && ACT_Door == 2 && TM_DLY == 0){
		Door_Stop();
		TM_OPEN = TM_MAX;
		ACT_Door = 1;
		Wait_flg = TRUE;
		Cycle_times_up++;
		
		Ext_CNTER();
		//HAL_GPIO_WritePin(PORT_Status_Out, RL_TME, GPIO_PIN_SET);
		//Delay_ms(RLY_Delay_ms);
		//HAL_GPIO_WritePin(PORT_Status_Out, RL_TME, GPIO_PIN_RESET);
		printf("\n\rNoise test 2");

	}
	
	Door_manage();
	PWR_CTRL(); 
	//OpEnd_Detect();
	//Operate_Infor_Save();

	
	//if(TM_Printf == 0){
	//	printf("\n\r==============�`������-���Ascan2===================");			
	//	Voc_ = ADC_AVE_16u *(3.3/4096);		
	//	printf("\n\r�ثe�q���� = %f V",Voc_);
	//	printf("\n\r�ݾ��q��   = %f V",Volt_StandBy_32f);
	//	printf("\n\rACT_Door = %d",ACT_Door);
	//	if(TM_OPEN > 0){
	//		printf("\n\n\r�}���Ѿl�ɶ� = %d ms",TM_OPEN);
	//		//printf("\n\r�}������ = %d\n",Cycle_times_down);
	//	}
	//	if(TM_CLOSE > 0){
	//		printf("\n\n\r�����Ѿl�ɶ� = %d ms",TM_CLOSE);
	//		//printf("\n\r�������� = %d\n",Cycle_times_down);
	//	}
	//	if(TM_DLY > 0){
	//		printf("\n\rTM_DLY = %d",TM_DLY);
	//	}
	//	TM_Printf= 10;
	//}	


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
	HAL_GPIO_WritePin(PORT_Status_Out, RL_TME, GPIO_PIN_SET);
	Delay_ms(RLY_Delay_ms);
	HAL_GPIO_WritePin(PORT_Status_Out, RL_TME, GPIO_PIN_RESET);
}

void PWR_CTRL(void){
	uint8_t iIR_tmp_8u;	
	
	if(TM_CLOSE > 0 && TM_OPEN > 0){
		//Avoid both of OPEN and CLOSE have be trigger simultaneously.		
		Door_Stop();
		TM_OPEN = 0;
		TM_CLOSE = 0;
		printf("\n\n\rNG:�B��ɶ��P��>0");

	}else{	
		
		//�����eŪ�����~�u�P����,��ON�ɨ�������
		iIR_tmp_8u = HAL_GPIO_ReadPin(PORT_IR, W_IR);
		if(TM_CLOSE > 0 && iIR_tmp_8u == FALSE){
			TM_CLOSE = 0;
		}
		
		ST_Door_buf = ST_Door;
		if(Flag_WindowsDoor == FALSE){		//���`�}�����Ҧ�
			if(TM_OPEN > 0){
				PWM_Grade_Select(3);
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
				ST_Anti = 4;
				if(ST_Door == 1){
					Open_IT = FALSE;
					Close_IT = FALSE;
					Close_IT2 = FALSE;

				}else if(ST_Door == 2){
					if(Open_IT == TRUE){
						ST_Close = 1;
					}else if(ST_Close == 1){	//�Ĥ@�q����
						printf("TEST 01");
						if(TM_CLOSE_b == 0){
							printf("TEST 01");
							ST_Close = 2;
						}
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
	//if(Flag_WindowsDoor == FALSE && TM_CLOSE > 0){
		//OpEnd_Detect_Close();
	//}else 
	if(TM_OPEN > 6 || TM_CLOSE > 6){
		OpEnd_Detect();
		OpEnd_Detect_2();
	//������:�Ĥ@�q�����������x�s����AD��
	}else if(Flag_WindowsDoor == TRUE && TM_CLOSE == 0 && TM_CLOSE_b != 0 && Anti_Event == 0){
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
	
	
	//����c�O��
	if(TM_OPEN > 0){
		CtrlBox_Light_Up();
	}else if(TM_CLOSE > 0){
		CtrlBox_Light_Down();
	}else{
		CtrlBox_Light_OFF();
	}
	
	if(TM_CLOSE == 0){
		ST_OpEnd_8u = 0;
	}
	
	if(TM_OPEN == 0 && TM_CLOSE == 0){
		PWM_Grade_Select(PWM_Grade);
	}
	
	if(TM_OPEN == 0 && TM_CLOSE == 0){
		if(TM_Idle_16u == 0){
			if(HAL_GPIO_ReadPin(PORT_Motor_Out, RLY_DIR) != GPIO_PIN_RESET){
				HAL_GPIO_WritePin(PORT_Motor_Out, RLY_DIR, GPIO_PIN_RESET);		
			}
			//if(HAL_GPIO_ReadPin(PORT_Motor_MOS, MOS_ACT) == GPIO_PIN_RESET){
			//	HAL_GPIO_WritePin(PORT_Motor_MOS, MOS_ACT, GPIO_PIN_SET);		
			//}
		}
	}else{
		TM_Idle_16u = 30;
	}
	//Fun_Break_AFT_Open();
	
	if(TM_OPEN != 0 && TM_OPEN_Buf_16u == 0){
		REC_Operate_Times_32u++;
	}
	TM_OPEN_Buf_16u = TM_OPEN;
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
					TM_CLOSE_EndPart = TM_CLOSE - 20;
					TM_AntiDly = Time_AntiDly;
					TM_EndDetec = 10;
					AClose_Flg = FALSE;
					ST_OpEnd_8u = 0;
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
							TM_CLOSE_EndPart = TM_CLOSE - 10;
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
		
		//ADC�������ݮɶ�
		if(TM_OPEN > 0 || TM_CLOSE > 0){
			TM_ADC_Relaod = 50;
			TM_Low_Operate = 0;
		}
	}
}	

static void Auto_Close_CTRL(void){
	
	//����~�u�Ѱ��� + �۰������\�঳, ���s�۰������˼�
	W_IR_Buf0 = HAL_GPIO_ReadPin(PORT_IR, W_IR);
	if(W_IR_Buf0 == TRUE && W_IR_Buf1 == FALSE){
		if(Flag_AutoClose != 0){
			AClose_Flg = TRUE;
			TM_Auto_Close = Time_Auto_Close;
		}
	}
	
	//�۰������\��
	if(Flag_SMK == FALSE){
		// �}���᩵��ɶ��g�L�۰�����
		if(Flag_AutoClose == 1){
			if(AClose_Flg == TRUE && TM_Auto_Close == 0){		//�۰������˼Ʈɶ�����
				printf("\n\r�۰��������ݮɶ���F");
				printf("\n\r�����ɶ��]��");
				AClose_Flg = FALSE;									//�۰������X��:OFF
				if(Anti_Event == 2){ 		//����������: break
						
				}else{
					TM_CLOSE = TM_MAX;									//�����ɶ��]�w
					TM_AntiDly = Time_AntiDly;
					TM_EndDetec = 10;
					ST_OpEnd_8u = 0;
				}
			}
		}else if(Flag_AutoClose == 2){							//�۰������\��: ON
			if(Flag_WindowsDoor == FALSE){								//��q�����\��:�L
				if(	ST_Door == 0 && ST_Door_buf == 1){					//�P�_������e�����A�O�_���}��
					printf("\n\r�۰������X��1 & ���ݮɶ��]��");
					TM_Auto_Close = Time_Auto_Close;					//�]�w�۰������˼Ʈɶ�
					AClose_Flg = TRUE;									//�۰������X��:ON
				}else if( ST_Door == 0 && ST_Door_buf2 == 1){			//������O�W->��
					printf("\n\r�۰������X��2 & ���ݮɶ��]��");
					TM_Auto_Close = Time_Auto_Close;					//�]�w�۰������˼Ʈɶ�
					AClose_Flg = TRUE;									//�۰������X��:ON
					ST_Door_buf2 = 0;
				}else if(AClose_Flg == TRUE && TM_Auto_Close == 0){		//�۰������˼Ʈɶ�����
					printf("\n\r�۰��������ݮɶ���F");
					printf("\n\r�����ɶ��]��");
					AClose_Flg = FALSE;									//�۰������X��:OFF
					if(Anti_Event == 2){ 		//����������: break
							
					}else{
						TM_CLOSE = TM_MAX;									//�����ɶ��]�w
						TM_AntiDly = Time_AntiDly;
						TM_EndDetec = 10;
						ST_OpEnd_8u = 0;
					}
				}
			}else{
				//empty
			}
		}else{
			//Empty
		}
	}
	
	//���ﾹ�ʧ@�P�_
	if(TM_Auto_Close > 0 && TM_Auto_Close < 50){
		if(W_IR_Buf0 == TRUE){
			ST_BUZZ_8u = 8;
		}
	}
	
	W_IR_Buf1 = W_IR_Buf0;
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
	printf("\n\r RLY_DIR = %d",HAL_GPIO_ReadPin(PORT_Motor_Out, RLY_DIR));
	
	if(TM_OPEN > 0){
		Delay_ms(RLY_Delay_ms);
		HAL_GPIO_WritePin(PORT_Motor_Out, RLY_ACT, GPIO_PIN_SET);		
		Delay_ms(RLY_Delay_ms);
	}
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
	
	printf("\n\r RLY_DIR = %d",HAL_GPIO_ReadPin(PORT_Motor_Out, RLY_DIR));
	
	if(TM_CLOSE > 0){
		Delay_ms(RLY_Delay_ms);
		HAL_GPIO_WritePin(PORT_Motor_Out, RLY_ACT, GPIO_PIN_SET);
		Delay_ms(RLY_Delay_ms);
	}
	//MOSFET switch ON
	//HAL_GPIO_WritePin(PORT_Motor_MOS, MOS_ACT, GPIO_PIN_RESET);
	if (HAL_TIM_Base_Start_IT(&TimHandle17) != HAL_OK){
		/* Starting Error */
		Error_Handler();
	}	
}

void Door_Stop(void){
//	printf("\n\r----STOP_Relay");
	if(HAL_GPIO_ReadPin(PORT_Motor_Out, RLY_ACT) != GPIO_PIN_RESET){
	
		if (HAL_TIM_Base_Stop_IT(&TimHandle17) != HAL_OK){
			/* Starting Error */
			Error_Handler();
		}	
		HAL_GPIO_WritePin(PORT_Motor_MOS, MOS_ACT, GPIO_PIN_SET);			
	
		//Delay_ms(RLY_Delay_ms);
		HAL_GPIO_WritePin(PORT_Motor_Out, RLY_ACT, GPIO_PIN_RESET);
		//Delay_ms(RLY_Delay_ms);
		//HAL_GPIO_WritePin(PORT_Motor_Out, RLY_DIR, GPIO_PIN_RESET);
		
		printf("\n\r STOP~1");
	}
}
void Door_Stop_2(void){
//	printf("\n\r----STOP_Relay");
	if(HAL_GPIO_ReadPin(PORT_Motor_Out, RLY_ACT) != GPIO_PIN_RESET){
	
		if (HAL_TIM_Base_Stop_IT(&TimHandle17) != HAL_OK){
			/* Starting Error */
			Error_Handler();
		}	
		HAL_GPIO_WritePin(PORT_Motor_MOS, MOS_ACT, GPIO_PIN_SET);			
	
		//Delay_ms(RLY_Delay_ms);
		HAL_GPIO_WritePin(PORT_Motor_Out, RLY_ACT, GPIO_PIN_RESET);
		//Delay_ms(RLY_Delay_ms);
		HAL_GPIO_WritePin(PORT_Motor_Out, RLY_DIR, GPIO_PIN_SET);		
		
		printf("\n\r STOP~2");

	}
}

//For JOG
void Door_Open_JOG(void){
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
	HAL_GPIO_WritePin(PORT_Motor_MOS, MOS_ACT, GPIO_PIN_RESET);
}

void Door_Close_JOG(void){
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
	HAL_GPIO_WritePin(PORT_Motor_MOS, MOS_ACT, GPIO_PIN_RESET);
}

void Door_Stop_JOG(void){
//	printf("\n\r----STOP_Relay");
	if(HAL_GPIO_ReadPin(PORT_Motor_Out, RLY_ACT) != GPIO_PIN_RESET){
		HAL_GPIO_WritePin(PORT_Motor_MOS, MOS_ACT, GPIO_PIN_SET);			
		Delay_ms(RLY_Delay_ms);
		HAL_GPIO_WritePin(PORT_Motor_Out, RLY_ACT, GPIO_PIN_RESET);
		Delay_ms(RLY_Delay_ms);
		HAL_GPIO_WritePin(PORT_Motor_Out, RLY_DIR, GPIO_PIN_RESET);		
	}
}

//******************Relay control end******************//

//�L���������찻��
static void OpEnd_Detect(void){
	
	uint16_t TM_OPEN_A,TM_CLOSE_A;
	
	if(TM_EndDetec == 0 && Flag_WindowsDoor == FALSE){
		if(OpEnd_Detect_Start_Flag == FALSE){                  //����
			TM_DoorOperateDly = 5;                             //Delay 0.5 second��}�l�T�{�O�_�쭭��
			OpEnd_Detect_Start_Flag = TRUE;
			ADC_Detect_Start_Flag = 1;		                   //ADC�B����^���}�l:Operate_ADC_Detect
		}else{
			ADC_TMP_16u = ADC_AVE_16u;
			Voc = (float)ADC_TMP_16u *(3.3/4096);		
			//if(Voc > Volt_StandBy_32f && Voc > (Volt_StandBy_b_32f) && TM_DoorOperateDly == 0){	//���q������P�w
			//	TM_Vstb_Extend_8u = Time_Vstb_Extend_8u;
			//	Flag_End = FALSE;
			//}else{
			//	Flag_End = TRUE;
			//}
			//
			//if(Flag_End == TRUE){
			//	//PWM_Grade_Select(1);
			//}else{
			//	PWM_Grade_Select(PWM_Grade);
			//}
			
			if((Voc <= Volt_StandBy_32f && HAL_GPIO_ReadPin(PORT_OC, PIN_OC) == RESET)&&// && Voc >= Volt_StandBy_b_32f) && 
			   (TM_DoorOperateDly == 0 && TM_Vstb_Extend_8u == 0)){
				
				ADC_OpEnd_16u = ADC_TMP_16u;
				
				TM_OPEN_A = TM_OPEN;
				TM_CLOSE_A = TM_CLOSE;
				
				TM_OPEN = 0;
				TM_CLOSE = 0;
				Door_Stop();
				
				printf("\n\n\r�����-����B��!\n\n");
		//printf("\n\r �ݾ��W��V = %2.3f V/ I = %2.2f A/ AD = %d",Volt_StandBy_32f,Volt_StandBy_32f*20/1.825, ADC_StandBy_16u);
		printf("\n\r �ثe    V = %2.3f V/ I = %2.2f A/ AD = %d",Voc,Voc*20/1.825,ADC_TMP_16u);
		printf("\n\r �ݾ��U��V = %2.3f V/ I = %2.2f A/ AD = %d",Volt_StandBy_32f,Volt_StandBy_32f*20/1.825, ADC_StandBy_b_16u* iWeight_Vstb_8u);
				
				//�P�_�O�_�}�����,�åB�]�w�ө��ɶ�
				if(TM_OPEN_A > 0){
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
				if(TM_OPEN_A > 0){
					Flag_Door_UpLimit   = TRUE;
					Flag_Door_DownLimit = FALSE;
					Flag2_Door_UpLimit_8u   = TRUE;
					Flag2_Door_DownLimit_8u = FALSE;
					Flag3_Door_UpLimit_8u = TRUE;
					Flag4_Door_UpLimit_8u = TRUE;
					Door_Close();	//�}���ϦV�٨�
				}else if(TM_CLOSE_A > 0){
					Flag_Door_UpLimit   = FALSE;
					Flag_Door_DownLimit = TRUE;
					Flag2_Door_UpLimit_8u   = FALSE;
					Flag2_Door_DownLimit_8u = TRUE;
					Door_Open();
				}
				
				//�B��ɶ��p��
				if(TM_OPEN_A > 0){
					Time_Remain_Open_16u = TM_MAX - TM_OPEN_A;
					ST_ONEKEY_8u = 2;
					Door_Stop();	//�ϦV�٨���ߧY����B��
					printf("\n\n\r �}��STOP");
					printf("\n\n\r");
				}
				
				if(TM_CLOSE_A > 0){
					Time_Remain_Close_16u = TM_MAX - TM_CLOSE_A;
					ST_ONEKEY_8u = 4;
					Door_Stop_2();
				}
				
				//�B�স��
				if(TM_OPEN_A > 0){
					//REC_Operate_Times_32u++;
					ST_Save_8u = 1;
				}
				
				if(Flag_WindowsDoor == TRUE){
					if(TM_OPEN_A > 0){
						ST_Close = 1;
					}
				}
				
				//TM_OPEN = 0;
				//TM_CLOSE = 0;
				ST_Anti = 0;
				OpEnd_Detect_Start_Flag = FALSE;		
				ADC_Detect_Start_Flag = 2;		//�B�൲���åB�x�sAD��: Operate_ADC_Detect
			}
		}
	}
}

static void OpEnd_Detect_Close(void){
	uint8_t Grade_tmp_8u;
	uint16_t TM_OPEN_A,TM_CLOSE_A;
	uint8_t	ST_Slope_8u;
	
	TM_CLOSE_A = TM_CLOSE;
	//ADC_Calculate();
	
	switch(ST_OpEnd_8u){
		case 0:
			//���A��l��
			ST_End_Detect_P1_8u = 0;
			ADC_Buf_16u = 0;
			Flag_Slope_Hi_8u = FALSE;
			ST_OpEnd_8u = 1;
			CNT_NoWork_8u = 0;
			break;
		
		case 1:
			if(ST_End_Detect_P1_8u == 0){
				TM_Dly_InitSample_16u = 5;				//�]�w����ɶ�0.5s
				ST_End_Detect_P1_8u = 1;				//�����ܤU�@���q
			}else if(ST_End_Detect_P1_8u == 1){
				if(TM_Dly_InitSample_16u > 0){
					//����ɶ��p�Ƥ�
				}else{
					ADC_EndDetect_16u = ADC_AVE_16u;	//Ū���B��í�w��AD��
					ADC_Buf_16u = ADC_EndDetect_16u;
					if(ADC_EndDetect_16u >= ADC_StandBy_16u){
						ST_OpEnd_8u = 2;
						ST_End_Detect_P1_8u = 0;
						ADC_Detect_Start_Flag = 1;		    //ADC�B����^���}�l:Operate_ADC_Detect
					}else{
						CNT_NoWork_8u++;
						if(CNT_NoWork_8u == 10){
							Door_Stop();
							TM_CLOSE = 0;
							CNT_NoWork_8u = 0;
							ST_OpEnd_8u = 0;
						}
					}
				}
			}
			//Flag_Slope_Hi_8u = FALSE;

			break;
		
		case 2:
			ADC_Buf_16u = (uint16_t)(ADC_Buf_16u + ADC_AVE_16u)/2;
			
			
			if(ADC_Buf_16u >= ADC_EndDetect_16u){
				Slope_ADC_16u =  (uint16_t)(100 *(float)(ADC_Buf_16u - ADC_EndDetect_16u)/ADC_EndDetect_16u);
				ST_Slope_8u = 0;
			}else{
				//Slope_ADC_16u =  (uint16_t)(100 *(float)(ADC_EndDetect_16u - ADC_Buf_16u)/ADC_EndDetect_16u);
				Slope_ADC_16u = 0;
				ST_Slope_8u = 1;
			}
			
			if(Slope_ADC_16u < 30){
				//ADC_EndDetect_16u = ADC_Buf_16u;
				PWM_Grade_Select(PWM_Grade);
				
			//}else if(Slope_ADC_16u < 85){
				//if(PWM_Grade > 2){
				//	Grade_tmp_8u = PWM_Grade - 1;
				//	PWM_Grade_Select(PWM_Grade);
				//}
				
			}else if(Slope_ADC_16u < 70){
				//if(PWM_Grade > 2){
				//	Grade_tmp_8u = PWM_Grade - 2;
					PWM_Grade_Select(2);
				//}
				
			}else if(Slope_ADC_16u < 90){
				PWM_Grade_Select(1);
				
			}else {
				PWM_Grade_Select(0);
				Flag_Slope_Hi_8u = TRUE;
				//Door_Stop();
				//TM_CLOSE = 0;
				//TM_OPEN = 0;
			}
			
			if(Flag_Slope_Hi_8u == TRUE){
	//			if(ADC_AVE_16u <= ADC_StandBy_16u){
				if(ST_Slope_8u == 1){
					Door_Stop();
					TM_CLOSE = 0;
					TM_OPEN = 0;
					ST_OpEnd_8u = 0;
					
					//ADC�B��̤j�Ȧs��
					Flag_Door_UpLimit   = FALSE;
					Flag_Door_DownLimit = TRUE;
					//���ARELAY����
					Flag2_Door_UpLimit_8u   = FALSE;
					Flag2_Door_DownLimit_8u = TRUE;
					//�����Ӯɭp��
					Time_Remain_Close_16u = TM_MAX - TM_CLOSE_A;
					//OenKey���A�E��
					ST_ONEKEY_8u = 4;
					//�����ʧ@�E��RST
					ST_Anti = 0;
					ADC_Detect_Start_Flag = 2;		//�B�൲���åB�x�sAD��: Operate_ADC_Detect
				}
			}
			
			break;
				
		default:
			break;
	}
	
	printf("\n\n\r=================================");
	printf("\n\rTM_CLOSE = %d",TM_CLOSE);
	printf("\n\rTM_Dly_InitSample_16u = %d",TM_Dly_InitSample_16u);
	printf("\n\rST_OpEnd_8u = %d",ST_OpEnd_8u);
	printf("\n\rST_End_Detect_P1_8u = %d",ST_End_Detect_P1_8u);
	printf("\n\rADC_StandBy_16u = %d",ADC_StandBy_16u);
	printf("\n\rADC_AVE_16u = %d",ADC_AVE_16u);
	printf("\n\rADC_EndDetect_16u = %d",ADC_EndDetect_16u);
	printf("\n\rADC_Buf_16u = %d",ADC_Buf_16u);
	printf("\n\rSlope_ADC_8u = %d",Slope_ADC_16u);
	printf("\n\rFlag_Slope_Hi_8u = %d",Flag_Slope_Hi_8u);
	printf("\n\rPWM_Ratio = %3.1f",100*((float)PWM_Duty/PWM_Period));

}

//�����������찻��
static void OpEnd_Detect_2(void){
	uint16_t TM_OPEN_A;//,TM_CLOSE_A;

	if(TM_EndDetec == 0 && Flag_WindowsDoor == TRUE){
		if(OpEnd_Detect_Start_Flag == FALSE){                  //����
			TM_DoorOperateDly = 5;                             //Delay 0.5 second��}�l�T�{�O�_�쭭��
			OpEnd_Detect_Start_Flag = TRUE;
			ADC_Detect_Start_Flag = 1;		                   //ADC�B����^���}�l:Operate_ADC_Detect
		}else if(TM_OPEN > 0){
			
			ADC_TMP_16u = ADC_AVE_16u;
			Voc = (float)ADC_TMP_16u *(3.3/4096);		
			
			//if(Voc > Volt_StandBy_32f && Voc > Volt_StandBy_b_32f && TM_DoorOperateDly == 0){	//���q������P�w
			//	TM_Vstb_Extend_8u = Time_Vstb_Extend_8u;
			//}
			
			//if((Voc <= Volt_StandBy_32f && Voc >= Volt_StandBy_b_32f) && 
			//   (TM_DoorOperateDly == 0 && TM_Vstb_Extend_8u == 0)){

			if((Voc <= Volt_StandBy_32f && HAL_GPIO_ReadPin(PORT_OC, PIN_OC) == RESET)&&// && Voc >= Volt_StandBy_b_32f) && 
			   (TM_DoorOperateDly == 0 && TM_Vstb_Extend_8u == 0)){
				
				ADC_OpEnd_16u = ADC_TMP_16u;
				
				TM_OPEN_A = TM_OPEN;
				TM_OPEN = 0;
				TM_CLOSE = 0;
				Door_Stop();

				printf("\n\n\r�����-����B��!\n\n");
				
				//�P�_�O�_�}�����,�åB�]�w�ө��ɶ�
				if(TM_OPEN > 0){
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
				Flag2_Door_UpLimit_8u   = TRUE;
				Flag2_Door_DownLimit_8u = FALSE;
				Flag3_Door_UpLimit_8u = TRUE;
				Flag4_Door_UpLimit_8u = TRUE;
				Door_Close();	//�}���ϦV�٨�

				//�B��ɶ��p��
				Time_Remain_Open_16u = TM_MAX - TM_OPEN_A;
				ST_ONEKEY_8u = 2;
				Door_Stop();	//�ϦV�٨���ߧY����B��
				printf("\n\n\r �}��STOP");
				printf("\n\n\r");

				//�B�স��
				//REC_Operate_Times_32u++;
				ST_Save_8u = 1;
				
				ST_Close = 1;

				TM_OPEN = 0;
				TM_CLOSE = 0;
				ST_Anti = 0;
				OpEnd_Detect_Start_Flag = FALSE;		
				ADC_Detect_Start_Flag = 2;		//�B�൲���åB�x�sAD��: Operate_ADC_Detect
				
				//ST_ONEKEY_8u = 2;
			}
		}else if(TM_CLOSE > 0){			
			Voc = ADC_AVE_16u *(3.3/4096);		
						
			if(Voc > Volt_StandBy_32f && Voc > Volt_StandBy_b_32f && TM_DoorOperateDly == 0){	//���q������P�w
				TM_Vstb_Extend_8u = Time_Vstb_Extend_8u;
			}
			
			if((Voc <= Volt_StandBy_32f && Voc >= Volt_StandBy_b_32f) && 
			   (TM_DoorOperateDly == 0 && TM_Vstb_Extend_8u == 0)){
				printf("\n\n\r�����-����B��!\n\n");
												
				//����X�а���
				Flag_Door_UpLimit   = FALSE;
				Flag_Door_DownLimit = TRUE;
				Flag2_Door_UpLimit_8u   = FALSE;
				Flag2_Door_DownLimit_8u = TRUE;
				
				if(ST_Close == 1){
					Time_Remain_Close_16u = TM_WindowsDoor_ClosePart1 - TM_CLOSE;
				}else{
					Time_Remain_Close_16u = 0;
				}
				
				TM_OPEN = 0;
				TM_CLOSE = 0;
				ST_Anti = 0;
				OpEnd_Detect_Start_Flag = FALSE;
								
				ST_ONEKEY_8u = 4;
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
	
	//����LED
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
	
	
	//���߱���A or B�ҥή�: Enable

	if(Flag_Rly_ACT_A_8u > 0 || Flag_Rly_ACT_B_8u > 0){
		GPIO_InitStruct.Pin = RL_ACT;
		HAL_GPIO_Init(PORT_Status_Out, &GPIO_InitStruct);
	}
	if(Flag_Rly_TME_A_8u > 0 || Flag_Rly_TME_B_8u > 0){
		GPIO_InitStruct.Pin = RL_TME;
		HAL_GPIO_Init(PORT_Status_Out, &GPIO_InitStruct);
	}
	if(Flag_Rly_POS_A_8u > 0 || Flag_Rly_POS_B_8u > 0){
		GPIO_InitStruct.Pin = RL_POS;
		HAL_GPIO_Init(PORT_Status_Out, &GPIO_InitStruct);
	}
	
	//GPIO_InitStruct.Pin = RL_ACT | RL_TME | RL_POS;
	//HAL_GPIO_Init(PORT_Status_Out, &GPIO_InitStruct);
		
	//Initial condition setting: OFF.
	Relay_ACT_OFF();
	Relay_TME_OFF();
	Relay_POS_OFF();
	
	//HAL_GPIO_WritePin(PORT_Status_Out, RL_ACT, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(PORT_Status_Out, RL_TME, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(PORT_Status_Out, RL_POS, GPIO_PIN_RESET);
}

static void StatusRelay_Control(void){
	//Relay TME ����
	SR_CTRL_TME_A();
	SR_CTRL_TME_B();
	SR_TERMINATE_TME();
	
	//Relay ACT ����
	SR_CTRL_ACT_A();
	SR_CTRL_ACT_B();
	SR_TERMINATE_ACT();

	//Relay POS ����
	SR_CTRL_POS_A();
	SR_CTRL_POS_B();
	SR_TERMINATE_POS();

	//Relay ��X����
	SR_OUTPUT_CTRL();
	
	//�P�_�ΰѼ�RESET
	SR_STVAR_RST();
	
	//Edge�ܼƽw�İ�
	SR_VAR_BUF();
}

//Ĳ�o����A
static void SR_CTRL_TME_A(void){
	//Ĳ�o����P�_
	if(Rly_TME_A_8u == FALSE){
		switch(Flag_Rly_TME_A_8u){
			case SR_Uplimit:
				if(Flag2_Door_UpLimit_8u == TRUE){
					ST_RlyEvent_TME_A_8u = 1;	
					//Flag2_Door_UpLimit_8u = FALSE;
				}else{
					
				}
				
				break;
			
			case SR_Downlimit:
				if(Flag2_Door_DownLimit_8u == TRUE){
					ST_RlyEvent_TME_A_8u = 1;	
					//Flag2_Door_DownLimit_8u = FALSE;
				}else{
					
				}

				break;
			
			case SR_MidStop:
				if( TM_OPEN == 0  && 
					TM_CLOSE == 0 &&
				   (TMP_TM_OPEN_16u != 0 || TMP_TM_CLOSE_16u != 0)){
					   if(Flag2_Door_UpLimit_8u != TRUE && Flag2_Door_DownLimit_8u != TRUE){
							ST_RlyEvent_TME_A_8u = 1;
					   }
				}else{
					//empty
				}			
				//SR_VAR_BUF();
				break;
			
			case SR_Open:
				if(TM_OPEN > 0 && TMP_TM_OPEN_16u == 0){
					ST_RlyEvent_TME_A_8u = 1;	
				}else{
					//empty
				}
				//SR_VAR_BUF();
				break;
			
			case SR_Down:
				if(TM_CLOSE > 0 && TMP_TM_CLOSE_16u == 0){
					ST_RlyEvent_TME_A_8u = 1;	
				}else{
					//empty
				}
				//SR_VAR_BUF();
				break;
			
			case SR_CmdOpen:
				if(BTST(Trig_RM_8u,BIT0) != 0){
					ST_RlyEvent_TME_A_8u = 1;	
					//BCLR(Trig_RM_8u,BIT0);
				}else{
					//empty
				}
				break;
			
			case SR_CmdStop:
				if(BTST(Trig_RM_8u,BIT1) != 0){
					ST_RlyEvent_TME_A_8u = 1;	
					//BCLR(Trig_RM_8u,BIT1);
				}else{
					//empty
				}
				break;
			
			case SR_CmdClose:
				if(BTST(Trig_RM_8u,BIT2) != 0){
					ST_RlyEvent_TME_A_8u = 1;	
					//BCLR(Trig_RM_8u,BIT2);
				}else{
					//empty
				}
				break;
			
			case SR_CmdLock:
				if(Flag_LOCK == TRUE && TMP_Flag_LOCK_8u == FALSE){
					ST_RlyEvent_TME_A_8u = 1;	
				}else{
					//empty
				}
				//SR_VAR_BUF();			
				break;
			
			default:
				//Empty
				break;
		}
		//=========================================//
		//��X���ݮɶ�
		switch(ST_RlyEvent_TME_A_8u){
			case 0:
				//empty
				break;
			
			case 1:
				TM_RlyEventDelay_TME_A_16u = Time_RlyEvent_TME_A_16u;
				ST_RlyEvent_TME_A_8u = 2;
				break;
			
			case 2:
				if(TM_RlyEventDelay_TME_A_16u == 0){
					ST_RlyEvent_TME_A_8u = 3;
				}
				break;
			
			case 3:
				if(TM_Relay_TME_16u == 0){
					TM_Relay_TME_16u = Time_RlyOp_TME_16u;
				}else{
					//Empty
				}
				ST_RlyEvent_TME_A_8u = 0;
				Rly_TME_A_8u = TRUE;
				break;
			
			default:
				//empty
				break;
		}
		//=========================================//
	}
}

//Ĳ�o����B
static void SR_CTRL_TME_B(void){
	//Ĳ�o����P�_
	if(Rly_TME_B_8u == FALSE){
		switch(Flag_Rly_TME_B_8u){
			case SR_Uplimit:
				if(Flag2_Door_UpLimit_8u == TRUE){
					ST_RlyEvent_TME_B_8u = 1;	
					//Flag2_Door_UpLimit_8u = FALSE;
				}else{
					
				}
				
				break;
			
			case SR_Downlimit:
				if(Flag2_Door_DownLimit_8u == TRUE){
					ST_RlyEvent_TME_B_8u = 1;	
					//Flag2_Door_DownLimit_8u = FALSE;
				}else{
					
				}

				break;
			
			case SR_MidStop:
				if( TM_OPEN == 0  && 
					TM_CLOSE == 0 &&
				   (TMP_TM_OPEN_16u != 0 || TMP_TM_CLOSE_16u != 0)){
					   if(Flag2_Door_UpLimit_8u != TRUE && Flag2_Door_DownLimit_8u != TRUE){
							ST_RlyEvent_TME_B_8u = 1;
					   }
				}else{
					//empty
				}			
				//SR_VAR_BUF();
				break;
			
			case SR_Open:
				if(TM_OPEN > 0 && TMP_TM_OPEN_16u == 0){
					ST_RlyEvent_TME_B_8u = 1;	
				}else{
					//empty
				}
				//SR_VAR_BUF();
				break;
			
			case SR_Down:
				if(TM_CLOSE > 0 && TMP_TM_CLOSE_16u == 0){
					ST_RlyEvent_TME_B_8u = 1;	
				}else{
					//empty
				}
				//SR_VAR_BUF();
				break;
			
			case SR_CmdOpen:
				if(BTST(Trig_RM_8u,BIT0) != 0){
					ST_RlyEvent_TME_B_8u = 1;	
					//BCLR(Trig_RM_8u,BIT0);
				}else{
					//empty
				}
				break;
			
			case SR_CmdStop:
				if(BTST(Trig_RM_8u,BIT1) != 0){
					ST_RlyEvent_TME_B_8u = 1;	
					//BCLR(Trig_RM_8u,BIT1);
				}else{
					//empty
				}
				break;
			
			case SR_CmdClose:
				if(BTST(Trig_RM_8u,BIT2) != 0){
					ST_RlyEvent_TME_B_8u = 1;	
					//BCLR(Trig_RM_8u,BIT2);
				}else{
					//empty
				}
				break;
			
			case SR_CmdLock:
				if(Flag_LOCK == TRUE && TMP_Flag_LOCK_8u == FALSE){
					ST_RlyEvent_TME_B_8u = 1;	
				}else{
					//empty
				}
				//SR_VAR_BUF();			
				break;
			
			default:
				//Empty
				break;
		}
		//=========================================//
		//��X���ݮɶ�
		switch(ST_RlyEvent_TME_B_8u){
			case 0:
				//empty
				break;
			
			case 1:
				TM_RlyEventDelay_TME_B_16u = Time_RlyEvent_TME_B_16u;
				ST_RlyEvent_TME_B_8u = 2;
				break;
			
			case 2:
				if(TM_RlyEventDelay_TME_B_16u == 0){
					ST_RlyEvent_TME_B_8u = 3;
				}
				break;
			
			case 3:
				if(TM_Relay_TME_16u == 0){
					TM_Relay_TME_16u = Time_RlyOp_TME_16u;
				}else{
					//Empty
				}
				ST_RlyEvent_TME_B_8u = 0;
				Rly_TME_B_8u = TRUE;
				break;
			
			default:
				//empty
				break;
		}
		//=========================================//
	}
}

static void SR_CTRL_ACT_A(void){
	//Ĳ�o����P�_
	if(Rly_ACT_A_8u == FALSE){
		switch(Flag_Rly_ACT_A_8u){
			case SR_Uplimit:
				if(Flag2_Door_UpLimit_8u == TRUE){
					ST_RlyEvent_ACT_A_8u = 1;	
					//Flag2_Door_UpLimit_8u = FALSE;
				}else{
					
				}
				
				break;
			
			case SR_Downlimit:
				if(Flag2_Door_DownLimit_8u == TRUE){
					ST_RlyEvent_ACT_A_8u = 1;	
					//Flag2_Door_DownLimit_8u = FALSE;
				}else{
					
				}

				break;
			
			case SR_MidStop:
				if( TM_OPEN == 0  && 
					TM_CLOSE == 0 &&
				   (TMP_TM_OPEN_16u != 0 || TMP_TM_CLOSE_16u != 0)){
					   if(Flag2_Door_UpLimit_8u != TRUE && Flag2_Door_DownLimit_8u != TRUE){
							ST_RlyEvent_ACT_A_8u = 1;
					   }
				}else{
					//empty
				}			
				//SR_VAR_BUF();
				break;
			
			case SR_Open:
				if(TM_OPEN > 0 && TMP_TM_OPEN_16u == 0){
					ST_RlyEvent_ACT_A_8u = 1;	
				}else{
					//empty
				}
				//SR_VAR_BUF();
				break;
			
			case SR_Down:
				if(TM_CLOSE > 0 && TMP_TM_CLOSE_16u == 0){
					ST_RlyEvent_ACT_A_8u = 1;	
				}else{
					//empty
				}
				//SR_VAR_BUF();
				break;
			
			case SR_CmdOpen:
				if(BTST(Trig_RM_8u,BIT0) != 0){
					ST_RlyEvent_ACT_A_8u = 1;	
					//BCLR(Trig_RM_8u,BIT0);
				}else{
					//empty
				}
				break;
			
			case SR_CmdStop:
				if(BTST(Trig_RM_8u,BIT1) != 0){
					ST_RlyEvent_ACT_A_8u = 1;	
					//BCLR(Trig_RM_8u,BIT1);
				}else{
					//empty
				}
				break;
			
			case SR_CmdClose:
				if(BTST(Trig_RM_8u,BIT2) != 0){
					ST_RlyEvent_ACT_A_8u = 1;	
					//BCLR(Trig_RM_8u,BIT2);
				}else{
					//empty
				}
				break;
			
			case SR_CmdLock:
				if(Flag_LOCK == TRUE && TMP_Flag_LOCK_8u == FALSE){
					ST_RlyEvent_ACT_A_8u = 1;	
				}else{
					//empty
				}
				//SR_VAR_BUF();			
				break;
			
			default:
				//Empty
				break;
		}
		//=========================================//
		//��X���ݮɶ�
		switch(ST_RlyEvent_ACT_A_8u){
			case 0:
				//empty
				break;
			
			case 1:
				TM_RlyEventDelay_ACT_A_16u = Time_RlyEvent_ACT_A_16u;
				ST_RlyEvent_ACT_A_8u = 2;
				break;
			
			case 2:
				if(TM_RlyEventDelay_ACT_A_16u == 0){
					ST_RlyEvent_ACT_A_8u = 3;
				}
				break;
			
			case 3:
				if(TM_Relay_ACT_16u == 0){
					TM_Relay_ACT_16u = Time_RlyOp_ACT_16u;
				}else{
					//Empty
				}
				ST_RlyEvent_ACT_A_8u = 0;
				Rly_ACT_A_8u = TRUE;
				break;
			
			default:
				//empty
				break;
		}
		//=========================================//
	}
}

//Ĳ�o����B
static void SR_CTRL_ACT_B(void){
	//Ĳ�o����P�_
	if(Rly_ACT_B_8u == FALSE){
		switch(Flag_Rly_ACT_B_8u){
			case SR_Uplimit:
				if(Flag2_Door_UpLimit_8u == TRUE){
					ST_RlyEvent_ACT_B_8u = 1;	
					//Flag2_Door_UpLimit_8u = FALSE;
				}else{
					
				}
				
				break;
			
			case SR_Downlimit:
				if(Flag2_Door_DownLimit_8u == TRUE){
					ST_RlyEvent_ACT_B_8u = 1;	
					//Flag2_Door_DownLimit_8u = FALSE;
				}else{
					
				}

				break;
			
			case SR_MidStop:
				if( TM_OPEN == 0  && 
					TM_CLOSE == 0 &&
				   (TMP_TM_OPEN_16u != 0 || TMP_TM_CLOSE_16u != 0)){
					   if(Flag2_Door_UpLimit_8u != TRUE && Flag2_Door_DownLimit_8u != TRUE){
							ST_RlyEvent_ACT_B_8u = 1;
					   }
				}else{
					//empty
				}			
				//SR_VAR_BUF();
				break;
			
			case SR_Open:
				if(TM_OPEN > 0 && TMP_TM_OPEN_16u == 0){
					ST_RlyEvent_ACT_B_8u = 1;	
				}else{
					//empty
				}
				//SR_VAR_BUF();
				break;
			
			case SR_Down:
				if(TM_CLOSE > 0 && TMP_TM_CLOSE_16u == 0){
					ST_RlyEvent_ACT_B_8u = 1;	
				}else{
					//empty
				}
				//SR_VAR_BUF();
				break;
			
			case SR_CmdOpen:
				if(BTST(Trig_RM_8u,BIT0) != 0){
					ST_RlyEvent_ACT_B_8u = 1;	
					//BCLR(Trig_RM_8u,BIT0);
				}else{
					//empty
				}
				break;
			
			case SR_CmdStop:
				if(BTST(Trig_RM_8u,BIT1) != 0){
					ST_RlyEvent_ACT_B_8u = 1;	
					//BCLR(Trig_RM_8u,BIT1);
				}else{
					//empty
				}
				break;
			
			case SR_CmdClose:
				if(BTST(Trig_RM_8u,BIT2) != 0){
					ST_RlyEvent_ACT_B_8u = 1;	
					//BCLR(Trig_RM_8u,BIT2);
				}else{
					//empty
				}
				break;
			
			case SR_CmdLock:
				if(Flag_LOCK == TRUE && TMP_Flag_LOCK_8u == FALSE){
					ST_RlyEvent_ACT_B_8u = 1;	
				}else{
					//empty
				}
				//SR_VAR_BUF();			
				break;
			
			default:
				//Empty
				break;
		}
		//=========================================//
		//��X���ݮɶ�
		switch(ST_RlyEvent_ACT_B_8u){
			case 0:
				//empty
				break;
			
			case 1:
				TM_RlyEventDelay_ACT_B_16u = Time_RlyEvent_ACT_B_16u;
				ST_RlyEvent_ACT_B_8u = 2;
				break;
			
			case 2:
				if(TM_RlyEventDelay_ACT_B_16u == 0){
					ST_RlyEvent_ACT_B_8u = 3;
				}
				break;
			
			case 3:
				if(TM_Relay_ACT_16u == 0){
					TM_Relay_ACT_16u = Time_RlyOp_ACT_16u;
				}else{
					//Empty
				}
				ST_RlyEvent_ACT_B_8u = 0;
				Rly_ACT_B_8u = TRUE;
				break;
			
			default:
				//empty
				break;
		}
		//=========================================//
	}
}

static void SR_CTRL_POS_A(void){
	//Ĳ�o����P�_
	if(Rly_POS_A_8u == FALSE){
		switch(Flag_Rly_POS_A_8u){
			case SR_Uplimit:
				if(Flag2_Door_UpLimit_8u == TRUE){
					ST_RlyEvent_POS_A_8u = 1;	
					//Flag2_Door_UpLimit_8u = FALSE;
				}else{
					
				}
				
				break;
			
			case SR_Downlimit:
				if(Flag2_Door_DownLimit_8u == TRUE){
					ST_RlyEvent_POS_A_8u = 1;	
					//Flag2_Door_DownLimit_8u = FALSE;
				}else{
					
				}

				break;
			
			case SR_MidStop:
				if( TM_OPEN == 0  && 
					TM_CLOSE == 0 &&
				   (TMP_TM_OPEN_16u != 0 || TMP_TM_CLOSE_16u != 0)){
					   if(Flag2_Door_UpLimit_8u != TRUE && Flag2_Door_DownLimit_8u != TRUE){
							ST_RlyEvent_POS_A_8u = 1;
					   }
				}else{
					//empty
				}			
				//SR_VAR_BUF();
				break;
			
			case SR_Open:
				if(TM_OPEN > 0 && TMP_TM_OPEN_16u == 0){
					ST_RlyEvent_POS_A_8u = 1;	
				}else{
					//empty
				}
				//SR_VAR_BUF();
				break;
			
			case SR_Down:
				if(TM_CLOSE > 0 && TMP_TM_CLOSE_16u == 0){
					ST_RlyEvent_POS_A_8u = 1;	
				}else{
					//empty
				}
				//SR_VAR_BUF();
				break;
			
			case SR_CmdOpen:
				if(BTST(Trig_RM_8u,BIT0) != 0){
					ST_RlyEvent_POS_A_8u = 1;	
					//BCLR(Trig_RM_8u,BIT0);
				}else{
					//empty
				}
				break;
			
			case SR_CmdStop:
				if(BTST(Trig_RM_8u,BIT1) != 0){
					ST_RlyEvent_POS_A_8u = 1;	
					//BCLR(Trig_RM_8u,BIT1);
				}else{
					//empty
				}
				break;
			
			case SR_CmdClose:
				if(BTST(Trig_RM_8u,BIT2) != 0){
					ST_RlyEvent_POS_A_8u = 1;	
					//BCLR(Trig_RM_8u,BIT2);
				}else{
					//empty
				}
				break;
			
			case SR_CmdLock:
				if(Flag_LOCK == TRUE && TMP_Flag_LOCK_8u == FALSE){
					ST_RlyEvent_POS_A_8u = 1;	
				}else{
					//empty
				}
				//SR_VAR_BUF();			
				break;
			
			default:
				//Empty
				break;
		}
		//=========================================//
		//��X���ݮɶ�
		switch(ST_RlyEvent_POS_A_8u){
			case 0:
				//empty
				break;
			
			case 1:
				TM_RlyEventDelay_POS_A_16u = Time_RlyEvent_POS_A_16u;
				ST_RlyEvent_POS_A_8u = 2;
				break;
			
			case 2:
				if(TM_RlyEventDelay_POS_A_16u == 0){
					ST_RlyEvent_POS_A_8u = 3;
				}
				break;
			
			case 3:
				if(TM_Relay_POS_16u == 0){
					TM_Relay_POS_16u = Time_RlyOp_POS_16u;
				}else{
					//Empty
				}
				ST_RlyEvent_POS_A_8u = 0;
				Rly_POS_A_8u = TRUE;
				break;
			
			default:
				//empty
				break;
		}
		//=========================================//
	}
}

//Ĳ�o����B
static void SR_CTRL_POS_B(void){
	//Ĳ�o����P�_
	if(Rly_POS_B_8u == FALSE){
		switch(Flag_Rly_POS_B_8u){
			case SR_Uplimit:
				if(Flag2_Door_UpLimit_8u == TRUE){
					ST_RlyEvent_POS_B_8u = 1;	
					//Flag2_Door_UpLimit_8u = FALSE;
				}else{
					
				}
				
				break;
			
			case SR_Downlimit:
				if(Flag2_Door_DownLimit_8u == TRUE){
					ST_RlyEvent_POS_B_8u = 1;	
					//Flag2_Door_DownLimit_8u = FALSE;
				}else{
					
				}

				break;
			
			case SR_MidStop:
				if( TM_OPEN == 0  && 
					TM_CLOSE == 0 &&
				   (TMP_TM_OPEN_16u != 0 || TMP_TM_CLOSE_16u != 0)){
					   if(Flag2_Door_UpLimit_8u != TRUE && Flag2_Door_DownLimit_8u != TRUE){
							ST_RlyEvent_POS_B_8u = 1;
					   }
				}else{
					//empty
				}			
				//SR_VAR_BUF();
				break;
			
			case SR_Open:
				if(TM_OPEN > 0 && TMP_TM_OPEN_16u == 0){
					ST_RlyEvent_POS_B_8u = 1;	
				}else{
					//empty
				}
				//SR_VAR_BUF();
				break;
			
			case SR_Down:
				if(TM_CLOSE > 0 && TMP_TM_CLOSE_16u == 0){
					ST_RlyEvent_POS_B_8u = 1;	
				}else{
					//empty
				}
				//SR_VAR_BUF();
				break;
			
			case SR_CmdOpen:
				if(BTST(Trig_RM_8u,BIT0) != 0){
					ST_RlyEvent_POS_B_8u = 1;	
					//BCLR(Trig_RM_8u,BIT0);
				}else{
					//empty
				}
				break;
			
			case SR_CmdStop:
				if(BTST(Trig_RM_8u,BIT1) != 0){
					ST_RlyEvent_POS_B_8u = 1;	
					//BCLR(Trig_RM_8u,BIT1);
				}else{
					//empty
				}
				break;
			
			case SR_CmdClose:
				if(BTST(Trig_RM_8u,BIT2) != 0){
					ST_RlyEvent_POS_B_8u = 1;	
					//BCLR(Trig_RM_8u,BIT2);
				}else{
					//empty
				}
				break;
			
			case SR_CmdLock:
				if(Flag_LOCK == TRUE && TMP_Flag_LOCK_8u == FALSE){
					ST_RlyEvent_POS_B_8u = 1;	
				}else{
					//empty
				}
				//SR_VAR_BUF();			
				break;
			
			default:
				//Empty
				break;
		}
		//=========================================//
		//��X���ݮɶ�
		switch(ST_RlyEvent_POS_B_8u){
			case 0:
				//empty
				break;
			
			case 1:
				TM_RlyEventDelay_POS_B_16u = Time_RlyEvent_POS_B_16u;
				ST_RlyEvent_POS_B_8u = 2;
				break;
			
			case 2:
				if(TM_RlyEventDelay_POS_B_16u == 0){
					ST_RlyEvent_POS_B_8u = 3;
				}
				break;
			
			case 3:
				if(TM_Relay_POS_16u == 0){
					TM_Relay_POS_16u = Time_RlyOp_POS_16u;
				}else{
					//Empty
				}
				ST_RlyEvent_POS_B_8u = 0;
				Rly_POS_B_8u = TRUE;
				break;
			
			default:
				//empty
				break;
		}
		//=========================================//
	}
}

//�Ѱ�����
static void SR_TERMINATE_TME(void){
	if(Rly_TME_A_8u == TRUE || Rly_TME_B_8u == TRUE){
		switch(Flag_Rly_TME_TER_8u){
			case SR_Uplimit:
				if(Flag2_Door_UpLimit_8u == TRUE){
					ST_RlyEvent_TER_TME_8u = 1;
				}
				break;
			
			case SR_Downlimit:
				if(Flag2_Door_DownLimit_8u == TRUE){
					ST_RlyEvent_TER_TME_8u = 1;
				}
				break;
			
			case SR_MidStop:
				if( TM_OPEN == 0  && 
					TM_CLOSE == 0 &&
				   (TMP_TM_OPEN_16u != 0 || TMP_TM_CLOSE_16u != 0)){
					   if(Flag2_Door_UpLimit_8u != TRUE && Flag2_Door_DownLimit_8u != TRUE){
							ST_RlyEvent_TER_TME_8u = 1;
					   }
				}
				break;
			
			case SR_Open:
				if(TM_OPEN > 0 && TMP_TM_OPEN_16u == 0){
					ST_RlyEvent_TER_TME_8u = 1;
				}
				break;
			
			case SR_Down:
				if(TM_CLOSE > 0 && TMP_TM_CLOSE_16u == 0){
					ST_RlyEvent_TER_TME_8u = 1;
				}
				break;
			
			case SR_CmdOpen:
				if(BTST(Trig_RM_8u,BIT0) != 0){
					ST_RlyEvent_TER_TME_8u = 1;
				}
				break;
			
			case SR_CmdStop:
				if(BTST(Trig_RM_8u,BIT1) != 0){
					ST_RlyEvent_TER_TME_8u = 1;
				}
				break;
			
			case SR_CmdClose:
				if(BTST(Trig_RM_8u,BIT2) != 0){
					ST_RlyEvent_TER_TME_8u = 1;
				}
				break;
			
			case SR_CmdLock:
				if(Flag_LOCK == TRUE && TMP_Flag_LOCK_8u == FALSE){
					ST_RlyEvent_TER_TME_8u = 1;
				}
				break;
			
			default:
				//Empty
				break;
		}
		
		switch(ST_RlyEvent_TER_TME_8u){
			case 0:
				//Empty
				break;
				
			case 1:	//�]�w����ɶ�
				TM_RlyEventDelay_TER_TME_16u = Time_RlyEvent_TER_TME_16u;
				ST_RlyEvent_TER_TME_8u = 2;
				break;
				
			case 2:
				if(TM_RlyEventDelay_TER_TME_16u == 0){
					ST_RlyEvent_TER_TME_8u = 3;
				}
				break;
				
			case 3:
				TM_Relay_TME_16u = 0;
				//ST_RlyEvent_TER_TME_8u = 0;
				break;
				
			
			default:
				//Empty
				break;
		}
	}
}

static void SR_TERMINATE_ACT(void){
	if(Rly_ACT_A_8u == TRUE || Rly_ACT_B_8u == TRUE){
		switch(Flag_Rly_ACT_TER_8u){
			case SR_Uplimit:
				if(Flag2_Door_UpLimit_8u == TRUE){
					ST_RlyEvent_TER_ACT_8u = 1;
				}
				break;
			
			case SR_Downlimit:
				if(Flag2_Door_DownLimit_8u == TRUE){
					ST_RlyEvent_TER_ACT_8u = 1;
				}
				break;
			
			case SR_MidStop:
				if( TM_OPEN == 0  && 
					TM_CLOSE == 0 &&
				   (TMP_TM_OPEN_16u != 0 || TMP_TM_CLOSE_16u != 0)){
					   if(Flag2_Door_UpLimit_8u != TRUE && Flag2_Door_DownLimit_8u != TRUE){
							ST_RlyEvent_TER_ACT_8u = 1;
					   }
				}
				break;
			
			case SR_Open:
				if(TM_OPEN > 0 && TMP_TM_OPEN_16u == 0){
					ST_RlyEvent_TER_ACT_8u = 1;
				}
				break;
			
			case SR_Down:
				if(TM_CLOSE > 0 && TMP_TM_CLOSE_16u == 0){
					ST_RlyEvent_TER_ACT_8u = 1;
				}
				break;
			
			case SR_CmdOpen:
				if(BTST(Trig_RM_8u,BIT0) != 0){
					ST_RlyEvent_TER_ACT_8u = 1;
				}
				break;
			
			case SR_CmdStop:
				if(BTST(Trig_RM_8u,BIT1) != 0){
					ST_RlyEvent_TER_ACT_8u = 1;
				}
				break;
			
			case SR_CmdClose:
				if(BTST(Trig_RM_8u,BIT2) != 0){
					ST_RlyEvent_TER_ACT_8u = 1;
				}
				break;
			
			case SR_CmdLock:
				if(Flag_LOCK == TRUE && TMP_Flag_LOCK_8u == FALSE){
					ST_RlyEvent_TER_ACT_8u = 1;
				}
				break;
			
			default:
				//Empty
				break;
		}
		
		switch(ST_RlyEvent_TER_ACT_8u){
			case 0:
				//Empty
				break;
				
			case 1:	//�]�w����ɶ�
				TM_RlyEventDelay_TER_ACT_16u = Time_RlyEvent_TER_ACT_16u;
				ST_RlyEvent_TER_ACT_8u = 2;
				break;
				
			case 2:
				if(TM_RlyEventDelay_TER_ACT_16u == 0){
					ST_RlyEvent_TER_ACT_8u = 3;
				}
				break;
				
			case 3:
				TM_Relay_ACT_16u = 0;
				//ST_RlyEvent_TER_ACT_8u = 0;
				break;
				
			
			default:
				//Empty
				break;
		}
	}
}

static void SR_TERMINATE_POS(void){
	if(Rly_POS_A_8u == TRUE || Rly_POS_B_8u == TRUE){
		switch(Flag_Rly_POS_TER_8u){
			case SR_Uplimit:
				if(Flag2_Door_UpLimit_8u == TRUE){
					ST_RlyEvent_TER_POS_8u = 1;
				}
				break;
			
			case SR_Downlimit:
				if(Flag2_Door_DownLimit_8u == TRUE){
					ST_RlyEvent_TER_POS_8u = 1;
				}
				break;
			
			case SR_MidStop:
				if( TM_OPEN == 0  && 
					TM_CLOSE == 0 &&
				   (TMP_TM_OPEN_16u != 0 || TMP_TM_CLOSE_16u != 0)){
					   if(Flag2_Door_UpLimit_8u != TRUE && Flag2_Door_DownLimit_8u != TRUE){
							ST_RlyEvent_TER_POS_8u = 1;
					   }
				}
				break;
			
			case SR_Open:
				if(TM_OPEN > 0 && TMP_TM_OPEN_16u == 0){
					ST_RlyEvent_TER_POS_8u = 1;
				}
				break;
			
			case SR_Down:
				if(TM_CLOSE > 0 && TMP_TM_CLOSE_16u == 0){
					ST_RlyEvent_TER_POS_8u = 1;
				}
				break;
			
			case SR_CmdOpen:
				if(BTST(Trig_RM_8u,BIT0) != 0){
					ST_RlyEvent_TER_POS_8u = 1;
				}
				break;
			
			case SR_CmdStop:
				if(BTST(Trig_RM_8u,BIT1) != 0){
					ST_RlyEvent_TER_POS_8u = 1;
				}
				break;
			
			case SR_CmdClose:
				if(BTST(Trig_RM_8u,BIT2) != 0){
					ST_RlyEvent_TER_POS_8u = 1;
				}
				break;
			
			case SR_CmdLock:
				if(Flag_LOCK == TRUE && TMP_Flag_LOCK_8u == FALSE){
					ST_RlyEvent_TER_POS_8u = 1;
				}
				break;
			
			default:
				//Empty
				break;
		}
		
		switch(ST_RlyEvent_TER_POS_8u){
			case 0:
				//Empty
				break;
				
			case 1:	//�]�w����ɶ�
				TM_RlyEventDelay_TER_POS_16u = Time_RlyEvent_TER_POS_16u;
				ST_RlyEvent_TER_POS_8u = 2;
				break;
				
			case 2:
				if(TM_RlyEventDelay_TER_POS_16u == 0){
					ST_RlyEvent_TER_POS_8u = 3;
				}
				break;
				
			case 3:
				TM_Relay_POS_16u = 0;
				//ST_RlyEvent_TER_POS_8u = 0;
				break;
				
			
			default:
				//Empty
				break;
		}
	}
}

//�ܼƼȦs��
static void SR_VAR_BUF(void){
	TMP_TM_OPEN_16u = TM_OPEN;
	TMP_TM_CLOSE_16u = TM_CLOSE;
	TMP_Flag_LOCK_8u = Flag_LOCK;
}

static void SR_STVAR_RST(void){
	//���A�ܼ�RESET
	if(Flag2_Door_UpLimit_8u == TRUE){
		Flag2_Door_UpLimit_8u = FALSE;
	}
	if(Flag2_Door_DownLimit_8u == TRUE){
		Flag2_Door_DownLimit_8u = FALSE;
	}
	if(BTST(Trig_RM_8u,BIT0) != 0){
		BCLR(Trig_RM_8u,BIT0);
	}
	if(BTST(Trig_RM_8u,BIT1) != 0){
		BCLR(Trig_RM_8u,BIT1);
	}
	if(BTST(Trig_RM_8u,BIT2) != 0){
		BCLR(Trig_RM_8u,BIT2);
	}

}

static void SR_OUTPUT_CTRL(void){
	if(TM_Relay_TME_16u > 0){
		Relay_TME_ON();
	}else{
		Relay_TME_OFF();
		Rly_TME_A_8u = FALSE;
		Rly_TME_B_8u = FALSE;
		ST_RlyEvent_TER_TME_8u = 0;
	}
	
	if(TM_Relay_ACT_16u > 0){
		Relay_ACT_ON();
	}else{
		Relay_ACT_OFF();
		Rly_ACT_A_8u = FALSE;
		Rly_ACT_B_8u = FALSE;
		ST_RlyEvent_TER_ACT_8u = 0;
	}


	if(TM_Relay_POS_16u > 0){
		Relay_POS_ON();
	}else{
		Relay_POS_OFF();
		Rly_POS_A_8u = FALSE;
		Rly_POS_B_8u = FALSE;
		ST_RlyEvent_TER_POS_8u = 0;
	}

}

// Relay_TME:ON /OFF
void Relay_TME_ON(void){
	HAL_GPIO_WritePin(PORT_Status_Out, RL_TME, GPIO_PIN_SET);
	//printf("\n\r Relay_TME ON!");
}

void Relay_TME_OFF(void){
	HAL_GPIO_WritePin(PORT_Status_Out, RL_TME, GPIO_PIN_RESET);
	//printf("\n\r Relay_TME OFF!");
}

// Relay_ACT:ON /OFF
void Relay_ACT_ON(void){
	HAL_GPIO_WritePin(PORT_Status_Out, RL_ACT, GPIO_PIN_SET);
	//printf("\n\r Relay_ACT ON!");
}

void Relay_ACT_OFF(void){
	HAL_GPIO_WritePin(PORT_Status_Out, RL_ACT, GPIO_PIN_RESET);
	//printf("\n\r Relay_ACT OFF!");
}

// Relay_POS:ON /OFF
void Relay_POS_ON(void){
	HAL_GPIO_WritePin(PORT_Status_Out, RL_POS, GPIO_PIN_SET);
	//printf("\n\r Relay_POS  ON!");
}

void Relay_POS_OFF(void){
	HAL_GPIO_WritePin(PORT_Status_Out, RL_POS, GPIO_PIN_RESET);
	//printf("\n\r Relay_POS OFF!");
}

static void EXTI0_1_IRQHandler_Config(void){
  GPIO_InitTypeDef   GPIO_InitStructure;
	GPIOA_CLK_DISABLE();
	
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;//GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;

  GPIO_InitStructure.Pin = PIN_OC;
  HAL_GPIO_Init(PORT_OC, &GPIO_InitStructure);
	
	HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0);
}

static void EXTI2_3_IRQHandler_Config(void){
  GPIO_InitTypeDef   GPIO_InitStructure;
	EXTI_CTRL_ONEKEY_CLK_ENABLE();
	
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;//GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;

  GPIO_InitStructure.Pin = W_ONEKEY;
  HAL_GPIO_Init(PORT_ONEKEY, &GPIO_InitStructure);
	
	HAL_NVIC_SetPriority(EXTI2_3_IRQn, 1, 0);
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
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
  //HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

//�~�����_:���䰻��
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	printf("\n\r������O: ");
	CNT_Jog_Press = 0;
	CNT_LOCK_Press = 0;
	Flag_JOG = FALSE;
	ST_Door_buf = ST_Door;
	ST_Door_buf2 = ST_Door;
	
	switch(GPIO_Pin){
		case W_STOP:
			
			CNT_Debug_BTN_8u++;
			
		  if(CNT_Debug_BTN_8u == 10){
				ST_BUZZ_8u = 0;
			}else{
				ST_BUZZ_8u = 9;
			}

			if(Flag_SMK  == TRUE)	break;
			if(Flag_LOCK == TRUE){			//��q�\��ON: ���ʧ@
				break;
			}
			ST_BTN = TRUE;
			ACT_Door = 0;
			ST_Anti = 0;
			ADC_Detect_Start_Flag = 0;
			
			if(ST_BUZZ_8u != 9){
				ST_BUZZ_8u = 0;
			}
			
			BSET(Trig_RM_8u,BIT1);

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
			
			//Relay_out: ��������
			//�ʧ@����: ��B�त���U����
			if(TM_OPEN > 0 || TM_CLOSE > 0){
				Flag_Relay_MidStop = TRUE;
			}
			
			if(TM_OPEN > 0){
				ST_ONEKEY_8u = 2;
			}else if(TM_CLOSE > 0){
				ST_ONEKEY_8u = 4;
			}
			
			Anti_Event = 0;
			printf("\n\rSTOP!\n");
			break;
		
		case W_OPEN:
			ST_BUZZ_8u = 9;
			
			CNT_Debug_BTN_8u = 0;
			
			if(Flag_SMK  == TRUE)	break;	//��������ON: ���ʧ@
			if(Flag_LOCK == TRUE){			//��q�\��ON: ���ʧ@

				break;
			}
			
			BSET(Trig_RM_8u,BIT0);

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
							Door_Open_JOG();
						}
						Flag_JOG = TRUE;
						printf("\n\rJOG Mode:OPEN...\n");
						Door_Open_JOG();						
					}
				}
				printf("\n\rContinue = %d\n",CNT_Jog_Press);
				if(Flag_JOG == TRUE){
					Door_Stop_JOG();
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
				//�����\���l��
				ST_Anti = 0;
				TM_AntiDly = Time_AntiDly;
				Times_OverADC = 0;
			}
			//***����P�w END***//
			
			Flag_Relay_OPEN = TRUE;
			
			ST_BTN = TRUE;
			ACT_Door = 1;
			ST_ONEKEY_8u = 1;
			printf("\n\rOPEN!\n");
			break;

		case W_CLOSE:			
			ST_BUZZ_8u = 9;
			CNT_Debug_BTN_8u = 0;

			if(Flag_SMK   == TRUE)	break;		//�����P����Ĳ�o
			if(Anti_Event == 2)     break;      //����������
			if(Flag_LOCK  == TRUE){				//��q�\��ON: ���ʧ@

				break;
			}
			if(Flag_IR    == TRUE){				//���~�uĲ�o: ���ʧ@
				ST_BUZZ_8u = 3;
				break;
			}
			
			BSET(Trig_RM_8u,BIT2);
			
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
						Door_Close_JOG();
						//OpEnd_Detect();
						//...�ثe����������_���\��,
						//�p�G����,�A�NOPEN����챱��[�J
					}
				}
				printf("\n\rContinue = %d\n",CNT_Jog_Press);
				if(Flag_JOG == TRUE){
					Door_Stop_JOG();
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
				//�����\���l��
				ST_Anti = 0;
				TM_AntiDly = Time_AntiDly;
				Times_OverADC = 0;
			}
			//***����P�w END***//
			
			Flag_Relay_CLOSE = TRUE;

			ST_BTN = TRUE;
			ACT_Door = 2;
			ST_ONEKEY_8u = 3;
			printf("\n\rClose!\n");
			break;
		
		case RM_LOCK:
			printf("\n\rLock!\n");
			CNT_Debug_BTN_8u = 0;

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
							ST_BUZZ_8u = 5;
							printf("\n\rLOCK~~~~~~!\n");
						}
					}
				}
				
			}
			break;
		
		case W_IR:	//���~�u����
			if(ST_Door == 2 || TM_CLOSE > 0){
				TM_CLOSE = 0;
				Flag_IR = TRUE;
				Door_Stop();
				Delay_ms(100);
				ST_BTN = TRUE;
				ACT_Door = 1;	 // �}��
				TM_IR_Lock = 30; // ��w�ɶ� 30 sec.
				ST_ONEKEY_8u = 1;
				ST_BUZZ_8u = 2;
			}
			break;
		
		case W_SMK:
			Flag_SMK = TRUE;
			Door_Stop();
			Delay_ms(100);
			ST_BTN = TRUE;
			ACT_Door = 1;	 //�}��
			ST_ONEKEY_8u = 1;
			ST_BUZZ_8u = 4;
			break;
		
		case W_ONEKEY:
			ST_ONEKEY_8u++;
			if(ST_ONEKEY_8u > 4){
				ST_ONEKEY_8u = 1;
			}
			
			ST_BUZZ_8u = 9;
			
			switch(ST_ONEKEY_8u){
				case 1:
					ACT_Door = 1;
					break;
				
				case 2:
					ACT_Door = 0;
					break;
				
				case 3:
					ACT_Door = 2;
					break;
				
				case 4:
					ACT_Door = 0;
					break;
				
				default:
					//empty
					break;
			}
			ST_BTN = TRUE;
			
			break;
		
		case PIN_OC:
				if(TM_OPEN > 0){
					Delay_ms(30);
				}else{
					Delay_ms(190);
				}
				
				if(HAL_GPIO_ReadPin(PORT_OC,PIN_OC) == RESET){
					Door_Stop();
					printf("\n\r OC Work");
				}
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
		}else{
			//empty
		}
	}
}


//	TIM14 handle
//	For PWM output
void HAL_TIM17_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{	
	PWM_Count++;	
	if(PWM_Count == PWM_Period){
		PWM_Count = 0;
	}
	
		//HAL_GPIO_WritePin(PORT_Motor_MOS, MOS_ACT, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(PORT_Motor_MOS, MOS_ACT, GPIO_PIN_RESET);
	
	if(PWM_Count < PWM_Duty){
		HAL_GPIO_WritePin(PORT_Motor_MOS, MOS_ACT, GPIO_PIN_RESET);
	}else{
		HAL_GPIO_WritePin(PORT_Motor_MOS, MOS_ACT, GPIO_PIN_SET);
	}
	
}


//	TIM16 handle
//	For TIMER Down-count
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	Tim_cnt_10ms++;
	Tim_cnt_100ms++;
	Tim_cnt_1s++;
	
	// 0.01 sec
	if(Tim_cnt_10ms == 10){
		TM_Buzz_ON_8u  = TIMDEC(TM_Buzz_ON_8u);
		TM_Buzz_OFF_8u  = TIMDEC(TM_Buzz_OFF_8u);
		TM_Vstb_Extend_8u = TIMDEC(TM_Vstb_Extend_8u);

		Tim_cnt_10ms = 0;
	}
	
	// 0.1 sec
	if(Tim_cnt_100ms == 100){
		TM_OPEN 	      = TIMDEC(TM_OPEN);
		TM_CLOSE 	      = TIMDEC(TM_CLOSE);
		TM_AntiDly 	    = TIMDEC(TM_AntiDly);
		TM_AntiDly2 	  = TIMDEC(TM_AntiDly2);
		TM_AntiDly3 	  = TIMDEC(TM_AntiDly3);
		TM_EndDetec 	  = TIMDEC(TM_EndDetec);
		TM_DoorOperateDly = TIMDEC(TM_DoorOperateDly);
		TM_Printf 	      = TIMDEC(TM_Printf);
		TM_DLY 	          = TIMDEC(TM_DLY);
		TM_Auto_Close     = TIMDEC(TM_Auto_Close);
		TM_Anti_Occur     = TIMDEC(TM_Anti_Occur);
		TM_ADC_Relaod     = TIMDEC(TM_ADC_Relaod);
		TM_CLOSE_EndPart  = TIMDEC(TM_CLOSE_EndPart);

		TM_Relay_TME_16u  = TIMDEC(TM_Relay_TME_16u);
		TM_Relay_ACT_16u  = TIMDEC(TM_Relay_ACT_16u);
		TM_Relay_POS_16u  = TIMDEC(TM_Relay_POS_16u);
		
		TM_RlyEventDelay_TME_A_16u = TIMDEC(TM_RlyEventDelay_TME_A_16u);
		TM_RlyEventDelay_TME_B_16u = TIMDEC(TM_RlyEventDelay_TME_B_16u);
		TM_RlyEventDelay_TER_TME_16u  = TIMDEC(TM_RlyEventDelay_TER_TME_16u);
		
		TM_RlyEventDelay_ACT_A_16u = TIMDEC(TM_RlyEventDelay_ACT_A_16u);
		TM_RlyEventDelay_ACT_B_16u = TIMDEC(TM_RlyEventDelay_ACT_B_16u);
		TM_RlyEventDelay_TER_ACT_16u  = TIMDEC(TM_RlyEventDelay_TER_ACT_16u);
		
		TM_RlyEventDelay_POS_A_16u = TIMDEC(TM_RlyEventDelay_POS_A_16u);
		TM_RlyEventDelay_POS_B_16u = TIMDEC(TM_RlyEventDelay_POS_B_16u);
		TM_RlyEventDelay_TER_POS_16u  = TIMDEC(TM_RlyEventDelay_TER_POS_16u);
		
		TM_Low_Operate    = TIMINC(TM_Low_Operate);
		
		TM_Buzz_16u  = TIMDEC(TM_Buzz_16u);
		
		TM_Dly_InitSample_16u = TIMDEC(TM_Dly_InitSample_16u);
		TM_Idle_16u = TIMDEC(TM_Idle_16u);
		
		Tim_cnt_100ms = 0;

	}
	
	// 1 sec
	if(Tim_cnt_1s == 1000){
		Tim_cnt_1s = 0;
		TM_IR_Lock 	      = TIMDEC(TM_IR_Lock);
		TM_Save 	      	= TIMDEC(TM_Save);
		TM_Save_1st_16u   = TIMDEC(TM_Save_1st_16u);
	}
	
	if(TM_Buzz_ON_8u > 0){
		Buzz_ON();
	}else if(TM_Buzz_OFF_8u > 0){
		Buzz_OFF();
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

	TimHandle17.Init.Period            = 10 - 1;
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
  
	//if(HAL_UART_DeInit(&UartHandle) != HAL_OK){
	//	Error_Handler();
	//}
	//
	//if(HAL_UART_Init(&UartHandle) != HAL_OK){
	//	Error_Handler();
	//}

}

static void Parameter_Load(void){
	uint8_t i;
	float iweight;
	
	if(HAL_I2C_Mem_Read(&I2cHandle,(uint16_t)I2C_ADDRESS, 0, I2C_MEMADD_SIZE_8BIT, (uint8_t*)aRxBuffer, 256, 10000) != HAL_OK){
		if(HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF){
			Error_Handler();
		}
	}
	
	EE_Default = HAL_GPIO_ReadPin(PORT_EE_SEL, EEPROM_SEL);
	
	if(EE_Default == TRUE){
		//Default parameter
		Flag_CycleTest       = FALSE;   //�`������(���ɴ���)
		Flag_WindowsDoor     = FALSE;   //�������\��
		Flag_AntiPress_Open  = TRUE;    //�}�������\��
		Flag_AntiPress_Close = TRUE;    //���������\��
		Flag_AutoClose       = FALSE;   //�۰������\��
		Flag_Func_JOG        = FALSE;   //�T�ʥ\��
		Flag_Motor_Direction = TRUE;   //���F�B���V
		Flag_Remote_Lock     = TRUE;   //��q�\��
		Flag_Rate_Regulate   = FALSE;   //�����ճt
		Flag_Buzzer          = TRUE;    //���ﾹ
		Flag_Light           = FALSE;    //�۰ʷө�
		Flag_Low_Operate     = FALSE;  //�w�_�B & �w����
		
		//EE_Addr_P = 31;


		TM_DLY_Value              = 300;   //�`�����ն��j�ɶ�
		TM_WindowsDoor_ClosePart1 = 130;   //������_�Ĥ@�q�����ɶ�
		TM_MAX                    = 600;   //�}�����̪��B��ɶ�
		Time_Auto_Close           = 100;   //�۰���������ɶ�
		Time_Light                = 100;   //�ө��B��ɶ�

		iWeight_Vstb_8u = 1.2; //�}���۰ʨM�w�ݾ���
		//Volt_StandBy_32f = 0.3;
		Anti_Weight_Open_select  = 5;   //�����v��: �}��
		Anti_Weight_Close_select = 5;   //�����v��: ����
		
		Times_JOG        = 50;   //�T�ʧP�w����
		Times_Remote_Lock = 75;   //��q���ߦ���
		
		PWM_Grade        = 3;   //�K���t��
		
		Time_Low_Operate_Ini = 20;
		Time_Low_Operate_Mid = 80;
		
		//TME-RELAY
		Flag_Rly_TME_A_8u = 0;			//���߱���A
		Flag_Rly_TME_B_8u = 0;			//���߱���B
		Flag_Rly_TME_TER_8u = 0;		//�Ѱ�����
		Time_RlyEvent_TME_A_16u = 10;	//���߮ɶ�A
		Time_RlyEvent_TME_B_16u = 30;	//���߮ɶ�B
		Time_RlyEvent_TER_TME_16u = 15;	//�Ѱ��ɶ�
		Time_RlyOp_TME_16u = 100;		//��X�ɶ�
		
		//ACT-RELAY
		Flag_Rly_ACT_A_8u = 0;			//���߱���A
		Flag_Rly_ACT_B_8u = 0;			//���߱���B
		Flag_Rly_ACT_TER_8u = 0;		//�Ѱ�����
		Time_RlyEvent_ACT_A_16u = 20;	//���߮ɶ�A
		Time_RlyEvent_ACT_B_16u = 20;	//���߮ɶ�B
		Time_RlyEvent_TER_ACT_16u = 10;	//�Ѱ��ɶ�
		Time_RlyOp_ACT_16u = 100;		//��X�ɶ�
		
		//POS-RELAY
		Flag_Rly_POS_A_8u = 0;			//���߱���A
		Flag_Rly_POS_B_8u = 0;			//���߱���B
		Flag_Rly_POS_TER_8u = 0;		//�Ѱ�����
		Time_RlyEvent_POS_A_16u = 30;	//���߮ɶ�A
		Time_RlyEvent_POS_B_16u = 10;	//���߮ɶ�B
		Time_RlyEvent_TER_POS_16u = 5;	//�Ѱ��ɶ�
		Time_RlyOp_POS_16u = 100;		//��X�ɶ�
		
		Door_Select_8u = 1;
		Time_Open_Break_8u = 1;
		
	}else{
		//******Parameter form EEPROM*****//
		EE_Addr_P = 0;
		VER1       = aRxBuffer[EE_Addr_P++];   //�{���s��
		VER2       = aRxBuffer[EE_Addr_P++];   //�{������
		Maintain_Year       = aRxBuffer[EE_Addr_P++];   //���@�ɶ�:�~
		Maintain_Month      = aRxBuffer[EE_Addr_P++];   //���@�ɶ�:��
		Maintain_Day        = aRxBuffer[EE_Addr_P++];   //���@�ɶ�:��
		Warranty_Year       = aRxBuffer[EE_Addr_P++];   //�O�T�ɶ�:�~
		Warranty_Month      = aRxBuffer[EE_Addr_P++];   //�O�T�ɶ�:��
		Warranty_Day        = aRxBuffer[EE_Addr_P++];   //�O�T�ɶ�:��
		
		//���O�Ǹ�
		PN1 = aRxBuffer[EE_Addr_P++];
		PN2 = aRxBuffer[EE_Addr_P++];
		PN3 = aRxBuffer[EE_Addr_P++];
		PN4 = aRxBuffer[EE_Addr_P++];
		PN5 = aRxBuffer[EE_Addr_P++];
		PN6 = aRxBuffer[EE_Addr_P++];
		
		//Funtion ON/OFF
		EE_Addr_P = 20;
		Flag_CycleTest       = aRxBuffer[EE_Addr_P++];   //�`������(���ɴ���)
		Flag_WindowsDoor     = aRxBuffer[EE_Addr_P++];   //�������\��
		Flag_AntiPress_Close = aRxBuffer[EE_Addr_P++];   //�����\��(����)
		Flag_AutoClose       = aRxBuffer[EE_Addr_P++];   //�۰������\��
		Flag_Func_JOG        = aRxBuffer[EE_Addr_P++];   //�T�ʥ\��
		Flag_Motor_Direction = aRxBuffer[EE_Addr_P++];   //���F�B���V
		Flag_Remote_Lock     = aRxBuffer[EE_Addr_P++];   //��q�\��
		Flag_Rate_Regulate   = aRxBuffer[EE_Addr_P++];   //�����ճt
		Flag_Buzzer          = aRxBuffer[EE_Addr_P++];   //���ﾹ
		Flag_Light           = aRxBuffer[EE_Addr_P++];   //�۰ʷө�
		Flag_Low_Operate     = aRxBuffer[EE_Addr_P++];   //�w�_�B & �w����
		Flag_AntiPress_Open  = aRxBuffer[EE_Addr_P++];   //�����\��(�}��)


		EE_Addr_P = 40;
		TM_DLY_Value              = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;   //�`�����ն��j�ɶ�
		EE_Addr_P+=2;
		
		TM_WindowsDoor_ClosePart1 = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;    //�`�����ն��j�ɶ�
		EE_Addr_P+=2;
		
		TM_MAX                    = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;   //�}�����̪��B��ɶ�
		EE_Addr_P+=2;
		
		Time_Auto_Close           = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;   //�۰���������ɶ�
		EE_Addr_P+=2;
		
		Time_Light                = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;   //�ө��B��ɶ�
		EE_Addr_P+=2;
		
		//EE_Addr_P = 50;
		iWeight_Vstb_8u          = (float)aRxBuffer[EE_Addr_P++]/100;   //�ݾ��q���v��for���P�w�ϥ�
		Anti_Weight_Open_select  = aRxBuffer[EE_Addr_P++];   //�����v��: �}��
		Anti_Weight_Close_select = aRxBuffer[EE_Addr_P++];   //�����v��: ����
		
		Times_JOG         = aRxBuffer[EE_Addr_P++];   //�T�ʧP�w����
		Times_Remote_Lock = aRxBuffer[EE_Addr_P++];   //��q���ߦ���
		
		PWM_Grade      = aRxBuffer[EE_Addr_P++];   //�K���t��
		
		//EE_Addr_P = 56;
		//EE_Addr_P = 56;
		

		EE_Addr_P = 58;
		Time_Low_Operate_Ini = aRxBuffer[EE_Addr_P++]; 
		Time_Low_Operate_Mid = aRxBuffer[EE_Addr_P++]*10; 
		
		//�~�q���ɶ��Ѽ� TME/ACT/POS 
		EE_Addr_P = 60;
		//TME���߮ɶ�
		Time_RlyEvent_TME_A_16u = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;   //
		EE_Addr_P+=2;
		
		Time_RlyEvent_TME_B_16u = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;    //
		EE_Addr_P+=2;
		
		//ACT���߮ɶ�
		Time_RlyEvent_ACT_A_16u = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;   //
		EE_Addr_P+=2;
		
		Time_RlyEvent_ACT_B_16u = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;    //
		EE_Addr_P+=2;
		
		//POS���߮ɶ�
		Time_RlyEvent_POS_A_16u = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;   //
		EE_Addr_P+=2;
		
		Time_RlyEvent_POS_B_16u = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;    //
		EE_Addr_P+=2;
		
		//��X�ɶ� TME/ACT/POS
		Time_RlyOp_TME_16u = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;    //
		EE_Addr_P+=2;
		
		Time_RlyOp_ACT_16u = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;   //
		EE_Addr_P+=2;
		
		Time_RlyOp_POS_16u = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;   //
		EE_Addr_P+=2;

		//�Ѱ����߮ɶ� TME/ACT/POS
		Time_RlyEvent_TER_TME_16u = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;    //
		EE_Addr_P+=2;
		
		Time_RlyEvent_TER_ACT_16u = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;   //
		EE_Addr_P+=2;
		
		Time_RlyEvent_TER_POS_16u = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;   //
		EE_Addr_P+=2;
		
		//�ݾ��q���P�w����ɶ�
		EE_Addr_P = 84;
		Time_Vstb_Extend_8u = aRxBuffer[EE_Addr_P++];
		Door_Select_8u      = aRxBuffer[EE_Addr_P++];
		Time_Open_Break_8u  = aRxBuffer[EE_Addr_P++];

		
		//���ARELAY���߱���
		EE_Addr_P = 100;
		Flag_Rly_TME_A_8u    = aRxBuffer[EE_Addr_P++];   //
		Flag_Rly_TME_B_8u    = aRxBuffer[EE_Addr_P++];   //
		Flag_Rly_ACT_A_8u    = aRxBuffer[EE_Addr_P++];   //
		Flag_Rly_ACT_B_8u    = aRxBuffer[EE_Addr_P++];   //
		Flag_Rly_POS_A_8u    = aRxBuffer[EE_Addr_P++];   //
		Flag_Rly_POS_B_8u    = aRxBuffer[EE_Addr_P++];   //
		Flag_Rly_TME_TER_8u  = aRxBuffer[EE_Addr_P++];   //
		Flag_Rly_ACT_TER_8u  = aRxBuffer[EE_Addr_P++];   //
		Flag_Rly_POS_TER_8u  = aRxBuffer[EE_Addr_P++];   //
	}
	
	
	//�����B�榸��
	REC_Operate_Times_32u = 0;
	//REC_Operate_Times_32u = (uint32_t)aRxBuffer[30] | (uint32_t)aRxBuffer[31]<<8 | (uint32_t)aRxBuffer[32]<<16 | (uint32_t)aRxBuffer[33]<<24;
	for(i=0;i<4;i++){
		REC_Operate_Times_32u = REC_Operate_Times_32u | (uint32_t)aRxBuffer[200+i]<<(8*i);
	}
	
	//�}�������v���]�w
	switch(Anti_Weight_Open_select){
		case 1:
			iweight = 0.01;
			break;
		case 2:
			iweight = 0.02;
			break;
		case 3:
			iweight = 0.03;
			break;
		case 4:
			iweight = 0.04;
			break;
		case 5:
			iweight = 0.05;
			break;
		case 6:
			iweight = 0.10;
			break;
		case 7:
			iweight = 0.20;
			break;
		case 8:
			iweight = 0.30;
			break;
		case 9:
			iweight = 0.40;
			break;
		case 10:
			iweight = 0.50;
			break;
		default:
			iweight = 0.05;
			break;
	}
	Anti_Weight_Open = iweight;
	
	//���������v���]�w
	switch(Anti_Weight_Close_select){
		case 1:
			iweight = 0.01;
			break;
		case 2:
			iweight = 0.02;
			break;
		case 3:
			iweight = 0.03;
			break;
		case 4:
			iweight = 0.04;
			break;
		case 5:
			iweight = 0.05;
			break;
		case 6:
			iweight = 0.10;
			break;
		case 7:
			iweight = 0.20;
			break;
		case 8:
			iweight = 0.30;
			break;
		case 9:
			iweight = 0.40;
			break;
		case 10:
			iweight = 0.50;
			break;
		default:
			iweight = 0.05;
			break;
	}
	Anti_Weight_Close = iweight;
	
	//PWM�t�׿��
	if(Flag_Rate_Regulate == FALSE){
		PWM_Grade = 3;
	}
	
	PWM_Grade_Select(PWM_Grade);
	
	//�ݾ��q���]�w
	//iWeight_Vstb_8u = 1.05;
	ADC_Calculate();
	ADC_StandBy_b_16u = ADC_AVE_16u;
	ADC_StandBy_16u = (uint16_t)((float)ADC_StandBy_b_16u * iWeight_Vstb_8u);
	Volt_StandBy_b_32f = (float)ADC_StandBy_b_16u *(3.3/4096)*0.95;
	Volt_StandBy_32f = (float)ADC_StandBy_b_16u *(3.3/4096) * iWeight_Vstb_8u;
	
	if(Flag_AntiPress_Open == TRUE || Flag_AntiPress_Close == TRUE){
		Flag_AntiPress = TRUE;
	}else{
		Flag_AntiPress = FALSE;
	}
	
	switch(Door_Select_8u){
		case 0:	//�p��
			ADC_Anti_MAX_STD_8u = 2200;
			break;
			
		case 1: //����
			ADC_Anti_MAX_STD_8u = 2390;
			break;
			
		case 2: //�j��
			ADC_Anti_MAX_STD_8u = 4000;
			break;
			
		default:
			ADC_Anti_MAX_STD_8u = 2390;
			break;
	
	}
}

static void Parameter_List(void){
	//int i,j;
	Buzz_OFF();
  //�{���̫�ק���
	printf("\n\r*************************************"); 
	printf("\n\r*************************************"); 
	printf("\n\r** Final Modify Date: %d     **", Ver_date);
	printf("\n\r** Model: TYG-NCP-R01              **");
	printf("\n\r** Version: H%dV%d T%d               **",VER1,VER2,VER3); 
	printf("\n\r*************************************"); 
	printf("\n\r*************************************"); 

	printf("\n\r============================");
	printf("\n\r==========�ѼƳ]�w==========");
	printf("\n\r============================");
	printf("\n\r*****�\��}��(0:���� / 1:�}��)");
	printf("\n\r ��������  : %d", Flag_CycleTest);
	printf("\n\r �������\��: %d", Flag_WindowsDoor);
	printf("\n\r �����\��  : %d", Flag_AntiPress);
	printf("\n\r �۰�����  : %d (�Ҧ�)", Flag_AutoClose);
	printf("\n\r �T�ʥ\��  : %d", Flag_Func_JOG);
	printf("\n\r �B���V  : %d", Flag_Motor_Direction);
	printf("\n\r ��q�\��  : %d", Flag_Remote_Lock);
	printf("\n\r ��������  : %d", Flag_Rate_Regulate);
	printf("\n\r ���ܭ�    : %d", Flag_Buzzer);
	printf("\n\r �۰ʷө�  : %d", Flag_Light);
	printf("\n\r �w�Ұʥ\��: %d", Flag_Low_Operate);
	printf("\n\r �����\��(�})  : %d", Flag_AntiPress_Open);
	printf("\n\r �����\��(��)  : %d", Flag_AntiPress_Close);
	
	printf("\n\n\r*****�B��Ѽ�");
	printf("\n\r �}�����̤j�B��ɶ�      : %4.1f ��", TM_MAX *0.1);
	printf("\n\r �������ն}�������j�ɶ�  : %4.1f ��", TM_DLY_Value *0.1);
	printf("\n\r �����������ɶ�(Part 1)  : %4.1f ��", TM_WindowsDoor_ClosePart1 *0.1);
	printf("\n\r �۰������ɶ�            : %4.1f ��", Time_Auto_Close *0.1);
	printf("\n\r �ө��ɶ�                : %4.1f ��", Time_Light *0.1);
	printf("\n\r �ݾ��q�����ߩ���ɶ�     : %4.1f ��", Time_Vstb_Extend_8u *0.1);
	printf("\n\r �w�B�B��(��1�q)         : %4.1f ��", Time_Low_Operate_Ini *0.1);
	printf("\n\r �w�B�B��(��2�q)         : %4.1f ��", Time_Low_Operate_Mid *0.1);

	printf("\n\r �ݾ�Volt                : %1.2f(V)", Volt_StandBy_32f);
	printf("\n\r �����v��(OPEN)          : %2.3f", Anti_Weight_Open);
	printf("\n\r �����v��(CLOSE)         : %2.3f", Anti_Weight_Close);
	
	printf("\n\r �T�ʧP�w�Ѽ�    : %d", Times_JOG);
	printf("\n\r ��q�P�w�Ѽ�    : %d", Times_Remote_Lock);
	
	printf("\n\r �B��t��(0~3)   : %d", PWM_Grade);
	printf("\n\r PWM Duty Cycle  : %3.1f", (float)PWM_Duty*100/PWM_Period);
	
	printf("\n\n\r�B�স��  :%d (��)", REC_Operate_Times_32u);
	
	printf("\n\n\r ���j�p���        :%d", Door_Select_8u);
	printf("\n\r ���������ʧ@��(AD):%d", ADC_Anti_MAX_STD_8u);
	printf("\n\r ���������ʧ@��(A) :%2.3f", ADC_Anti_MAX_STD_8u*(3.3*200)/(4096*34));
	
	printf("\n\n\r �}���٨��ʧ@�ɶ�  :%2.1f", Time_Open_Break_8u *0.1);
	
	//�~�����ARelay�Ѽ�
	printf("\n\n\r*****State-Relay parameter*****");
	printf("\n\r===TME-Relay===");
	printf("\n\r ���߱���A: %d", Flag_Rly_TME_A_8u);
	printf("\n\r ���߱���B: %d", Flag_Rly_TME_B_8u);
	printf("\n\r �Ѱ����� : %d", Flag_Rly_TME_TER_8u);
	printf("\n\r ���߮ɶ�A: %4.1f ��", Time_RlyEvent_TME_A_16u*0.1);
	printf("\n\r ���߮ɶ�B: %4.1f ��", Time_RlyEvent_TME_B_16u*0.1);
	printf("\n\r �Ѱ��ɶ� : %4.1f ��", Time_RlyEvent_TER_TME_16u*0.1);
	printf("\n\r ��X�ɶ� : %4.1f ��", Time_RlyOp_TME_16u*0.1);

	printf("\n\n\r===ACT-Relay===");
	printf("\n\r ���߱���A: %d", Flag_Rly_ACT_A_8u);
	printf("\n\r ���߱���B: %d", Flag_Rly_ACT_B_8u);
	printf("\n\r �Ѱ����� : %d", Flag_Rly_ACT_TER_8u);
	printf("\n\r ���߮ɶ�A: %4.1f ��", Time_RlyEvent_ACT_A_16u*0.1);
	printf("\n\r ���߮ɶ�B: %4.1f ��", Time_RlyEvent_ACT_B_16u*0.1);
	printf("\n\r �Ѱ��ɶ� : %4.1f ��", Time_RlyEvent_TER_ACT_16u*0.1);
	printf("\n\r ��X�ɶ� : %4.1f ��", Time_RlyOp_ACT_16u*0.1);

	printf("\n\n\r===POS-Relay===");
	printf("\n\r ���߱���A: %d", Flag_Rly_POS_A_8u);
	printf("\n\r ���߱���B: %d", Flag_Rly_POS_B_8u);
	printf("\n\r �Ѱ����� : %d", Flag_Rly_POS_TER_8u);
	printf("\n\r ���߮ɶ�A: %4.1f ��", Time_RlyEvent_POS_A_16u*0.1);
	printf("\n\r ���߮ɶ�B: %4.1f ��", Time_RlyEvent_POS_B_16u*0.1);
	printf("\n\r �Ѱ��ɶ� : %4.1f ��", Time_RlyEvent_TER_POS_16u*0.1);
	printf("\n\r ��X�ɶ� : %4.1f ��", Time_RlyOp_POS_16u*0.1);
	printf("\n\n\r*****State-Relay parameter End*****");
	printf("\n\n\r========�ѼƳ]�w End========");
	
	Buzz_OFF();
	printf("\n\n\r======== EEPROM Print ========");	
	for(i=0;i<32;i++){
		printf("\n\r R%03d: ",i*8);
		for(j=i*8;j<(i*8+8);j++){
			printf("%02X, ",aRxBuffer[j]);
		}
	}
	printf("\n\n\r======== EEPROM Finish ========");	

}


static void Anti_Pressure_5(void){
	uint16_t ADC_Buf;
	
	if(Flag_AntiPress == TRUE &&
		 (ADC_OPEN_MAX_16u != 0 && ADC_CLOSE_MAX_16u != 0)){
		
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
					
				}else if(TM_AntiDly == 0){
					//�ھڨ����Ѽ��v��,�]�w�����ʧ@��
					if(TM_OPEN > 0 && TM_CLOSE > 0){
					
					}else if(TM_OPEN > 0){
						//Anti_Weight = 1 + (Anti_Weight_Open / 10);
						Anti_Weight = 1 + Anti_Weight_Open;
						ADC_Anti_Max = ADC_OPEN_MAX_16u * Anti_Weight;
						ST_Anti = 2;
					}else if(TM_CLOSE > 0){
						//Anti_Weight = 1 + (Anti_Weight_Close / 10);
						Anti_Weight = 1 + Anti_Weight_Close;
						ADC_Anti_Max = ADC_CLOSE_MAX_16u * Anti_Weight;
						ST_Anti = 3;
					}
					
					TM_AntiDly2 = 10; //Delay 0.1 second��}�l����
				}
				break;
			
			case 2:	//�}����������
				if(Flag_AntiPress_Open == FALSE){
					ST_Anti = 0;
				}
				
				if(TM_AntiDly2 == 0){

					ADC_Buf = ADC_AVE_16u;	//Ū����eAD��

					//�P�_��eAD��	
					if(ADC_Buf >= ADC_Anti_Max || ADC_Buf >= ADC_Anti_MAX_STD_8u){
						Times_OverADC++;				
					}else{
						Times_OverADC = 0;
					}
					
					//printf("\r\n\n ADC_Buf = %d, ADC_Anti_Max = %d",ADC_Buf,ADC_Anti_Max);
					
					if(Times_OverADC >= Times_OverADC_Target){
						ST_Anti = 4;
						Anti_Event = 1; 	// OPEN����ON
						TM_Anti_Occur = 100; //10�����ɶ��]�w
						
						//����
						TM_OPEN = 0;
						TM_CLOSE  = 0;
						
						TM_AntiDly3 = 0; 	//�B�ఱ��,�K����,���ݵ��ݮɶ�
						
						ST_BUZZ_8u = 6;
					}else{
						//ST_Anti = 2;
						Anti_Event = 0; 	// ���`�B��
						TM_AntiDly2 = 3;	//�U�^�����������ݮɶ�
					}
				}
				break;
			
			case 3:	//������������
				if(Flag_AntiPress_Close == FALSE){
					ST_Anti = 0;
				}
				
				if(ST_Close == 2){
					ST_Anti = 0;
				}else if(TM_AntiDly2 == 0){

					ADC_Buf = ADC_AVE_16u;	//Ū����eADC��

					
					//�p��B��q���ܤ�
										
					if(ADC_Buf >= ADC_Anti_Max || ADC_Buf >= ADC_Anti_MAX_STD_8u){
						Times_OverADC++;				
					}else{
						Times_OverADC = 0;
					}
									
					if(Times_OverADC == Times_OverADC_Target){
						ST_Anti = 4;
						TM_AntiDly3 = 1;
						
						if(TM_CLOSE_EndPart > 0){
							Anti_Event = 2; 	// CLOSE����ON
							//����
							TM_OPEN = 0;
							TM_CLOSE = 0;
							TM_AntiDly3 = 1;	//���y0.1���,����}��
							ST_BUZZ_8u = 7;
						}
					}else{
						//ST_Anti = 2;
						Anti_Event = 0; 	// ���`�B��
						TM_AntiDly2 = 1;	//�U�^�����������ݮɶ�
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
	uint16_t i;
	uint16_t Voc_Buf;
	uint32_t adc_amnt_32u = 0;
	uint16_t x_tmp[ADC_CONVERTED_DATA_BUFFER_SIZE];
	uint16_t y_tmp;
	uint16_t Array_size = ADC_CONVERTED_DATA_BUFFER_SIZE;
	uint8_t  Sample_size = 24;
			
	//�ƻs��eAD��
	for(i=0;i<32;i++){
		x_tmp[i] = aADCxConvertedData[i];
	}
	
	//�ƧǤp->�j
	for(i=0;i<Array_size;i++){
		for(j=0;j<=i;j++){
			if(x_tmp[i] < x_tmp[j]){
				y_tmp = x_tmp[j];
				x_tmp[j] = x_tmp[i];
				x_tmp[i] = y_tmp;
			}
		}
	}
	
	//�o�i:�h���Y���U4��
	for(i=0;i<=(Sample_size-1);i++){
		adc_amnt_32u = adc_amnt_32u + x_tmp[i+3];
	}		
	
	//������
	ADC_AVE_16u = (uint16_t)((float)adc_amnt_32u / Sample_size);
	//printf("\n\r adc_amnt_32u:%d / %d",adc_amnt_32u,ADC_AVE_16u);
	
	return Voc_Buf;
}


static void Buzzer_Config(void){
	if(Flag_Buzzer == TRUE){
		GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

		GPIO_InitStruct.Pin = Buzzer;
		HAL_GPIO_Init(PORT_Buzzer, &GPIO_InitStruct);
	}
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

 //Name: Buzz_ON
 //Description: Buzzer ON
static void Buzz_ON(void){
	HAL_GPIO_WritePin(PORT_Buzzer, Buzzer, GPIO_PIN_SET);
}

//Name: Buzz_OFF
//Description: Buzzer OFF
static void Buzz_OFF(void){
	HAL_GPIO_WritePin(PORT_Buzzer, Buzzer, GPIO_PIN_RESET);
}

 
static void Buzzer_CTRL(void){
		
	if(ST_BUZZ_8u != ST_BUZZ_Buf_8u){
		ST_BUZZ_A_8u = 0;
	}
	switch(ST_BUZZ_8u){
		case 1:	//��e�q
			if(ST_BUZZ_A_8u == 0){
				TM_Buzz_ON_8u = 5*10;
				TM_Buzz_16u = 8;
				ST_BUZZ_A_8u = 1;
			}
			
			if(TM_Buzz_ON_8u > 0){
				Buzz_ON();
			}else if(TM_Buzz_OFF_8u > 0){
				Buzz_OFF();
			}
			
			if(TM_Buzz_16u == 0){
				ST_BUZZ_8u = 0;
				ST_BUZZ_A_8u = 0;
			}
			
			if(TM_Buzz_ON_8u == 0 && TM_Buzz_ON_Buf_8u != 0){
				TM_Buzz_OFF_8u = 5*10;
			}
			
			break;
			
		case 2://���~�uĲ�o+�}����
			if(ST_BUZZ_A_8u == 0){
				TM_Buzz_ON_8u = 5*10;
				TM_Buzz_16u = 300;
				ST_BUZZ_A_8u = 1;
			}
			
			if(TM_Buzz_ON_8u > 0){
				Buzz_ON();
			}else if(TM_Buzz_OFF_8u > 0){
				Buzz_OFF();
			}
			
			if(TM_Buzz_16u == 0){
				ST_BUZZ_8u = 0;
				ST_BUZZ_A_8u = 0;
			}
			
			if(TM_Buzz_ON_8u == 0 && TM_Buzz_ON_Buf_8u != 0){
				TM_Buzz_OFF_8u = 5*10;
			}else if(TM_Buzz_OFF_8u == 0 && TM_Buzz_OFF_Buf_8u != 0){
				TM_Buzz_ON_8u = 5*10;
			}
			break;
			
		case 3://���~�uĲ�o+�������
			if(ST_BUZZ_A_8u == 0){
				TM_Buzz_ON_8u = 2*10;
				TM_Buzz_16u = 100;
				ST_BUZZ_A_8u = 1;
			}
			
			if(TM_Buzz_ON_8u > 0){
				Buzz_ON();
			}else if(TM_Buzz_OFF_8u > 0){
				Buzz_OFF();
			}
			
			if(TM_Buzz_16u == 0){
				ST_BUZZ_8u = 0;
				ST_BUZZ_A_8u = 0;
			}
			
			if(TM_Buzz_ON_8u == 0 && TM_Buzz_ON_Buf_8u != 0){
				TM_Buzz_OFF_8u = 2*10;
			}else if(TM_Buzz_OFF_8u == 0 && TM_Buzz_OFF_Buf_8u != 0){
				TM_Buzz_ON_8u = 2*10;
			}
			break;
			
		case 4://�����P����ON
			if(ST_BUZZ_A_8u == 0){
				TM_Buzz_ON_8u = 5*10;
				//TM_Buzz_16u = 100;
				ST_BUZZ_A_8u = 1;
			}
			
			if(TM_Buzz_ON_8u > 0){
				Buzz_ON();
			}else if(TM_Buzz_OFF_8u > 0){
				Buzz_OFF();
			}
			
			//if(TM_Buzz_16u == 0){
			if(Flag3_Door_UpLimit_8u == TRUE){
				ST_BUZZ_8u = 0;
				ST_BUZZ_A_8u = 0;
				Flag3_Door_UpLimit_8u = FALSE;
			}
			
			if(TM_Buzz_ON_8u == 0 && TM_Buzz_ON_Buf_8u != 0){
				TM_Buzz_OFF_8u = 5*10;
			}else if(TM_Buzz_OFF_8u == 0 && TM_Buzz_OFF_Buf_8u != 0){
				TM_Buzz_ON_8u = 5*10;
			}
			break;
			
		case 5://��qON
			if(ST_BUZZ_A_8u == 0){
				TM_Buzz_ON_8u = 5*10;
				CNT_Buzz_8u = 2;
				ST_BUZZ_A_8u = 1;
			}
			
			if(TM_Buzz_ON_8u > 0){
				Buzz_ON();
			}else if(TM_Buzz_OFF_8u > 0){
				Buzz_OFF();
			}
			
			if(CNT_Buzz_8u == 0){
				ST_BUZZ_8u = 0;
				ST_BUZZ_A_8u = 0;
			}
			
			if(TM_Buzz_ON_8u == 0 && TM_Buzz_ON_Buf_8u != 0){
				TM_Buzz_OFF_8u = 5*10;
			}else if(TM_Buzz_OFF_8u == 0 && TM_Buzz_OFF_Buf_8u != 0){
				TM_Buzz_ON_8u = 5*10;
				CNT_Buzz_8u--;
			}
			break;
			
		case 6://����ON:�}��
			if(ST_BUZZ_A_8u == 0){
				TM_Buzz_ON_8u = 5*10;
				TM_Buzz_16u = 100;
				ST_BUZZ_A_8u = 1;
			}
			
			if(TM_Buzz_ON_8u > 0){
				Buzz_ON();
			}else if(TM_Buzz_OFF_8u > 0){
				Buzz_OFF();
			}
			
			if(TM_Buzz_16u == 0){
				ST_BUZZ_8u = 0;
				ST_BUZZ_A_8u = 0;
			}
			
			if(TM_Buzz_ON_8u == 0 && TM_Buzz_ON_Buf_8u != 0){
				TM_Buzz_OFF_8u = 5*10;
			}else if(TM_Buzz_OFF_8u == 0 && TM_Buzz_OFF_Buf_8u != 0){
				TM_Buzz_ON_8u = 5*10;
			}
			break;
			
		case 7://����ON:����
			if(ST_BUZZ_A_8u == 0){
				TM_Buzz_ON_8u = 2*10;
				TM_Buzz_16u = 100;
				ST_BUZZ_A_8u = 1;
			}
			
			if(TM_Buzz_ON_8u > 0){
				Buzz_ON();
			}else if(TM_Buzz_OFF_8u > 0){
				Buzz_OFF();
			}
			
			//if(TM_Buzz_16u == 0){
			if(Flag3_Door_UpLimit_8u == TRUE){
				ST_BUZZ_8u = 0;
				ST_BUZZ_A_8u = 0;
				Flag3_Door_UpLimit_8u = FALSE;
			}
			
			if(TM_Buzz_ON_8u == 0 && TM_Buzz_ON_Buf_8u != 0){
				TM_Buzz_OFF_8u = 2*10;
			}else if(TM_Buzz_OFF_8u == 0 && TM_Buzz_OFF_Buf_8u != 0){
				TM_Buzz_ON_8u = 2*10;
			}

			break;
		
		case 8://�۰�����
			if(ST_BUZZ_A_8u == 0){
				TM_Buzz_ON_8u = 5*10;
				TM_Buzz_16u = 50;
				ST_BUZZ_A_8u = 1;
			}
			
			if(TM_Buzz_ON_8u > 0){
				Buzz_ON();
			}else if(TM_Buzz_OFF_8u > 0){
				Buzz_OFF();
			}
			
			if(TM_Buzz_16u == 0){
				ST_BUZZ_8u = 0;
				ST_BUZZ_A_8u = 0;
			}
			
			if(TM_Buzz_ON_8u == 0 && TM_Buzz_ON_Buf_8u != 0){
				TM_Buzz_OFF_8u = 10*10;
			}else if(TM_Buzz_OFF_8u == 0 && TM_Buzz_OFF_Buf_8u != 0){
				TM_Buzz_ON_8u = 5*10;
			}
		
			break;
			
		case 9:	//������O(OPEN/CLOSE)
			if(ST_BUZZ_A_8u == 0){
				TM_Buzz_ON_8u = 3*10;
				TM_Buzz_16u = 5;
				ST_BUZZ_A_8u = 1;
			}
			
			if(TM_Buzz_ON_8u > 0){
				Buzz_ON();
			}else if(TM_Buzz_OFF_8u > 0){
				Buzz_OFF();
			}
			
			if(TM_Buzz_16u == 0){
				ST_BUZZ_8u = 0;
				ST_BUZZ_A_8u = 0;
			}
			
			if(TM_Buzz_ON_8u == 0 && TM_Buzz_ON_Buf_8u != 0){
				TM_Buzz_OFF_8u = 3*10;
			}
			
			break;
			
		case 10:	//bibi���n
			if(ST_BUZZ_A_8u == 0){
				TM_Buzz_ON_8u = 2*10;
				TM_Buzz_16u = 7;
				ST_BUZZ_A_8u = 1;
			}
			
			if(TM_Buzz_ON_8u > 0){
				Buzz_ON();
			}else if(TM_Buzz_OFF_8u > 0){
				Buzz_OFF();
			}
			
			if(TM_Buzz_16u == 0){
				ST_BUZZ_8u = 0;
				ST_BUZZ_A_8u = 0;
			}
			
			if(TM_Buzz_ON_8u == 0 && TM_Buzz_ON_Buf_8u != 0){
				TM_Buzz_OFF_8u = 2*10;
			}else if(TM_Buzz_OFF_8u == 0 && TM_Buzz_OFF_Buf_8u != 0){
				TM_Buzz_ON_8u = 2*10;
			}
			
			break;
			
		default:
			ST_BUZZ_A_8u = 0;
			Buzz_OFF();
			Flag3_Door_UpLimit_8u = FALSE;
			break;
	}
	
	ST_BUZZ_Buf_8u = ST_BUZZ_8u;
	TM_Buzz_ON_Buf_8u = TM_Buzz_ON_8u;
	TM_Buzz_OFF_Buf_8u = TM_Buzz_OFF_8u;
	
}

//Name: TEST_LED
//Description: LED ON
static void LED_ON(void){
	HAL_GPIO_WritePin(PORT_TEST, TEST_PIN, GPIO_PIN_RESET);
}

//Description: LED OFF
static void LED_OFF(void){
	HAL_GPIO_WritePin(PORT_TEST, TEST_PIN, GPIO_PIN_SET);
}

static void LED_CTRL(void){
	//if(ADC_OPEN_MAX_16u <= 500 || ADC_CLOSE_MAX_16u <= 500){
	//	LED_ON();
	//}else if(ADC_OPEN_MAX_16u >= ADC_Anti_MAX_STD_8u || ADC_CLOSE_MAX_16u >= ADC_Anti_MAX_STD_8u){
	//	LED_ON();
	if(ADC_OPEN_MAX_16u == 0 || ADC_CLOSE_MAX_16u == 0){
		LED_ON();
	}else{
		LED_OFF();
	}

}

//Debug �\��}������
static void Dubug_CTRL(void){
	if(CNT_Debug_BTN_8u == 10){
		if(Flag_Debug_8u == TRUE){
			Flag_Debug_8u = FALSE;
		}else{
			Flag_Debug_8u = TRUE;
		}

		if(Flag_Debug_8u == TRUE){
			ST_BUZZ_8u = 0;
			Buzz_OFF();
			Buzz_OFF();
			Buzz_OFF();
			Buzz_OFF();
			Buzz_OFF();
			//Buzzer_CTRL();
			//Delay_ms(100);
			Fun_Debug_Enable();
			Parameter_List();
			ST_BUZZ_8u = 10;
		}else{
			Fun_Debug_Disable();
			ST_BUZZ_8u = 10;
		}

		CNT_Debug_BTN_8u = 0;
	}
}

static void Fun_Debug_Enable(void){
  
  //UartHandle.Instance        = USARTx;
  //
  //UartHandle.Init.BaudRate   = 9600;
  //UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  //UartHandle.Init.StopBits   = UART_STOPBITS_1;
  //UartHandle.Init.Parity     = UART_PARITY_NONE;
  //UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  //UartHandle.Init.Mode       = UART_MODE_TX_RX;
  //UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  
  if(HAL_UART_Init(&UartHandle) != HAL_OK){
  	Error_Handler();
  }
}

static void Fun_Debug_Disable(void){
  
  //UartHandle.Instance        = USARTx;
  //
  //UartHandle.Init.BaudRate   = 9600;
  //UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  //UartHandle.Init.StopBits   = UART_STOPBITS_1;
  //UartHandle.Init.Parity     = UART_PARITY_NONE;
  //UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  //UartHandle.Init.Mode       = UART_MODE_TX_RX;
  //UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  
  if(HAL_UART_DeInit(&UartHandle) != HAL_OK){
  	Error_Handler();
  }
}

static void Fun_Break_AFT_Open(void){
	if(TM_OPEN == 0 && TM_OPEN_Buf_16u != 0){
		if(Flag4_Door_UpLimit_8u == TRUE){
			TM_CLOSE = (uint16_t)Time_Open_Break_8u;
			Flag4_Door_UpLimit_8u = FALSE;
		}
	}
}


static void PWM_Grade_Select(uint8_t Grade_8u){
	switch(Grade_8u){
		case 0:
			PWM_Duty = 1;
			PWM_Period = 2;
			break;
			
		case 1:
			PWM_Duty = 3;
			PWM_Period = 4;
			break;
			
		case 2:
			PWM_Duty = 4;
			PWM_Period = 5;
			break;
		
		case 3:
			PWM_Duty = 99;
			PWM_Period = 100;
			break;
		
		default:
			PWM_Duty = 4;
			PWM_Period = 5;
			break;
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
