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
//*******參數設定*******//
uint8_t VER1, VER2;
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
uint8_t Flag_WindowsDoor; 		//捲窗門選擇:	
uint8_t Flag_CycleTest;             //循環測試(長時測試)
uint8_t Flag_AntiPress;			//防夾功能
uint8_t Times_JOG;				//長按判定次數: 吋動判定次數, 大於:吋動, 小於:一鍵
uint8_t Times_Remote_Lock;
uint8_t PWM_Grade;	
uint8_t Flag_Rate_Regulate;
uint8_t Flag_Buzzer;
uint8_t Flag_AutoClose;			//自動關門功能
uint8_t Flag_Func_JOG;				//吋動功能
uint8_t Flag_Motor_Direction;		//馬達運轉方向
uint8_t Flag_Remote_Lock;				//鎖電功能
uint8_t Flag_Low_Operate;				//

uint16_t TM_MAX;                  //開關門最長運轉時間 TM_MAX * 100ms
uint16_t TM_WindowsDoor_ClosePart1;			 // 捲窗門第一段關門時間: n*0.1sec
uint16_t TM_DLY;							//cycle-test等待秒數(*100ms)
uint16_t TM_DLY_Value;
uint16_t Time_Auto_Close;			// 自動關門延遲時間: n * 0.1sec.
uint16_t Time_Light;				//照明運轉時間n * 0.1sec

uint8_t Anti_Weight_Open_select;					//防夾權重(可小數):開門(越小越靈敏),建議>1
float Anti_Weight_Open;					//防夾權重(可小數):開門(越小越靈敏),建議>1
uint8_t Anti_Weight_Close_select;				//防夾權重(可小數):關門(越小越靈敏),建議>1
float Anti_Weight_Close;				//防夾權重(可小數):關門(越小越靈敏),建議>1
float Volt_StandBy, Volt_StandBy_b;				//待機電壓(填0為初次啟動偵測),建議值0.3~0.5

//*******參數設定結束*******//

/* Private variables ---------------------------------------------------------*/
	//Boolean
uint8_t ST_BTN;                            //(Remote) controller trigger(0:standby, 1:cmd trigger
uint8_t Open_IT;                           //Interrupt in open
uint8_t Close_IT;                          //Interrupt in close 2-1
uint8_t Close_IT2;                         //Interrupt in close 2-2
uint8_t OpEnd_Detect_Start_Flag = FALSE;
uint8_t Anti_flg2 = TRUE;
uint8_t AClose_Flg = FALSE;				// 自動關門動作旗標
uint8_t Wait_flg;
uint8_t Flag_LOCK = FALSE;					//鎖電動作旗標,預設FALSE,控制器可動作.
uint8_t Flag_JOG;					//吋動動作旗標
uint8_t Flag_Light;
uint8_t Flag_IR;
uint8_t Flag_SMK;
uint8_t Flag_Door_UpLimit;
uint8_t Flag_Door_DownLimit;
uint8_t Flag_No_VSB; //The default stand-by volt is ZERO.
uint8_t Flag_Relay_MidStop;
uint8_t Flag_Relay_OPEN;
uint8_t Flag_Relay_CLOSE;

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
//uint8_t Auto_Close_Mode;
uint16_t Auto_Close_Mode;
uint8_t CNT_Jog_Press = 0;
uint8_t CNT_LOCK_Press = 0;
uint8_t ADC_Detect_Start_Flag = 0;
uint8_t Times_OverADC = 0;
uint8_t Times_OverADC_Target = 2;	//防夾被觸發次數
uint8_t Anti_Event = 0;				//防夾觸發類別 0:正常運轉 1:開門防夾 2:關門防夾
uint8_t Anti_Event_buf = 0;				//防夾觸發類別 0:正常運轉 1:開門防夾 2:關門防夾
uint8_t ST_Low_Operate = 0;	

uint8_t EE_Addr_P = 0;
	//16-bits

uint16_t TM_Printf = 10;
static uint16_t aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];	  //Variable containing ADC conversions data
uint16_t TM_OPEN = 0;                   //Time: Door open
uint16_t TM_CLOSE = 0;                  //Time: Door close
uint16_t TM_CLOSE_EndPart = 0;                  //Time: Door close
uint16_t TM_CLOSE_b = 0;                  //Time: Door close
uint16_t TM_AntiDly;
uint16_t Time_AntiDly = 20;
uint16_t TM_AntiDly2;
uint16_t TM_AntiDly3;
uint16_t TM_EndDetec;
uint16_t TM_DoorOperateDly = 20;        //到位偵測延遲時間(*100ms)
uint16_t OpenTM_Remain = 0;             //兩段式開門剩餘時間
uint16_t CloseTM_Remain = 0;            //兩段式關門剩餘時間
uint16_t TM_Light_ON = 0;
uint16_t TM_Auto_Close = 0;
uint16_t CloseTM2;
uint16_t Tim_cnt_1s = 0;
uint16_t Tim_cnt_100ms = 0;
uint16_t Tim_cnt_10ms = 0;

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
uint16_t TM_Save = 2*60*60;			//運轉次數儲存ˊ週期
uint16_t TM_ADC_Relaod = 0;
uint16_t TM_Low_Operate = 0;

uint16_t Time_Remain_Open = 0;                   
uint16_t Time_Remain_Close = 0;          

uint8_t Time_Low_Operate_Ini;
uint8_t Time_Low_Operate_Mid;


	//32-bits
uint32_t RLY_Delay_ms = 20;			   //Relay_Delay_time(*1ms)
uint32_t uwPrescalerValue = 0;         // Prescaler declaration
uint32_t Cycle_times_up = 0;
uint32_t Cycle_times_down = 0;
uint32_t Ver_date = 20210313;
uint32_t REC_Operate_Times;

uint8_t TXBuf[4];

	//Float
float Voc_base,Voc_base_;
float Voc_base_2;
float Vo1,Vo2;
float V_Diff,V_Diff_1,V_Diff_2;
float Anti_Weight;
float Vadc_ave;

//===============================================================//
//繼電器TME 
//EEPROM
uint8_t Flag_Rly_TME_A_8u;			//TME成立條件A
uint8_t Flag_Rly_TME_B_8u;			//TME成立條件B
uint8_t Flag_Rly_TME_TER_8u;		//TME解除條件
uint16_t Time_RlyEvent_TME_A_16u;	//繼電器TME成立時間A
uint16_t Time_RlyEvent_TME_B_16u;	//繼電器TME成立時間B
uint16_t Time_RlyEvent_TER_TME_16u;	//繼電器TME解除時間
uint16_t Time_RlyOp_TME_16u;			//繼電器TME動作時間

//輸出Relay
uint8_t Rly_TME_A_8u;
uint8_t Rly_TME_B_8u;
uint8_t ST_RlyEvent_TME_A_8u;
uint8_t ST_RlyEvent_TME_B_8u;
uint8_t ST_RlyEvent_TER_TME_8u;
uint16_t TM_RlyEventDelay_TME_A_16u;
uint16_t TM_RlyEventDelay_TME_B_16u;
uint16_t TM_RlyEventDelay_TER_TME_16u;

//Relay輸出判斷TIMER
uint16_t TM_Relay_TME_16u;

//===============================================================//
//繼電器ACT 
//EEPROM
uint8_t Flag_Rly_ACT_A_8u;			//ACT成立條件A
uint8_t Flag_Rly_ACT_B_8u;			//ACT成立條件B
uint8_t Flag_Rly_ACT_TER_8u;		//ACT解除條件
uint16_t Time_RlyEvent_ACT_A_16u;	//繼電器ACT成立時間A
uint16_t Time_RlyEvent_ACT_B_16u;	//繼電器ACT成立時間B
uint16_t Time_RlyEvent_TER_ACT_16u;	//繼電器ACT解除時間
uint16_t Time_RlyOp_ACT_16u;			//繼電器ACT動作時間

//輸出Relay
uint8_t Rly_ACT_A_8u;
uint8_t Rly_ACT_B_8u;
uint8_t ST_RlyEvent_ACT_A_8u;
uint8_t ST_RlyEvent_ACT_B_8u;
uint8_t ST_RlyEvent_TER_ACT_8u;
uint16_t TM_RlyEventDelay_ACT_A_16u;
uint16_t TM_RlyEventDelay_ACT_B_16u;
uint16_t TM_RlyEventDelay_TER_ACT_16u;

//Relay輸出判斷TIMER
uint16_t TM_Relay_ACT_16u;
//===============================================================//
//繼電器POS 
//EEPROM
uint8_t Flag_Rly_POS_A_8u;			//POS成立條件A
uint8_t Flag_Rly_POS_B_8u;			//POS成立條件B
uint8_t Flag_Rly_POS_TER_8u;		//POS解除條件
uint16_t Time_RlyEvent_POS_A_16u;	//繼電器POS成立時間A
uint16_t Time_RlyEvent_POS_B_16u;	//繼電器POS成立時間B
uint16_t Time_RlyEvent_TER_POS_16u;	//繼電器POS解除時間
uint16_t Time_RlyOp_POS_16u;			//繼電器POS動作時間

//輸出Relay
uint8_t Rly_POS_A_8u;
uint8_t Rly_POS_B_8u;
uint8_t ST_RlyEvent_POS_A_8u;
uint8_t ST_RlyEvent_POS_B_8u;
uint8_t ST_RlyEvent_TER_POS_8u;
uint16_t TM_RlyEventDelay_POS_A_16u;
uint16_t TM_RlyEventDelay_POS_B_16u;
uint16_t TM_RlyEventDelay_TER_POS_16u;

//Relay輸出判斷TIMER
uint16_t TM_Relay_POS_16u;


//共用: 緩衝暫存區
uint8_t TMP_Flag_LOCK_8u;
uint16_t TMP_TM_OPEN_16u;
uint16_t TMP_TM_CLOSE_16u;

//共用
uint8_t Flag2_Door_UpLimit_8u;
uint8_t Flag2_Door_DownLimit_8u;
uint8_t Trig_RM_8u;


//蜂鳴器
uint8_t ST_BUZZ_8u;
uint8_t ST_BUZZ_A_8u;
uint8_t TM_Buzz_ON_8u;
uint8_t TM_Buzz_ON_Buf_8u;
uint8_t TM_Buzz_OFF_8u;
uint8_t TM_Buzz_OFF_Buf_8u;
uint8_t Flag3_Door_UpLimit_8u;
uint8_t CNT_Buzz_8u;
uint16_t TM_Buzz_16u;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Door_Open(void);
void Door_Stop(void);
void Door_Close(void);		
void Door_manage(void);                                    //Operating time calculate
void PWR_CTRL(void);                                       //Power ON to motor
void Delay_ms(int32_t nms);
//static void SystemClock_Config(void);
static void EXTI4_15_IRQHandler_Config(void);
static void EXTI2_3_IRQHandler_Config(void);
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
static void Buzzer_Config(void);
static void ControlBox_Config(void);
static void Ext_CNTER(void);			//Relay觸發外部計數器
static uint16_t	TIMDEC(uint16_t TIMB);
static uint16_t	TIMINC(uint16_t TIMB);
static uint16_t ADC_Calculate(void);
uint16_t* BubbleSort(uint16_t arr[], uint16_t len);
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);	// Confirm the I2C R/W datas.

static void Buzzer_CTRL(void);
static void Buzz_ON(void);
static void Buzz_OFF(void);

static void Parameter_Load(void);	//EEPROM參數讀取
static void Parameter_List(void);	//EEPROM參數顯示
static void SMK_CTRL(void);	    //煙感偵測
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

/* Private functions ---------------------------------------------------------*/


//Variable to ADC conversion
//uint16_t i,j;
uint16_t adc_32_amnt = 0;
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
  TIM16_Config();
  TIM17_Config();
  Uart_Config();
  I2C_Config();
	
  /* Enable each GPIO Clock */
  CLOCK_Enable();
  
  //程式最後修改日期
  printf("\n\r***********************************"); 
  printf("\n\r***********************************"); 
  printf("\n\r* Final Modify Date: %d     *", Ver_date);
  printf("\n\r* Model: TYG-NCP-R01              *");
  printf("\n\r***********************************"); 
  printf("\n\r***********************************"); 
  

  // Parameter access
  //EE_Default = FALSE;
  Parameter_Load();
  Parameter_List();	
  
  /* Configure IOs in output push-pull mode to drive Relays */
  MotorRelay_out_config();
  StatusRelay_out_config();
  Buzzer_Config();	//No used
  ControlBox_Config();	//No used

  TM_OPEN = 0;
  TM_CLOSE = 0;
  CloseTM2 = TM_MAX - TM_WindowsDoor_ClosePart1;		//section_time_2 of close operation
  if(Flag_WindowsDoor == TRUE && CloseTM2 <= 0){
		Flag_WindowsDoor = FALSE;
		printf("\n\r捲窗門功能第二段關門時間 = %d", CloseTM2);
		printf("\n\r捲窗門功能: %d", Flag_WindowsDoor);
		printf("\n");
  }
  ST_Close = 1;
	
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);	
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);	
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);  

  //用途:避免開機的暫態影響GPIO判讀
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
		//Volt_StandBy = 0.3;
		printf("\n\r循環測試:有\n");
  }
	
  //開機提示音
  //Buzz_ON();
  //Delay_ms(500);
  //Buzz_OFF();
  //_Buzz_ON_8u = 5;
  ST_BUZZ_8u = 1;
	
  Rly_TME_A_8u = FALSE;
  Rly_TME_B_8u = FALSE;
  Rly_ACT_A_8u = FALSE;
  Rly_ACT_B_8u = FALSE;
  Rly_POS_A_8u = FALSE;
  Rly_POS_B_8u = FALSE;
  
  while(1){ 
		if(Flag_CycleTest == TRUE){
			//循環測試
			Cycle_Test();
			
		}else{ 
			//Main function
			Door_manage();
			Low_Operate_Function();
			IR_CTRL();
			SMK_CTRL();
			PWR_CTRL(); 
			Operate_ADC_Detect();
			Auto_Close_CTRL();
			Anti_Pressure_5();
			Debug_Monitor();
			Operate_Infor_Save();
			StatusRelay_Control();
			Buzzer_CTRL();
		}
  }
}

static void Debug_Monitor(void){
	//目前狀態偵測
	if(TM_Printf == 0){
		printf("\n\r==============狀態scan===================");			
		
		Voc_ = ADC_Calculate() *(3.3/4095);		
		printf("\n\r目前電壓值 = %f V",Voc_);
		printf("\n\r待機電壓   = %f V",Volt_StandBy);	
		
		printf("\n");
		printf("\n\n\r目前門狀態 = %d",ST_Door);
		
		if(TM_OPEN > 0 || TM_CLOSE > 0){
			if(Flag_Low_Operate == TRUE){
				printf("\n");
				printf("\n\r 緩步運轉時間 = %d", TM_Low_Operate);		
				printf("\n\r 緩步運轉狀態 = %d (1:初段/2:中段/3:尾段)", ST_Low_Operate);
			}

			if(TM_OPEN > 0){
				printf("\n\n\r開門剩餘時間 = %d ms",TM_OPEN);
				printf("\n\r PWM_Duty = %d",PWM_Duty);
				printf("\n\r PWM_Period = %d",PWM_Period);
			}
			if(TM_CLOSE > 0){
				printf("\n\n\r關門剩餘時間 = %d ms",TM_CLOSE);
				printf("\n\r PWM_Duty = %d",PWM_Duty);
				printf("\n\r PWM_Period = %d",PWM_Period);
			}
			
		}
				
		if(TM_Auto_Close > 0){
			printf("\n\n\r關門等待時間 = %d ms",TM_Auto_Close);
		}
		
		if(ST_Anti > 0){
			printf("\n\n\r防壓狀態 = %d",ST_Anti);
			printf("\n\n\r防壓界權重 = %f",Anti_Weight);
			printf("\n\n\r防壓界線值 = %d",ADC_Anti_Max);
		}
					
		if(Flag_WindowsDoor == TRUE){		//1-segment mode
			//printf("\n\rOPEN_IT= %d",Open_IT);
			printf("\n\n\r捲窗關門狀態");
			printf("ST_CLOSE= %d",ST_Close);
		}
		
		if(Anti_Event > 0){
			printf("\n");
			printf("\n\rAnti_Event = %d",Anti_Event);
			printf("\n\n\r防壓開門停止等待 = %d",TM_Anti_Occur);
		}
		
		printf("\n");
		printf("\n\r ******當前AD = %d",ADC_Calculate());
		printf("\n\r 防夾基準AD值:");
		printf("\n\r ADC_OPEN_MAX = %d",ADC_OPEN_MAX);
		printf("\n\r ADC_CLOSE_MAX = %d",ADC_CLOSE_MAX);
		
		if(ADC_Detect_Start_Flag == 1){
			printf("\n");
			printf("\n\r 運轉AD值(即時):");
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
			printf("\n\r //////////紅外線偵測觸發\\\\\\\\\\");
			printf("\n\r Flag_IR = %d",Flag_IR);
		}
		if(Flag_SMK == TRUE){
			printf("\n");
			printf("\n\r //////////煙霧感測器偵測觸發\\\\\\\\\\");
			printf("\n\r Flag_SMK = %d",Flag_SMK);
		}

		
		printf("\n");
		printf("\n\r OpEnd_Detect_Start_Flag = %d",OpEnd_Detect_Start_Flag);
		
		printf("\n\r");

		TM_Printf = 10;
	}	
}

static void Low_Operate_Function(void){
	if(Flag_Low_Operate == TRUE){
		if(TM_OPEN > 0 || TM_CLOSE > 0){
			if(TM_Low_Operate < Time_Low_Operate_Ini){ //初步啟動
				PWM_Duty = 1;
				PWM_Period = 2;
				ST_Low_Operate = 1;
			}else if(TM_Low_Operate < (Time_Low_Operate_Ini + Time_Low_Operate_Mid)){ //中段加速
				PWM_Duty = 99;
				PWM_Period = 100;
				ST_Low_Operate = 2;
			}else{	//尾段減速
				PWM_Duty = 1;
				PWM_Period = 2;
				ST_Low_Operate = 3;
			}
			
			if(Flag_No_VSB == TRUE){
				if(ST_Low_Operate == 2){
					Volt_StandBy = Volt_StandBy_b * 1.3;
				}else{
					Volt_StandBy = Volt_StandBy_b * 1.1;
				}
			}else{
				if(ST_Low_Operate == 2){
					Volt_StandBy = Volt_StandBy_b * 1.0;
				}else{
					Volt_StandBy = Volt_StandBy_b * 0.85;
				}
			}
		}else{
			if(Flag_No_VSB == TRUE){
					Volt_StandBy = Volt_StandBy_b * 1.3;
			}else{
					Volt_StandBy = Volt_StandBy_b * 1.0;
			}
		}
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
		if(HAL_I2C_Mem_Write(&I2cHandle,(uint16_t)I2C_ADDRESS, 200, I2C_MEMADD_SIZE_8BIT, (uint8_t*)TXBuf, 4, 10000) != HAL_OK){
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

//紀錄開關門時的最大與最小ADC值
static void Operate_ADC_Detect(void){
	uint16_t ADC_Tmp;
	
	//ADC_Detect_Start_Flag
	//0: 載入buffers
	//1: 透過Opend_detect()啟用,偵測ADC變化
	//2: Reload給原變數
	
	if(ADC_Detect_Start_Flag == 0){
		ADC_Tmp = ADC_Calculate();
		ADC_OPEN_MAX_b = ADC_Tmp;
		ADC_OPEN_MIN_b = ADC_Tmp;
		ADC_CLOSE_MAX_b = ADC_Tmp;
		ADC_CLOSE_MIN_b = ADC_Tmp;
	
	}else if(ADC_Detect_Start_Flag == 1){	//當限位偵測開始即執行
		if(ST_Door == 1){
			ADC_Tmp = ADC_Calculate();
			if(ADC_Tmp > ADC_OPEN_MAX_b){
				ADC_OPEN_MAX_b = ADC_Tmp;
			}else if(ADC_Tmp < ADC_OPEN_MIN_b){
				ADC_OPEN_MIN_b = ADC_Tmp;
			}
		}else if(ST_Door == 2 && TM_CLOSE_EndPart > 0){
			ADC_Tmp = ADC_Calculate();
			if(ADC_Tmp > ADC_CLOSE_MAX_b){
				ADC_CLOSE_MAX_b = ADC_Tmp;
			}else if(ADC_Tmp < ADC_CLOSE_MIN_b){
				ADC_CLOSE_MIN_b = ADC_Tmp;
			}
		}
		
	}else if(ADC_Detect_Start_Flag == 2){
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
		
		//防壓參考值下限
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
		//HAL_GPIO_WritePin(PORT_Status_Out, RL_TME, GPIO_PIN_SET);
		//Delay_ms(RLY_Delay_ms);
		//HAL_GPIO_WritePin(PORT_Status_Out, RL_TME, GPIO_PIN_RESET);
		printf("\n\rNoise test 2");

	}
	
	Door_manage();
	PWR_CTRL(); 
	//OpEnd_Detect();
	Operate_Infor_Save();

	
	if(TM_Printf == 0){
		printf("\n\r==============循環測試-狀態scan2===================");			
		Voc_ = ADC_Calculate() *(3.3/4095);		
		printf("\n\r目前電壓值 = %f V",Voc_);
		printf("\n\r待機電壓   = %f V",Volt_StandBy);
		printf("\n\rACT_Door = %d",ACT_Door);
		if(TM_OPEN > 0){
			printf("\n\n\r開門剩餘時間 = %d ms",TM_OPEN);
			printf("\n\r開門次數 = %d\n",Cycle_times_down);
		}
		if(TM_CLOSE > 0){
			printf("\n\n\r關門剩餘時間 = %d ms",TM_CLOSE);
			printf("\n\r關門次數 = %d\n",Cycle_times_down);
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
	HAL_GPIO_WritePin(PORT_Status_Out, RL_TME, GPIO_PIN_SET);
	Delay_ms(RLY_Delay_ms);
	HAL_GPIO_WritePin(PORT_Status_Out, RL_TME, GPIO_PIN_RESET);
}

void PWR_CTRL(void){
	if(TM_CLOSE > 0 && TM_OPEN > 0){
		//Avoid both of OPEN and CLOSE have be trigger simultaneously.		
		Door_Stop();
		TM_OPEN = 0;
		TM_CLOSE = 0;
		printf("\n\n\rNG:運轉時間同時>0");

	}else{	
		ST_Door_buf = ST_Door;
		if(Flag_WindowsDoor == FALSE){		//正常開關門模式
			if(TM_OPEN > 0){
				Door_Open();
			}else if(TM_CLOSE > 0){	
				Door_Close();
			}else{
				Door_Stop();
				OpEnd_Detect_Start_Flag = FALSE;
				ST_Door = 0;
			}
			
		}else{	//兩段式關門模式
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
					}else if(ST_Close == 1){	//第一段關門
						printf("TEST 01");
						if(TM_CLOSE_b == 0){
							printf("TEST 01");
							ST_Close = 2;
						}
					}else if(ST_Close == 2){	//第二段關門
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
	
	//到位偵測
	if(TM_OPEN > 0 || TM_CLOSE > 0){
		OpEnd_Detect();
		OpEnd_Detect_2();
	}else if(Flag_WindowsDoor == TRUE && TM_CLOSE == 0 && TM_CLOSE_b != 0 && Anti_Event == 0){
			printf("\n\n\rHelloxxx");
			printf("\n\n\rST_Close = %d", ST_Close);
			if(ST_Close == 1){
				Flag_Door_UpLimit   = FALSE;
				Flag_Door_DownLimit = TRUE;
				ADC_Detect_Start_Flag = 2;		//運轉結束並且儲存AD值: Operate_ADC_Detect
			}else if(ST_Close == 2){
				ADC_Detect_Start_Flag = 0;
			}			
	}else{
		ADC_Detect_Start_Flag =0;
	}
	TM_CLOSE_b = TM_CLOSE;
	

	if(TM_OPEN > 0){
		CtrlBox_Light_Up();
	}else if(TM_CLOSE > 0){
		CtrlBox_Light_Down();
	}else{
		CtrlBox_Light_OFF();
	}
			
}

void Door_manage(void){	
	if(ST_BTN == TRUE){							//控制器下達指令
		ST_BTN = FALSE;
		if(Flag_WindowsDoor == FALSE){			//兩段式關門:無
			switch(ACT_Door){					//指令判斷
				case 0:							//指令=停止
					ST_Door = 0;
					ST_Anti = 0;
					TM_OPEN = 0;
					TM_CLOSE = 0;
					break;
				
				case 1:							//指令=開門
					ST_Door = 1;
					TM_CLOSE = 0;
					TM_OPEN = TM_MAX;
					TM_AntiDly = Time_AntiDly;
					TM_EndDetec = 10;
					break;
				
				case 2:							//指令=關門
					if(Anti_Event == 2){ 		//關門防壓中: break
						break;
					}
					ST_Door = 2;
					TM_OPEN = 0;
					TM_CLOSE = TM_MAX;
					TM_CLOSE_EndPart = TM_CLOSE - 20;
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
		}else if(Flag_WindowsDoor == TRUE){	//兩段式關門:有
			switch(ACT_Door){
			//-------------------指令=停止--------------------//
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

					}else if(ST_Door == 2){			//前狀態:關門
						CloseTM_Remain = TM_CLOSE;
						if(ST_Close == 1){
								//CloseTM_Remain = TM_CLOSE;
								if(CloseTM_Remain > 0){
									Close_IT = TRUE;
								}else if(CloseTM_Remain == 0){
									Close_IT = FALSE;
									ST_Close = 2;		//等待第二段close指令
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
			//-------------------指令=停止 End----------------//
			//-------------------指令=開門--------------------//
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
			//-------------------指令=開門 End----------------//
			//-------------------指令=關門--------------------//
				case 2:
					if(Anti_Event == 2){ 		//關門防壓中: break
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
			//-------------------指令=關門 End-----------------//
			// ----- Else ----- //
				default:
					ST_Door = 0;
					TM_OPEN = 0;
					TM_CLOSE = 0;
					//break;
			}
		}
		
		//ADC偵測等待時間
		if(TM_OPEN > 0 || TM_CLOSE > 0){
			TM_ADC_Relaod = 50;
			TM_Low_Operate = 0;
		}
	}
}	

static void Auto_Close_CTRL(void){
	//自動關門功能
	if(Flag_SMK == FALSE){
		// 開門後延遲時間經過自動關門
		if(Flag_AutoClose == 1){
			if(AClose_Flg == TRUE && TM_Auto_Close == 0){		//自動關門倒數時間結束
				printf("\n\r自動關門等待時間到達");
				printf("\n\r關門時間設立");
				AClose_Flg = FALSE;									//自動關門旗標:OFF
				TM_CLOSE = TM_MAX;									//關門時間設定
			}
		}else if(Flag_AutoClose == 2){							//自動關門功能: ON
			if(Flag_WindowsDoor == FALSE){								//兩段關門功能:無
				if(	ST_Door == 0 && ST_Door_buf == 1){					//判斷門停止前的狀態是否為開門
					printf("\n\r自動關門旗標 & 等待時間設立");
					TM_Auto_Close = Time_Auto_Close;					//設定自動關門倒數時間
					AClose_Flg = TRUE;									//自動關門旗標:ON
				}else if(AClose_Flg == TRUE && TM_Auto_Close == 0){		//自動關門倒數時間結束
					printf("\n\r自動關門等待時間到達");
					printf("\n\r關門時間設立");
					AClose_Flg = FALSE;									//自動關門旗標:OFF
					TM_CLOSE = TM_MAX;									//關門時間設定
				}
			}else{
				//empty
			}
		}else{
			//Empty
		}
	}
	
	//蜂鳴器動作判斷
	if(TM_Auto_Close > 0 && TM_Auto_Close < 50){
		ST_BUZZ_8u = 8;
	}
}

static void IR_CTRL(void){
	//當紅外線偵測觸發後30秒除能該旗標
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

//******************Relay control end******************//

//無捲窗門限位偵測
static void OpEnd_Detect(void){
	if(TM_EndDetec == 0 && Flag_WindowsDoor == FALSE){
		if(OpEnd_Detect_Start_Flag == FALSE){                  //等待
			TM_DoorOperateDly = 5;                             //Delay 0.5 second後開始確認是否到限位
			OpEnd_Detect_Start_Flag = TRUE;
			ADC_Detect_Start_Flag = 1;		                   //ADC運轉值擷取開始:Operate_ADC_Detect
		}else{
			Voc = ADC_Calculate() *(3.3/4095);		
			if(Voc <= Volt_StandBy && TM_DoorOperateDly == 0){
				printf("\n\n\r門到位-停止運轉!\n\n");
				
				//判斷是否開門到位,並且設定照明時間
				if(TM_OPEN > 0){
					if(Flag_AutoClose == 1){
						TM_Auto_Close = Time_Auto_Close;					//設定自動關門倒數時間
						AClose_Flg = TRUE;									//自動關門旗標:ON
					}
					
					//若是關門防夾觸發,恢復成待機正常運轉
					Anti_Event_buf = Anti_Event;
					if(Anti_Event == 2){
						Anti_Event = 0;
					}
				}
								
				//限位旗標做成
				if(TM_OPEN > 0){
					Flag_Door_UpLimit   = TRUE;
					Flag_Door_DownLimit = FALSE;
					Flag2_Door_UpLimit_8u   = TRUE;
					Flag2_Door_DownLimit_8u = FALSE;
					Flag3_Door_UpLimit_8u = TRUE;
				}else if(TM_CLOSE > 0){
					Flag_Door_UpLimit   = FALSE;
					Flag_Door_DownLimit = TRUE;
					Flag2_Door_UpLimit_8u   = FALSE;
					Flag2_Door_DownLimit_8u = TRUE;
				}
				
				//運轉剩餘時間
				if(TM_OPEN > 0){
					Time_Remain_Open = TM_MAX - TM_OPEN;
					ST_ONEKEY_8u = 2;
				}
				if(TM_CLOSE > 0){
					Time_Remain_Close = TM_MAX - TM_CLOSE;
					ST_ONEKEY_8u = 4;
				}
				
				//運轉次數
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
				ADC_Detect_Start_Flag = 2;		//運轉結束並且儲存AD值: Operate_ADC_Detect
			}
		}
	}
}

//有捲窗門限位偵測
static void OpEnd_Detect_2(void){
	if(TM_EndDetec == 0 && Flag_WindowsDoor == TRUE){
		if(OpEnd_Detect_Start_Flag == FALSE){                  //等待
			TM_DoorOperateDly = 5;                             //Delay 0.5 second後開始確認是否到限位
			OpEnd_Detect_Start_Flag = TRUE;
			ADC_Detect_Start_Flag = 1;		                   //ADC運轉值擷取開始:Operate_ADC_Detect
		}else if(TM_OPEN > 0){
			Voc = ADC_Calculate() *(3.3/4095);		
			if(Voc <= Volt_StandBy && TM_DoorOperateDly == 0){
				printf("\n\n\r門到位-停止運轉!\n\n");
				
				//判斷是否開門到位,並且設定照明時間
				if(TM_OPEN > 0){
					if(Flag_AutoClose == 1){
						TM_Auto_Close = Time_Auto_Close;					//設定自動關門倒數時間
						AClose_Flg = TRUE;									//自動關門旗標:ON
					}
					
					//若是關門防夾觸發,恢復成待機正常運轉
					Anti_Event_buf = Anti_Event;
					if(Anti_Event == 2){
						Anti_Event = 0;
					}
				}
								
				//限位旗標做成
				Flag_Door_UpLimit   = TRUE;
				Flag_Door_DownLimit = FALSE;
				Flag2_Door_UpLimit_8u   = TRUE;
				Flag2_Door_DownLimit_8u = FALSE;
				Flag3_Door_UpLimit_8u = TRUE;
				
				//運轉次數
				REC_Operate_Times++;
				
				ST_Close = 1;

				Time_Remain_Open = TM_MAX - TM_OPEN;	//運轉剩餘時間

				TM_OPEN = 0;
				TM_CLOSE = 0;
				ST_Anti = 0;
				OpEnd_Detect_Start_Flag = FALSE;		
				ADC_Detect_Start_Flag = 2;		//運轉結束並且儲存AD值: Operate_ADC_Detect
				
				ST_ONEKEY_8u = 2;
			}
		}else if(TM_CLOSE > 0){			
			Voc = ADC_Calculate() *(3.3/4095);		
			if(Voc <= Volt_StandBy && TM_DoorOperateDly == 0){
				printf("\n\n\r門到位-停止運轉!\n\n");
												
				//限位旗標做成
				Flag_Door_UpLimit   = FALSE;
				Flag_Door_DownLimit = TRUE;
				Flag2_Door_UpLimit_8u   = FALSE;
				Flag2_Door_DownLimit_8u = TRUE;
				
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
	
	
	//當成立條件A or B啟用時: Enable

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
	//Relay TME 控制
	SR_CTRL_TME_A();
	SR_CTRL_TME_B();
	SR_TERMINATE_TME();
	
	//Relay ACT 控制
	SR_CTRL_ACT_A();
	SR_CTRL_ACT_B();
	SR_TERMINATE_ACT();

	//Relay POS 控制
	SR_CTRL_POS_A();
	SR_CTRL_POS_B();
	SR_TERMINATE_POS();

	//Relay 輸出控制
	SR_OUTPUT_CTRL();
	
	//判斷用參數RESET
	SR_STVAR_RST();
	
	//Edge變數緩衝區
	SR_VAR_BUF();
}

//觸發條件A
static void SR_CTRL_TME_A(void){
	//觸發條件判斷
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
		//輸出等待時間
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

//觸發條件B
static void SR_CTRL_TME_B(void){
	//觸發條件判斷
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
		//輸出等待時間
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
	//觸發條件判斷
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
		//輸出等待時間
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

//觸發條件B
static void SR_CTRL_ACT_B(void){
	//觸發條件判斷
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
		//輸出等待時間
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
	//觸發條件判斷
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
		//輸出等待時間
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

//觸發條件B
static void SR_CTRL_POS_B(void){
	//觸發條件判斷
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
		//輸出等待時間
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

//解除條件
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
				
			case 1:	//設定延遲時間
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
				
			case 1:	//設定延遲時間
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
				
			case 1:	//設定延遲時間
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

//變數暫存區
static void SR_VAR_BUF(void){
	TMP_TM_OPEN_16u = TM_OPEN;
	TMP_TM_CLOSE_16u = TM_CLOSE;
	TMP_Flag_LOCK_8u = Flag_LOCK;
}

static void SR_STVAR_RST(void){
	//狀態變數RESET
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

static void EXTI2_3_IRQHandler_Config(void){
  GPIO_InitTypeDef   GPIO_InitStructure;
	EXTI_CTRL_ONEKEY_CLK_ENABLE();
	
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;//GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;

  GPIO_InitStructure.Pin = W_ONEKEY;
  HAL_GPIO_Init(PORT_ONEKEY, &GPIO_InitStructure);
	
	HAL_NVIC_SetPriority(EXTI2_3_IRQn, 2, 0);
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

//外部中斷:按鍵偵測
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	printf("\n\r控制器指令: ");
	CNT_Jog_Press = 0;
	CNT_LOCK_Press = 0;
	Flag_JOG = FALSE;
	ST_Door_buf = ST_Door;
	
	switch(GPIO_Pin){
		case W_STOP:
			if(Flag_SMK  == TRUE)	break;
			if(Flag_LOCK == TRUE){			//鎖電功能ON: 不動作
				break;
			}
			ST_BTN = TRUE;
			ACT_Door = 0;
			ST_Anti = 0;
			ADC_Detect_Start_Flag = 0;
			
			BSET(Trig_RM_8u,BIT1);

			//自動關門等待時間載入
			if(AClose_Flg == TRUE){
				printf("\n\r重新載入等待關門時間: %d ms", TM_Auto_Close);
				TM_Auto_Close = Time_Auto_Close;					//設定自動關門倒數時間
			}
			
			//解除紅外線鎖定
			if(Flag_IR == TRUE){	
				Flag_IR = FALSE;
				TM_IR_Lock = 0;
			}
			
			//Relay_out: 中間停止
			//動作條件: 當運轉中按下停止
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
			if(Flag_SMK  == TRUE)	break;
			if(Flag_LOCK == TRUE){			//鎖電功能ON: 不動作

				break;
			}
			
			BSET(Trig_RM_8u,BIT0);
			
			//吋動功能
			if(Flag_Func_JOG == TRUE){
				//吋動偵測
				ST_Press = HAL_GPIO_ReadPin(PORT_OPEN, W_OPEN);
				while(ST_Press == 0){
					CNT_Jog_Press++;
					printf("\n\rOPEN key.....press\n");
					ST_Press = HAL_GPIO_ReadPin(PORT_OPEN, W_OPEN);
					if(CNT_Jog_Press > Times_JOG){		// 按鍵按下並保持: 50 times
						if(Flag_JOG == FALSE){
							//TM_DoorOperateDly = 5;
							Door_Open();
						}
						Flag_JOG = TRUE;
						printf("\n\rJOG Mode:OPEN...\n");
						Door_Open();						
					}
				}
				printf("\n\rContinue = %d\n",CNT_Jog_Press);
				if(Flag_JOG == TRUE){
					Door_Stop();
					printf("\n\rDoor Stop.....\n");
					break;
				}
			}
			
			//反向指令執行
			if(ST_Door_buf == 2){
				printf("\n\r--------立即反轉--------");
				Door_Stop();
				Delay_ms(100);
				OpEnd_Detect_Start_Flag = TRUE;				
			}
			//***反轉判定 END***//
			
			Flag_Relay_OPEN = TRUE;
			
			ST_BTN = TRUE;
			ACT_Door = 1;
			ST_ONEKEY_8u = 1;
			printf("\n\rOPEN!\n");
			break;

		case W_CLOSE:			
			if(Flag_SMK   == TRUE)	break;		//煙霧感測器觸發
			if(Anti_Event == 2)     break;      //關門防壓中
			if(Flag_LOCK  == TRUE){			//鎖電功能ON: 不動作

				break;
			}
			if(Flag_IR    == TRUE){				//紅外線觸發: 不動作
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
						Door_Close();
						//OpEnd_Detect();
						//...目前關門有到位斷路功能,
						//如果不行,再將OPEN的到位控制加入
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
			
			//***反轉判定***//
			if(ST_Door_buf == 1){
				printf("\n\r--------立即反轉--------");
				Door_Stop();
				Delay_ms(100);
				OpEnd_Detect_Start_Flag = TRUE;
			}
			//***反轉判定 END***//
			
			Flag_Relay_CLOSE = TRUE;

			ST_BTN = TRUE;
			ACT_Door = 2;
			ST_ONEKEY_8u = 3;
			printf("\n\rClose!\n");
			break;
		
		case RM_LOCK:
			printf("\n\rLock!\n");
			if(Flag_Remote_Lock == TRUE){	//鎖電功能開啟?
				
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
		
		case W_IR:	//紅外線偵測
			if(ST_Door == 2){
				Flag_IR = TRUE;
				Door_Stop();
				Delay_ms(100);
				ST_BTN = TRUE;
				ACT_Door = 1;	 // 開門
				TM_IR_Lock = 30; // 鎖定時間 30 sec.
				ST_ONEKEY_8u = 1;
				ST_BUZZ_8u = 2;
			}
			break;
		
		case W_SMK:
			Flag_SMK = TRUE;
			Door_Stop();
			Delay_ms(100);
			ST_BTN = TRUE;
			ACT_Door = 1;	 //開門
			ST_ONEKEY_8u = 1;
			break;
		
		case W_ONEKEY:
			ST_ONEKEY_8u++;
			if(ST_ONEKEY_8u > 4){
				ST_ONEKEY_8u = 1;
			}
			
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
		
		default:
				break;
	}

}

//煙霧感測偵測
static void SMK_CTRL(void){
	uint8_t SMK_tmp;
	
	//當煙霧偵測被觸發後,開始polling SMK pin,
	//直到警報解除才解除煙霧偵測旗標.
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
		TM_Buzz_ON_8u  = TIMDEC(TM_Buzz_ON_8u);
		TM_Buzz_OFF_8u  = TIMDEC(TM_Buzz_OFF_8u);
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
		TM_Buzz_ON_8u        = TIMDEC(TM_Buzz_ON_8u);
		TM_Buzz_OFF_8u       = TIMDEC(TM_Buzz_OFF_8u);
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
		//TM_Buzz_ON_8u  = TIMDEC(TM_Buzz_ON_8u);
		//TM_Buzz_OFF_8u  = TIMDEC(TM_Buzz_OFF_8u);

		
		Tim_cnt_100ms = 0;

	}
	
	// 1 sec
	if(Tim_cnt_1s == 1000){
		Tim_cnt_1s = 0;
		TM_IR_Lock 	      = TIMDEC(TM_IR_Lock);
		TM_Save 	      	= TIMDEC(TM_Save);
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
	float iweight;
	
	if(HAL_I2C_Mem_Read(&I2cHandle,(uint16_t)I2C_ADDRESS, 0, I2C_MEMADD_SIZE_8BIT, (uint8_t*)aRxBuffer, 256, 10000) != HAL_OK){
		if(HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF){
			Error_Handler();
		}
	}
	
	EE_Default = HAL_GPIO_ReadPin(PORT_EE_SEL, EEPROM_SEL);
	
	if(EE_Default == TRUE){
		//Default parameter
		Flag_CycleTest       = FALSE;   //循環測試(長時測試)
		Flag_WindowsDoor     = FALSE;   //捲窗門功能
		Flag_AntiPress       = TRUE;    //防夾功能
		Flag_AutoClose       = FALSE;   //自動關門功能
		Flag_Func_JOG        = FALSE;   //吋動功能
		Flag_Motor_Direction = TRUE;   //馬達運轉方向
		Flag_Remote_Lock     = TRUE;   //鎖電功能
		Flag_Rate_Regulate   = FALSE;   //捲門調速
		Flag_Buzzer          = FALSE;    //蜂鳴器
		Flag_Light           = FALSE;    //自動照明
		Flag_Low_Operate     = FALSE;  //緩起步 & 緩停止
		
		//EE_Addr_P = 31;


		TM_DLY_Value              = 300;   //循環測試間隔時間
		TM_WindowsDoor_ClosePart1 = 130;   //捲窗門_第一段關門時間
		TM_MAX                    = 600;   //開關門最長運轉時間
		Time_Auto_Close           = 100;   //自動關門延遲時間
		Time_Light                = 100;   //照明運轉時間

		Volt_StandBy_b = 0; //開機自動決定待機值
		//Volt_StandBy = 0.3;
		Anti_Weight_Open_select  = 5;   //防夾權重: 開門
		Anti_Weight_Close_select = 5;   //防夾權重: 關門
		
		Times_JOG        = 50;   //吋動判定次數
		Times_Remote_Lock = 75;   //鎖電成立次數
		
		PWM_Grade        = 2;   //鐵捲速度
		Auto_Close_Mode  = 1;	 //自動關門模式設定
		
		Time_Low_Operate_Ini = 20;
		Time_Low_Operate_Mid = 80;
		
		//TME-RELAY
		Flag_Rly_TME_A_8u = 0;			//成立條件A
		Flag_Rly_TME_B_8u = 0;			//成立條件B
		Flag_Rly_TME_TER_8u = 0;		//解除條件
		Time_RlyEvent_TME_A_16u = 10;	//成立時間A
		Time_RlyEvent_TME_B_16u = 30;	//成立時間B
		Time_RlyEvent_TER_TME_16u = 15;	//解除時間
		Time_RlyOp_TME_16u = 100;		//輸出時間
		
		//ACT-RELAY
		Flag_Rly_ACT_A_8u = 0;			//成立條件A
		Flag_Rly_ACT_B_8u = 0;			//成立條件B
		Flag_Rly_ACT_TER_8u = 0;		//解除條件
		Time_RlyEvent_ACT_A_16u = 20;	//成立時間A
		Time_RlyEvent_ACT_B_16u = 20;	//成立時間B
		Time_RlyEvent_TER_ACT_16u = 10;	//解除時間
		Time_RlyOp_ACT_16u = 100;		//輸出時間
		
		//POS-RELAY
		Flag_Rly_POS_A_8u = 0;			//成立條件A
		Flag_Rly_POS_B_8u = 0;			//成立條件B
		Flag_Rly_POS_TER_8u = 0;		//解除條件
		Time_RlyEvent_POS_A_16u = 30;	//成立時間A
		Time_RlyEvent_POS_B_16u = 10;	//成立時間B
		Time_RlyEvent_TER_POS_16u = 5;	//解除時間
		Time_RlyOp_POS_16u = 100;		//輸出時間
	}else{
		//******Parameter form EEPROM*****//
		EE_Addr_P = 0;
		VER1       = aRxBuffer[EE_Addr_P++];   //程式編號
		VER2       = aRxBuffer[EE_Addr_P++];   //程式版次
		Maintain_Year       = aRxBuffer[EE_Addr_P++];   //維護時間:年
		Maintain_Month      = aRxBuffer[EE_Addr_P++];   //維護時間:月
		Maintain_Day        = aRxBuffer[EE_Addr_P++];   //維護時間:日
		Warranty_Year       = aRxBuffer[EE_Addr_P++];   //保固時間:年
		Warranty_Month      = aRxBuffer[EE_Addr_P++];   //保固時間:月
		Warranty_Day        = aRxBuffer[EE_Addr_P++];   //保固時間:日
		
		//機板序號
		PN1 = aRxBuffer[EE_Addr_P++];
		PN2 = aRxBuffer[EE_Addr_P++];
		PN3 = aRxBuffer[EE_Addr_P++];
		PN4 = aRxBuffer[EE_Addr_P++];
		PN5 = aRxBuffer[EE_Addr_P++];
		PN6 = aRxBuffer[EE_Addr_P++];
		
		//Funtion ON/OFF
		EE_Addr_P = 20;
		Flag_CycleTest       = aRxBuffer[EE_Addr_P++];   //循環測試(長時測試)
		Flag_WindowsDoor     = aRxBuffer[EE_Addr_P++];   //捲窗門功能
		Flag_AntiPress       = aRxBuffer[EE_Addr_P++];   //防夾功能
		Flag_AutoClose       = aRxBuffer[EE_Addr_P++];   //自動關門功能
		Flag_Func_JOG        = aRxBuffer[EE_Addr_P++];   //吋動功能
		Flag_Motor_Direction = aRxBuffer[EE_Addr_P++];   //馬達運轉方向
		Flag_Remote_Lock     = aRxBuffer[EE_Addr_P++];   //鎖電功能
		Flag_Rate_Regulate   = aRxBuffer[EE_Addr_P++];   //捲門調速
		Flag_Buzzer          = aRxBuffer[EE_Addr_P++];   //蜂鳴器
		Flag_Light           = aRxBuffer[EE_Addr_P++];   //自動照明
		Flag_Low_Operate     = aRxBuffer[EE_Addr_P++];   //緩起步 & 緩停止


		EE_Addr_P = 40;
		TM_DLY_Value              = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;   //循環測試間隔時間
		EE_Addr_P+=2;
		
		TM_WindowsDoor_ClosePart1 = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;    //循環測試間隔時間
		EE_Addr_P+=2;
		
		TM_MAX                    = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;   //開關門最長運轉時間
		EE_Addr_P+=2;
		
		Time_Auto_Close           = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;   //自動關門延遲時間
		EE_Addr_P+=2;
		
		Time_Light                = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;   //照明運轉時間
		EE_Addr_P+=2;
		
		//EE_Addr_P = 50;
		Volt_StandBy_b     = (float)aRxBuffer[EE_Addr_P++]/10;   //待機電壓for到位判定使用
		Anti_Weight_Open_select  = aRxBuffer[EE_Addr_P++];   //防夾權重: 開門
		Anti_Weight_Close_select = aRxBuffer[EE_Addr_P++];   //防夾權重: 關門
		
		Times_JOG         = aRxBuffer[EE_Addr_P++];   //吋動判定次數
		Times_Remote_Lock = aRxBuffer[EE_Addr_P++];   //鎖電成立次數
		
		PWM_Grade      = aRxBuffer[EE_Addr_P++];   //鐵捲速度
		
		//EE_Addr_P = 56;
		Auto_Close_Mode   = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;	 //自動關門模式設定
		EE_Addr_P+=2;

		//EE_Addr_P = 58;
		Time_Low_Operate_Ini = aRxBuffer[EE_Addr_P++]; 
		Time_Low_Operate_Mid = aRxBuffer[EE_Addr_P++]; 
		
		//繼電器時間參數 TME/ACT/POS 
		EE_Addr_P = 60;
		//TME成立時間
		Time_RlyEvent_TME_A_16u = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;   //
		EE_Addr_P+=2;
		
		Time_RlyEvent_TME_B_16u = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;    //
		EE_Addr_P+=2;
		
		//ACT成立時間
		Time_RlyEvent_ACT_A_16u = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;   //
		EE_Addr_P+=2;
		
		Time_RlyEvent_ACT_B_16u = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;    //
		EE_Addr_P+=2;
		
		//POS成立時間
		Time_RlyEvent_POS_A_16u = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;   //
		EE_Addr_P+=2;
		
		Time_RlyEvent_POS_B_16u = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;    //
		EE_Addr_P+=2;
		
		//輸出時間 TME/ACT/POS
		Time_RlyOp_TME_16u = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;    //
		EE_Addr_P+=2;
		
		Time_RlyOp_ACT_16u = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;   //
		EE_Addr_P+=2;
		
		Time_RlyOp_POS_16u = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;   //
		EE_Addr_P+=2;

		//解除成立時間 TME/ACT/POS
		Time_RlyEvent_TER_TME_16u = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;    //
		EE_Addr_P+=2;
		
		Time_RlyEvent_TER_ACT_16u = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;   //
		EE_Addr_P+=2;
		
		Time_RlyEvent_TER_POS_16u = (uint16_t)aRxBuffer[EE_Addr_P] | (uint16_t)aRxBuffer[EE_Addr_P+1]<<8;   //
		EE_Addr_P+=2;
		
		//狀態RELAY成立條件
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
	
	//???:DEL AFT TEST
	/*
	Flag_Rly_TME_A_8u = 1;
	Flag_Rly_TME_B_8u = 8;
	Flag_Rly_TME_TER_8u = 3;
	Time_RlyEvent_TME_A_16u = 10;
	Time_RlyEvent_TME_B_16u = 30;
	Time_RlyEvent_TER_TME_16u = 15;
	Time_RlyOp_TME_16u = 100;
	
	Flag_Rly_ACT_A_8u = 2;
	Flag_Rly_ACT_B_8u = 7;
	Flag_Rly_ACT_TER_8u = 2;
	Time_RlyEvent_ACT_A_16u = 20;
	Time_RlyEvent_ACT_B_16u = 20;
	Time_RlyEvent_TER_ACT_16u = 10;
	Time_RlyOp_ACT_16u = 100;

	Flag_Rly_POS_A_8u = 3;
	Flag_Rly_POS_B_8u = 6;
	Flag_Rly_POS_TER_8u = 1;
	Time_RlyEvent_POS_A_16u = 30;
	Time_RlyEvent_POS_B_16u = 10;
	Time_RlyEvent_TER_POS_16u = 5;
	Time_RlyOp_POS_16u = 100;
	*/
	Flag_Buzzer = TRUE;
	//=====???=====//
	
	//捲門運行次數
	REC_Operate_Times = 0;
	//REC_Operate_Times = (uint32_t)aRxBuffer[30] | (uint32_t)aRxBuffer[31]<<8 | (uint32_t)aRxBuffer[32]<<16 | (uint32_t)aRxBuffer[33]<<24;
	for(i=0;i<4;i++){
		REC_Operate_Times = REC_Operate_Times | (uint32_t)aRxBuffer[200+i]<<(8*i);
	}
	
	//開門防夾權重設定
	switch(Anti_Weight_Open_select){
		case 1:
			iweight = 0.005;
			break;
		case 2:
			iweight = 0.007;
			break;
		case 3:
			iweight = 0.008;
			break;
		case 4:
			iweight = 0.009;
			break;
		case 5:
			iweight = 0.01;
			break;
		case 6:
			iweight = 0.011;
			break;
		case 7:
			iweight = 0.012;
			break;
		case 8:
			iweight = 0.013;
			break;
		case 9:
			iweight = 0.015;
			break;
		default:
			iweight = 0.01;
			break;
	}
	Anti_Weight_Open = iweight;
	
	//關門防夾權重設定
	switch(Anti_Weight_Close_select){
		case 1:
			iweight = 0.005;
			break;
		case 2:
			iweight = 0.007;
			break;
		case 3:
			iweight = 0.008;
			break;
		case 4:
			iweight = 0.009;
			break;
		case 5:
			iweight = 0.01;
			break;
		case 6:
			iweight = 0.011;
			break;
		case 7:
			iweight = 0.012;
			break;
		case 8:
			iweight = 0.013;
			break;
		case 9:
			iweight = 0.015;
			break;
		default:
			iweight = 0.01;
			break;
	}
	Anti_Weight_Close = iweight;
	
	//PWM速度選擇
	if(Flag_Rate_Regulate == FALSE){
		PWM_Grade = 2;
	}
	switch(PWM_Grade){
		case 0:
			PWM_Duty = 1;
			PWM_Period = 2;
			break;
			
		case 1:
			PWM_Duty = 3;
			PWM_Period = 4;
			break;
			
		case 2:
			PWM_Duty = 99;
			PWM_Period = 100;
			break;
		
		default:
			PWM_Duty = 1;
			PWM_Period = 2;
			break;
	}
	
	if(Volt_StandBy_b == 0){
		Flag_No_VSB = TRUE;
		Volt_StandBy_b = ADC_Calculate() *(3.3/4095);
		Volt_StandBy = Volt_StandBy_b * 1.3;
  }else{
		Flag_No_VSB = FALSE;
		Volt_StandBy = Volt_StandBy_b;
	}
}

static void Parameter_List(void){
	printf("\n\r==========參數設定==========");
	printf("\n\r*****功能開關(0:關閉 / 1:開啟)");
	printf("\n\r 長期測試  : %d", Flag_CycleTest);
	printf("\n\r 捲窗門功能: %d", Flag_WindowsDoor);
	printf("\n\r 防夾功能  : %d", Flag_AntiPress);
	printf("\n\r 自動關門  : %d (模式)", Flag_AutoClose);
	printf("\n\r 吋動功能  : %d", Flag_Func_JOG);
	printf("\n\r 運轉方向  : %d", Flag_Motor_Direction);
	printf("\n\r 鎖電功能  : %d", Flag_Remote_Lock);
	printf("\n\r 長期測試  : %d", Flag_Rate_Regulate);
	printf("\n\r 提示音    : %d", Flag_Buzzer);
	printf("\n\r 自動照明  : %d", Flag_Light);
	printf("\n\r 緩啟動功能: %d", Flag_Low_Operate);
	
	printf("\n\r*****運轉參數");
	printf("\n\r 開關門最大運轉時間      : %f 秒", TM_MAX *0.1);
	printf("\n\r 長期測試開關門間隔時間  : %f 秒", TM_DLY_Value *0.1);
	printf("\n\r 捲窗門關門時間(Part 1)  : %f 秒", TM_WindowsDoor_ClosePart1 *0.1);
	printf("\n\r 自動關門時間            : %f 秒", Time_Auto_Close *0.1);
	printf("\n\r 照明時間                : %f 秒", Time_Light *0.1);
	printf("\n\r 緩步運轉(第1段)         : %f 秒", Time_Low_Operate_Ini *0.1);
	printf("\n\r 緩步運轉(第2段)         : %f 秒", Time_Low_Operate_Mid *0.1);

	printf("\n\r 待機Volt                : %f(V)", Volt_StandBy);
	printf("\n\r 防夾權重(OPEN)          : %f", Anti_Weight_Open);
	printf("\n\r 防夾權重(CLOSE)         : %f", Anti_Weight_Close);
	
    printf("\n\r 吋動判定參數    : %d", Times_JOG);
    printf("\n\r 鎖電判定參數    : %d", Times_Remote_Lock);
	
    printf("\n\r 運轉速度(1~2)   : %d", PWM_Grade);
	
	printf("\n\r========參數設定 End========");
}


static void Anti_Pressure_5(void){
	uint16_t ADC_Buf;
	
	if(Flag_AntiPress == TRUE &&
		 (ADC_OPEN_MAX != 0 && ADC_CLOSE_MAX != 0)){
		
		//printf("\r\nAnti_ST = %d",ST_Anti);
		
		switch(ST_Anti){
			case 0:
				//待機
				//等待偵測延遲時間>0即開始偵測
				if(TM_AntiDly > 0){		
					ST_Anti = 1;
					Anti_Event = 0;
					Times_OverADC = 0;
				}
				
				//開門防壓時, 等待持續時間經過
				if(Anti_Event == 1 && TM_Anti_Occur == 0){
					Anti_Event =0;
				}
				
				break;
			
			case 1:		// 過載電流倍率設定
				if(TM_AntiDly > 0){
					// Empty
					
				}else if(TM_AntiDly == 0){
					//根據防夾參數權重,設定防夾動作值
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
					
					TM_AntiDly2 = 10; //Delay 0.1 second後開始偵測
				}
				break;
			
			case 2:	//開門防夾偵測
				if(TM_AntiDly2 == 0){

					ADC_Buf = ADC_Calculate();	//讀取當前AD值

					//判斷當前AD值	
					if(ADC_Buf >= ADC_Anti_Max){
						Times_OverADC++;				
					}else{
						Times_OverADC = 0;
					}
					
					//printf("\r\n\n ADC_Buf = %d, ADC_Anti_Max = %d",ADC_Buf,ADC_Anti_Max);
					
					if(Times_OverADC >= Times_OverADC_Target){
						ST_Anti = 4;
						Anti_Event = 1; 	// OPEN防夾ON
						TM_Anti_Occur = 100; //設定100秒計時
						
						//停機
						TM_OPEN = 0;
						TM_CLOSE  = 0;
						
						TM_AntiDly3 = 0; 	//運轉停止,免反轉,不需等待時間
						
						ST_BUZZ_8u = 6;
					}else{
						//ST_Anti = 2;
						Anti_Event = 0; 	// 正常運轉
						TM_AntiDly2 = 3;
					}
				}
				break;
			
			case 3:	//關門防夾偵測
				if(ST_Close == 2){
					ST_Anti = 0;
				}else if(TM_AntiDly2 == 0){

					ADC_Buf = ADC_Calculate();	//讀取當前ADC值

					
					//計算運轉電壓變化
										
					if(ADC_Buf >= ADC_Anti_Max){
						Times_OverADC++;				
					}else{
						Times_OverADC = 0;
					}
									
					if(Times_OverADC == Times_OverADC_Target){
						ST_Anti = 4;
						TM_AntiDly3 = 1;
						
						if(TM_CLOSE_EndPart > 0){
							Anti_Event = 2; 	// CLOSE防夾ON
							//停機
							TM_OPEN = 0;
							TM_CLOSE = 0;
							TM_AntiDly3 = 1;	//停頓0.1秒後,反轉開門
							ST_BUZZ_8u = 7;
						}
					}else{
						//ST_Anti = 2;
						Anti_Event = 0; 	// 正常運轉
						TM_AntiDly2 = 1;
					}
				}
				break;
			
			case 4:
			//保護動作
				if(TM_AntiDly3 == 0){
					if(Anti_Event == 1){
						//停機, 設定蜂鳴器
					}else if(Anti_Event == 2){
						//反轉開門,設定蜂鳴器
						TM_OPEN = TM_MAX;
						TM_CLOSE = 0;
					}else{
						//empty
					}
					//回復防夾待機
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
	for (i = 0; i < len - 1; ++i){          //循環N-1次
		for (j = 0; j < len - 1 - i; ++j){  //每次循環要比較的次數
			if (arr[j] > arr[j + 1])       //比大小後交換
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
  SysTick->VAL=0X00;				//清空計數器 
  SysTick->CTRL=0X01;				//使能，減到零是無動作，採用外部時鐘源 
  do 
  { 
       temp=SysTick->CTRL;	//讀取當前倒計數值 
  }
     while((temp&0x01)&&(!(temp&(1<<16))));//等待時間到達 
     
     SysTick->CTRL=0x00; 		//關閉計數器 
     SysTick->VAL =0X00; 		//清空計數器 
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
	
	switch(ST_BUZZ_8u){
		case 1:	//初送電
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
			
		case 2://紅外線觸發+開門中
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
			
		case 3://紅外線觸發+控制器關門
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
			
		case 4://煙霧感測器ON
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
			
		case 5://鎖電ON
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
			
		case 6://防夾ON:開門
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
			
		case 7://防夾ON:關門
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
		
		case 8://自動關門
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
			
		default:
			Buzz_OFF();
			break;
	}
	
	TM_Buzz_ON_Buf_8u = TM_Buzz_ON_8u;
	TM_Buzz_OFF_Buf_8u = TM_Buzz_OFF_8u;
	
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
