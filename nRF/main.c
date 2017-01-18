//Imported files
#include <stdio.h>
#include <stdbool.h>
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "nRF24.h"



/*
STM32F103 SPI2 output pin definition
Port B
*/
#define SPI_GPIO_Port	GPIOB
#define SPI_CSN				GPIO_Pin_12
#define SPI_SCK 			GPIO_Pin_13
#define SPI_MISO 			GPIO_Pin_14
#define SPI_MOSI 			GPIO_Pin_15
#define SPI_CE 				GPIO_Pin_10


//Initialization functions
static void GPIO_initialize(void);
void TIM_initialize(void);
static void ADC_initialize(void);
void NVIC_Configuration(void);
static void RCC_initialize(void);
static void SPI_initialize(void);

//Sensor value acquisition functions
float analog_voltage(uint8_t);



//Type Define
GPIO_InitTypeDef  GPIO_InitStructure;
SPI_InitTypeDef		SPI_InitStructure;
TIM_TimeBaseInitTypeDef timerInitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
ADC_InitTypeDef  ADC_InitStructure;
TIM_OCInitTypeDef outputChannelInit = {0,};	

//Variables
uint16_t CCR1_Val = 45;
uint16_t CCR2_Val = 80;

uint16_t PrescalerValue = 0;


uint8_t timer_flag1 = 0;
int timer_flag2 = 0;

uint8_t counter = 0;


/*APB1 -> 36MHz
	APB2 -> 72MHz
	AHB  -> 72MHz*/

/* -----------------------------------------------------------------------
    TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles:
    The TIM3CLK frequency is set to SystemCoreClock (Hz), to get TIM3 counter
    clock at 24 MHz the Prescaler is computed as following:
     - Prescaler = (TIM3CLK / TIM3 counter clock) - 1

    SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
    and Connectivity line devices and to 24 MHz for Low-Density Value line and
    Medium-Density Value line devices
		
    The TIM3 is running at 36 KHz: TIM3 Frequency = TIM3 counter clock/(ARR + 1)
                                                  = 24 MHz / 666 = 36 KHz
																									
    TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
    TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
    TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
    TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%
  ----------------------------------------------------------------------- */


int main(void) {
	
	RCC_initialize();
	GPIO_initialize();
	NVIC_Configuration();	
	TIM_initialize();	
	ADC_initialize();
	GPIO_SetBits(SPI_GPIO_Port, SPI_CE);
	//SPI_initialize();
	
		// Start Timer 3
	//TIM_Cmd(TIM3, ENABLE);
	
	// Start Timer 3 PWM output
	//TIM_CtrlPWMOutputs(TIM3,ENABLE);	
	
	
	TIM_Cmd(TIM4, ENABLE);	
	
	// Start Timer 4 interrupt
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		// Start Timer 2 interrupt
	//TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	//TIM_Cmd(TIM2, ENABLE);
	
	nRF_RX_TX_MODE();
	// LOOOOOOOOP
	while(1) {
		
		/*
		if (timer_flag2 == 1){
			GPIO_SetBits(GPIOA, GPIO_Pin_9);
		
		}
		
		if (timer_flag2 == 0){
			GPIO_ResetBits(GPIOA, GPIO_Pin_9);
		
		}
		
		*/
	
  }		
}

void SPI_initialize(void){

			SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
      SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
      SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
      SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
      SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
      SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
      SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
      SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
      SPI_InitStructure.SPI_CRCPolynomial = 7;
      SPI_Init(SPI2, &SPI_InitStructure);

      /* Enable SPIz */
      SPI_Cmd(SPI2, ENABLE);

}



void GPIO_initialize(void){
// Configure SPI_CSN pin
  GPIO_InitStructure.GPIO_Pin = SPI_CSN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(SPI_GPIO_Port, &GPIO_InitStructure);
	
// Configure SPI_SCK pin
  GPIO_InitStructure.GPIO_Pin = SPI_SCK;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(SPI_GPIO_Port, &GPIO_InitStructure);

// Configure SPI_MOSI pin
	GPIO_InitStructure.GPIO_Pin = SPI_MOSI;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(SPI_GPIO_Port, &GPIO_InitStructure);
	
	
// Configure SPI_MISO pin
  GPIO_InitStructure.GPIO_Pin = SPI_MISO;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(SPI_GPIO_Port, &GPIO_InitStructure);
	

// Configure SPI_CE pin
	GPIO_InitStructure.GPIO_Pin = SPI_CE;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(SPI_GPIO_Port, &GPIO_InitStructure);
	
}

void TIM_initialize(void){
	
	
	// Initialize Timer 2
	timerInitStructure.TIM_Prescaler = 14400;
  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Down;
  timerInitStructure.TIM_Period = 10000;
  timerInitStructure.TIM_ClockDivision = 0;
  timerInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &timerInitStructure);
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

	// Configure so that the interrupt flag is only set upon overflow
	TIM_UpdateRequestConfig(TIM2, TIM_UpdateSource_Regular);

	// Enable the TIM5 Update Interrupt type
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	
	/*
	//	Initialize Timer 3 -> PWM pins
	timerInitStructure.TIM_Prescaler = 7200;
  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timerInitStructure.TIM_Period = 100;
  timerInitStructure.TIM_ClockDivision = 0;
  timerInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM3, &timerInitStructure);
	*/
	
	// Initialize Timer 4 -> Obstacle sensor delay
	timerInitStructure.TIM_Prescaler = 7200;
  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timerInitStructure.TIM_Period = 5000;
  timerInitStructure.TIM_ClockDivision = 0;
  timerInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM4, &timerInitStructure);
  
}



//Initialize RCC clocks
void RCC_initialize(void){
	
	// RCC SPI2 CLK should be < 10 MHz
	//RCC_PCLK2Config(RCC_HCLK_Div16);
	
	//GPIO A
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
	
	//GPIO B (SPI pins)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		
	//SPI2 (SPI pins)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	
	
	
	
	

	
	//TIMER 2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

		//TIMER 4
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	 // ADC1
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
}

void NVIC_Configuration(void){
	
	
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;   
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;      
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
	
	
	// Sensor delay interrupt -> Timer 4
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;   
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;      
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 	
}


void ADC_initialize(void){
	
	//ADCCLK = PCLK2/8 = 9MHz
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);
	
	/* Put everything back to power-on defaults */
	ADC_DeInit(ADC1);
	
	/* ADC1 and ADC2 operate independently */
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	
  /* Disable the scan conversion so we do one at a time */
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	
  /* Don't do contimuous conversions - do them on demand */
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	
  /* Start conversin by software, not an external trigger */
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	
  /* Conversions are 12 bit - put them in the lower 12 bits of the result */
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	
  /* Say how many channels would be used by the sequencer */
  ADC_InitStructure.ADC_NbrOfChannel = 3;

  /* Now do the setup */
  ADC_Init(ADC1, &ADC_InitStructure);
	
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibaration register */
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));
  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
}



/* Interrupt Handling */

void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		
		timer_flag2 = 1;
		
		/*
		if (timer_flag2 == 1){
			timer_flag2 = 0;
		} else if(timer_flag2==0){
			timer_flag2 = 1;
		}
*/
    /*
		if (counter == 0){
			
			GPIO_ResetBits(GPIOA, GPIO_Pin_9);
			counter=1;
		}else if (counter==1){
			GPIO_SetBits(GPIOA, GPIO_Pin_9);
			counter=0;
		} */   
  }
		
} 



void TIM4_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

    //Whatever

  }
}



float analog_voltage (uint8_t channel){

	ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_7Cycles5);
  // Start the conversion
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  // Wait until conversion completion
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

	// Convert 12 bit ADC value to voltage between 0-3.3v
	// 4096/3.3 = 1241
	return (ADC_GetConversionValue(ADC1)/1241);
	
	
}


