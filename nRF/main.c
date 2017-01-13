//Imported files
#include <stdio.h>
#include <stdbool.h>
#include "stm32f10x.h"
#include "stm32f10x_conf.h"

#define sensorL_port 	GPIOA
#define sensorL_pin 	GPIO_Pin_11

#define sensorR_port	GPIOA	
#define sensorR_pin		GPIO_Pin_10

#define motorL1_port	GPIOB
#define motorL1_pin		GPIO_Pin_0

#define motorL2_port	GPIOB
#define motorL2_pin		GPIO_Pin_1

#define motorLEN_port	GPIOA
#define motorLEN_pin	GPIO_Pin_6

#define motorR1_port	GPIOB
#define motorR1_pin		GPIO_Pin_8

#define motorR2_port	GPIOB
#define motorR2_pin		GPIO_Pin_9

#define motorREN_port	GPIOA
#define motorREN_pin	GPIO_Pin_7

#define distanceSensor_port 	GPIOA
#define distanceSensor_pin		GPIO_Pin_1

#define leftencoder_port	GPIOA
#define leftencoder_pin		GPIO_Pin_2

#define rightencoder_port	GPIOA
#define rightencoder_pin	GPIO_Pin_3

//Initialization functions
static void GPIO_initialize(void);
static void TIM_initialize(void);
static void ADC_initialize(void);
static void Motor_PWM_initialize(void);
static void NVIC_Configuration(void);
static void RCC_initialize(void);
static void sensor_init(void);
static void motor_init(void);

//Sensor value acquisition functions
float analog_voltage(uint8_t);

//Robot movement functions 
static void full_stop(void);
static void forward(void);
static void backward(void);
static void stationary_left(void);
static void stationary_right(void);


//Type Define
GPIO_InitTypeDef  GPIO_InitStructure;
TIM_TimeBaseInitTypeDef timerInitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
ADC_InitTypeDef  ADC_InitStructure;
TIM_OCInitTypeDef outputChannelInit = {0,};	

//Variables
uint16_t CCR1_Val = 45;
uint16_t CCR2_Val = 80;

uint16_t PrescalerValue = 0;

uint8_t sensorL = 0;
uint8_t sensorR = 0;
float distance = 0;

uint16_t left_EncoderCount = 0;
uint16_t right_EncoderCounter = 0;
uint8_t timer_flag = 0;

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
	sensor_init();
	motor_init();
	NVIC_Configuration();	
	TIM_initialize();
	Motor_PWM_initialize();
	ADC_initialize();
	
		// Start Timer 3
	TIM_Cmd(TIM3, ENABLE);
	
	// Start Timer 3 PWM output
	TIM_CtrlPWMOutputs(TIM3,ENABLE);	
	
	// Start Timer 2
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
		
	// Start Timer 2 interrupt
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);	
	
	// Start Timer 4 interrupt
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	
	
	// Initialize misc GPIO pins
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
	
	
	// LOOOOOOOOP
	while(1) {
	
		/* Get all sensor values */
//-----------------------------------------------------------------		
		distance = 27/analog_voltage(1);
		
		// 0 = obstacle
		// 1 = unblocked
		sensorL = GPIO_ReadInputDataBit(sensorL_port, sensorL_pin);
		sensorR = GPIO_ReadInputDataBit(sensorR_port, sensorR_pin);
//-----------------------------------------------------------------
		
		
		
		/*Robot action*/
//------------------------------------------------------------------		
		// If no obstacles, move forward
		if ((distance >= 7)&&(sensorL==1)&&(sensorR==1)){
			forward();

		// If distance sensor blocked, turn right
		}
		else if((distance < 7)&&(sensorL==1)&&(sensorR==1)){
			stationary_right();
		// If left sensor blocked, turn right
		}
		else if((distance >= 7)&&(sensorL==0)&&(sensorR==1)){
			stationary_right();

		// If right sensor blocked, turn left
		}
		else if((distance >= 7)&&(sensorL==1)&&(sensorR==0)){
			stationary_left();
		// If distance + left sensor blocked, turn right
		}
		else if((distance < 7)&&(sensorL==0)&&(sensorR==1)){
			stationary_right();
		// If distance + right sensor blocked, turn left
		}
		else if((distance < 7)&&(sensorL==1)&&(sensorR==0)){
			stationary_left();
		// If only left and right sensor blocked, turn right
		}
		else if((distance >= 7)&&(sensorL==0)&&(sensorR==0)){
			stationary_right();
		// If all blocked, turn right
		}
		else if((distance < 7)&&(sensorL==0)&&(sensorR==0)){
			stationary_right();		
		}
		else{
		
		}
//------------------------------------------------------------
				
		
/*Encoder counter*/
		
		if (analog_voltage(3) > 1.5){
			GPIO_SetBits(GPIOB, GPIO_Pin_5);
		
		} else {
		
			GPIO_ResetBits(GPIOB, GPIO_Pin_5);
		}

  }		
}


void GPIO_initialize(void){

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void TIM_initialize(void){
	
	
	// Initialize Timer 2
	timerInitStructure.TIM_Prescaler = 7200;
  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timerInitStructure.TIM_Period = 5000;
  timerInitStructure.TIM_ClockDivision = 0;
  timerInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &timerInitStructure);
	
	
	//	Initialize Timer 3 -> PWM pins
	timerInitStructure.TIM_Prescaler = 7200;
  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timerInitStructure.TIM_Period = 100;
  timerInitStructure.TIM_ClockDivision = 0;
  timerInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM3, &timerInitStructure);
	
	// Initialize Timer 4 -> Obstacle sensor delay
	timerInitStructure.TIM_Prescaler = 7200;
  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timerInitStructure.TIM_Period = 5000;
  timerInitStructure.TIM_ClockDivision = 0;
  timerInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM4, &timerInitStructure);
  
}


void Motor_PWM_initialize(void){
		
    // Left Motor PWM Enable pins output channels initialize
	  outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = CCR1_Val;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
 
    TIM_OC1Init(TIM3, &outputChannelInit);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
		TIM_ARRPreloadConfig(TIM3, ENABLE);
	
	  // Right Motor PWM Enable pins output channels initialize
		outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
    outputChannelInit.TIM_Pulse = CCR2_Val;
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
 
    TIM_OC2Init(TIM3, &outputChannelInit);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
		TIM_ARRPreloadConfig(TIM3, ENABLE);
}

//Initialize RCC clocks
void RCC_initialize(void){
	
	//APB2 - GPIO A
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
	
	//APB2 - GPIO B
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	//APB2 - TIMER 1
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	//APB1 - TIMER 2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	//APB1 - TIMER 3 -> PWM
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
		//APB1 - TIMER 4 -> Sensor delay
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	  /* Enable ADC1 clock so that we can talk to it */
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


void sensor_init(void){

	// Left obstacle sensor
	GPIO_InitStructure.GPIO_Pin = sensorL_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(sensorL_port, &GPIO_InitStructure);
	
	// Right obstacle sensor
	GPIO_InitStructure.GPIO_Pin = sensorR_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(sensorR_port, &GPIO_InitStructure);
	
	// Analog distance sensor
	GPIO_InitStructure.GPIO_Pin = distanceSensor_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(distanceSensor_port, &GPIO_InitStructure);
	
	// Analog input left encoder
	GPIO_InitStructure.GPIO_Pin = leftencoder_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(leftencoder_port, &GPIO_InitStructure);
		
	// Analog input right encoder
	GPIO_InitStructure.GPIO_Pin = rightencoder_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(rightencoder_port, &GPIO_InitStructure);
	
	
}

// Initialize motor pins
void motor_init(void){
	
	/*
	Initialize left motor pins
	*/
	
	//Left forward pin
	GPIO_InitStructure.GPIO_Pin = motorL1_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(motorL1_port, &GPIO_InitStructure);
	
	//Left backward pin
	GPIO_InitStructure.GPIO_Pin = motorL2_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(motorL2_port, &GPIO_InitStructure);	
	
	//Left PWM pin
	GPIO_InitStructure.GPIO_Pin = motorLEN_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(motorLEN_port, &GPIO_InitStructure);
	
	/*
	Initialize right motor pins
	*/
	
	// Right forward pin
	GPIO_InitStructure.GPIO_Pin = motorR1_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(motorR1_port, &GPIO_InitStructure);	
	
	// Right backward pin
	GPIO_InitStructure.GPIO_Pin = motorR2_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(motorR2_port, &GPIO_InitStructure);	
	
	// RIght PWM pin
	GPIO_InitStructure.GPIO_Pin = motorREN_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(motorREN_port, &GPIO_InitStructure);
}
	

/* Interrupt Handling */

void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

    // Whatever 
		if (counter == 0){
			
			GPIO_ResetBits(GPIOA, GPIO_Pin_0);
			counter=1;
		}else if (counter==1){
			GPIO_SetBits(GPIOA, GPIO_Pin_0);
			counter=0;
		}    
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


void full_stop(void){
			
			// Left enable pulse = 0
			outputChannelInit.TIM_Pulse = 0;
			TIM_OC1Init(TIM3,&outputChannelInit);
			
			// Right enable pulse = 0
			outputChannelInit.TIM_Pulse = 0;
			TIM_OC2Init(TIM3,&outputChannelInit);
}

void forward(void){
	
		// Set forward state motor controller pins
		GPIO_SetBits(motorL1_port,motorL1_pin);
		GPIO_ResetBits(motorL2_port,motorL2_pin);
		
		GPIO_SetBits(motorR1_port,motorR1_pin);
		GPIO_ResetBits(motorR2_port,motorR2_pin);
	
					// Left enable pulse = 0
			outputChannelInit.TIM_Pulse = 50;
			TIM_OC1Init(TIM3,&outputChannelInit);
			
			// Right enable pulse = 0
			outputChannelInit.TIM_Pulse = 80;
			TIM_OC2Init(TIM3,&outputChannelInit);
}



void backward(void){
	
		// Set forward state motor controller pins
		GPIO_ResetBits(motorL1_port,motorL1_pin);
		GPIO_SetBits(motorL2_port,motorL2_pin);
		
		GPIO_ResetBits(motorR1_port,motorR1_pin);
		GPIO_SetBits(motorR2_port,motorR2_pin);
	
					// Left enable pulse = 0
			outputChannelInit.TIM_Pulse = 50;
			TIM_OC1Init(TIM3,&outputChannelInit);
			
			// Right enable pulse = 0
			outputChannelInit.TIM_Pulse = 80;
			TIM_OC2Init(TIM3,&outputChannelInit);
}

void stationary_left(void){
	// Set backward state for left motor 
		GPIO_ResetBits(motorL1_port,motorL1_pin);
		GPIO_SetBits(motorL2_port,motorL2_pin);
		
	// Set forward state for right motor
		GPIO_SetBits(motorR1_port,motorR1_pin);
		GPIO_ResetBits(motorR2_port,motorR2_pin);
	
					// Left enable pulse = 0
			outputChannelInit.TIM_Pulse = 50;
			TIM_OC1Init(TIM3,&outputChannelInit);
			
			// Right enable pulse = 0
			outputChannelInit.TIM_Pulse = 80;
			TIM_OC2Init(TIM3,&outputChannelInit);

}

void stationary_right(void){
	// Set forward state for left motor 
		GPIO_SetBits(motorL1_port,motorL1_pin);
		GPIO_ResetBits(motorL2_port,motorL2_pin);
		
	// Set backward state for right motor
		GPIO_ResetBits(motorR1_port,motorR1_pin);
		GPIO_SetBits(motorR2_port,motorR2_pin);
	
					// Left enable pulse = 0
			outputChannelInit.TIM_Pulse = 50;
			TIM_OC1Init(TIM3,&outputChannelInit);
			
			// Right enable pulse = 0
			outputChannelInit.TIM_Pulse = 80;
			TIM_OC2Init(TIM3,&outputChannelInit);

}

