//********************************************************************
//*                    EEE3099S Maze Solver                          *
//*==================================================================*
//* WRITTEN BY:    	                 		                         *
//* DATE CREATED:                                                    *
//* MODIFIED:                                                        *
//*==================================================================*
//* PROGRAMMED IN: Atollic TrueSTUDIO 9.3                            *
//* DEV. BOARD:    UCT STM32 Development Board                       *
//* TARGET:	   STMicroelectronics STM32F051C6                        *
//*==================================================================*
//* DESCRIPTION:                                                     *
//*                                                                  *
//********************************************************************
// INCLUDE FILES
//====================================================================
#include <stdint.h>
#include "stm32f0xx.h"
#include "lcd_stm32f0.h"
#include <stdio.h>
#include <math.h>
//====================================================================
// SYMBOLIC CONSTANTS
//====================================================================
#define ADC_REF 80//if above this value then we know that sensor reading will be high and on black line (inverse is true)
//====================================================================
// GLOBAL VARIABLES
//====================================================================
uint8_t f = 0; //front sensor reading
uint8_t mid_l = 0; //middle left sensor reading
uint8_t mid_r = 0; //middle right sensor reading
uint8_t c_l = 0; //corner left sensor reading
uint8_t c_r = 0; //corner right sensor reading
uint8_t ADC_reference=0; //if above this value then we know that sensor reading will be high and on black line (inverse is true)
int sensor1;
int sensor2;
int sensor3;
//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void init_GPIO(void);
void Delay(void);
int sampleADC(int channel);
void init_ADC(void);
void init_PWM(void);
int read_sensor(int channel);
void init_EXTI(void);
void EXTI2_3_IRQHandler (void);
void get_moving(void);
//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void)
{
	init_GPIO();
	init_ADC();
	init_PWM();
	init_LCD();
	init_EXTI();

	GPIOB -> ODR = 0b0;
	for(;;)
	{

		//CHECK ADC REFERENCE TO EACH SENSOR CHANNEL AND MAKE A TURN OR GO STRAIGHT OR STOP AT JUNCTION etc based off input

	}									// Loop forever
}										// End of main

//====================================================================
// FUNCTION DEFINITIONS
//====================================================================
void init_EXTI (void)
{
	RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; // ENABLE EXTI BUS CLK
	SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PA; // MAP INTERRUPT TO PA3
	EXTI -> IMR |= EXTI_IMR_MR3; // UNBLOCK INTERRUPT LINE 1 BUS
	EXTI -> FTSR |= EXTI_FTSR_TR3; // CONDITION CHECK: FALLING-EDGE
	NVIC_EnableIRQ(EXTI2_3_IRQn); // ENABLE LINE 0 &amp; LINE 1 INTERRUPT
}

void EXTI2_3_IRQHandler (void)
{
	EXTI -> PR |= EXTI_PR_PR3; // EXIT INTERRUPT

	// User Interrupt Service Routine Here
	get_moving();
}

void init_GPIO(void)
{

	RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;	//enable port a
	RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;	//enable port b

	GPIOA -> MODER &= ~GPIO_MODER_MODER1; //set PA1, input mode
	GPIOA -> MODER &= ~GPIO_MODER_MODER2; //set PA2, input mode
	GPIOA -> MODER &= ~GPIO_MODER_MODER3; //set PA3, input mode

	//set pull-up resistors on ports A0-3
	GPIOA->PUPDR |= (GPIO_PUPDR_PUPDR0_0|
						GPIO_PUPDR_PUPDR1_0|
						GPIO_PUPDR_PUPDR2_0|
						GPIO_PUPDR_PUPDR3_0);

	GPIOA -> MODER |= GPIO_MODER_MODER4; //set PA4, analogue mode
	GPIOA -> MODER |= GPIO_MODER_MODER5; //set PA5, analogue mode
	GPIOA -> MODER |= GPIO_MODER_MODER6; //set PA6, analogue mode

	RCC  ->AHBENR |= RCC_AHBENR_GPIOBEN;
	GPIOB->MODER  |= (GPIO_MODER_MODER0_0 |
						GPIO_MODER_MODER1_0|
						GPIO_MODER_MODER2_0|
						GPIO_MODER_MODER3_0|
						GPIO_MODER_MODER12_0|
						GPIO_MODER_MODER13_0|
						GPIO_MODER_MODER14_0|
						GPIO_MODER_MODER15_0);	//setting LEDs to output mode
	GPIOB->ODR = 0b0;
}

void Delay(void)
{
for(int i=0; i<1000; i++)
   for(int k=0; k<2000; k++);
}

void get_moving(void){
	for(;;){
		sensor1 = read_sensor(4);
		sensor2 = read_sensor(5);
		sensor3 = read_sensor(6);

		GPIOB -> ODR = 0b1001000000000000;

		//TODO Try 5 as the threshold
		if(sensor1<=2){
			//move forward
			TIM2->CCR3 = 50*80;
			TIM2->CCR4 = 50*80;
			//GPIOB -> ODR = 0b1001000000000000;
		}
		else if(sensor2<=2){
			//correct left
			TIM2->CCR3 = 30*80;
			TIM2->CCR4 = 50*80;
			//GPIOB -> ODR = 0b1010000000000000;
		}else if(sensor3<=2){
			//correct right
			TIM2->CCR3 = 50*80;
			TIM2->CCR4 = 30*80;
			//GPIOB -> ODR = 0b0110000000000000;
		}else if(sensor1>2 && sensor2>2 && sensor3>2){
			TIM2->CCR3 = 50*80;
			TIM2->CCR4 = 50*80;
			GPIOB -> ODR = 0b1010000000000000;
		}
	}//end for

}//end get moving function

void turn_left(void){
	//set motors to turn left
}

// Initialise ADC
void init_ADC(void){
	//configure analogue pins for line sensors
	RCC -> APB2ENR |= RCC_APB2ENR_ADCEN; //enable clock for ADC
	ADC1 -> CR &= ~(ADC_CR_ADSTART);		//stop adc if running

	ADC1 -> CFGR1 |= ADC_CFGR1_RES_1; //8 bit resolution
	ADC1 -> CFGR1 &= ~ADC_CFGR1_ALIGN; //right align
	ADC1 -> CFGR1 &= ~ADC_CFGR1_CONT; //single conversion mode

	ADC1 -> CR |= ADC_CR_ADEN; //enable ADC
	//while((ADC1->ISR & ADC_ISR_ADRDY) == 0); //exits loop when ADRDY == 1
}

//Triggers and ADC conversion and returns sampled ADC value:
int sampleADC(int channel){
	ADC1 -> CHSELR &= 0b0;	//disable channel
	ADC1 -> CHSELR |= channel;	//select channel 5
	ADC1 -> CR |= ADC_CR_ADEN;				//set aden to 1 (on)
	while((ADC1 -> ISR & ADC_ISR_ADRDY) ==0);	//wait for RDY

	ADC1 -> CR |= ADC_CR_ADSTART; //start ADC
	while((ADC1 -> ISR & ADC_ISR_EOC) ==0); //wait for end of conversion
	int ADCval=ADC1 -> DR; //get adc value
	return ADCval;
	ADC1 -> CHSELR &= 0b0;	//disable channel
}

//Initialize PWM
void init_PWM(void) {
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;
	//configure PB3 and PB10 to alternate function mode
	GPIOB->MODER |= GPIO_MODER_MODER10_1; // PB10 = AF LEFT INPUT 1
	GPIOB->MODER |= GPIO_MODER_MODER11_1; // PB11 = AF RIGHT INPUT 1
	GPIOB->AFR[1] |=  (2 << (4*(10 - 8))); //map PB10 to TIM2 channel 3 using AF2 of AFRH
	GPIOB->AFR[1] |=(2 << (4*(11 - 8))); //map PB11 to TIM2 channel 2 using AF2 of AFRL
	TIM2->ARR = 4000; // set ARR for approximately 1kHz PWM frequency
	// set PWM mode 1 for both channels
	TIM2->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1); // PWM Mode 1 for PB10
	TIM2->CCMR2 |= (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1);  // PWM Mode 1 for PB11
	TIM2->CCR3 = 20 * 80; // Duty Cycle for PB10 = 20%
	TIM2->CCR4 = 90 * 80; // Duty Cycle for PB11 = 90%
	// enable output compare on channels 2 and 3 of TIM2
	TIM2->CCER |= TIM_CCER_CC3E;//PB10
	TIM2->CCER |= TIM_CCER_CC4E; //PB11
	TIM2->CR1 |= TIM_CR1_CEN; //start TIM2
	}

	//Takes in ADC channel to be converted (1,2,3,4,5,6). Corresponding channel is selected
	//and sampled. Sampled sensor value is returned. uint8_t return type is sufficient since
	//ADC is configured to 8 bit resolution
int read_sensor(int channel){

	int channel_bitmask;
	switch(channel){
		case 1:
			channel_bitmask = ADC_CHSELR_CHSEL1; // Channel 1 (mapped to f sensor)

			break;
		case 2:
			channel_bitmask = ADC_CHSELR_CHSEL2; // Channel 2 (mapped to mid_r sensor)

			break;
		case 3:
			channel_bitmask = ADC_CHSELR_CHSEL3; // Channel 3 (mapped to mid_l sensor)

			break;
		case 5:
			channel_bitmask = ADC_CHSELR_CHSEL5; // Channel 5 (mapped to c_r sensor)

			break;
		case 6:
			channel_bitmask = ADC_CHSELR_CHSEL6; // Channel 6 (mapped to c_l sensor)
			break;

}
	int val = sampleADC(channel_bitmask);
	return val;
}

//Sets duty cycle (and consequently speed) of left motor.
void set_PWM_left(uint8_t ADCval){
	float temp = (float)ADCval;
	TIM2->CCR3 = (((temp/249)*100)) * 80; //adjust count/compare register value based on 8-bit range
}

//Sets duty cycle (and consequently speed) of right motor.
void set_PWM_right(uint8_t ADCval){
	float temp = (float)ADCval;
	TIM2->CCR4 = (((temp/249)*100)) * 80; //adjust count/compare register value based on 8-bit range
}

//Disable left motor. Set duty cycle to 0%
void PWM_disable_left(void){
	set_PWM_left(0);
}

//Disable right motor. Set duty cycle to 0%
void PWM_disable_right(void){
	set_PWM_right(0);
}

void TurnLeft(){//CONFIGURE PWM FOR EACH TURN TYPE
}

void TurnRight(){

}

void Forward(){
}

void Stop(){
}

void UTurn(){
}

void GetBackOnTrack(){
}


//********************************************************************
// END OF PROGRAM
//********************************************************************
