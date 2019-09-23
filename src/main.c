//********************************************************************
//*                    EEE2046F C example                            *
//*==================================================================*
//* WRITTEN BY:    	                 		                         *
//* DATE CREATED:                                                    *
//* MODIFIED:                                                        *
//*==================================================================*
//* PROGRAMMED IN: Eclipse Neon 3 Service Release 1 (4.4.1)          *
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
//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void init_GPIOB(void);
void Delay(void);
//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void)
{
	init_GPIOB();

	for(;;)
	{
		GPIOB->ODR =0b10101010;
		Delay();

		//CHECK ADC REFERENCE TO EACH SENSOR CHANNEL AND MAKE A TURN OR GO STRAIGHT OR STOP AT JUNCTION etc based off input

	}									// Loop forever
}										// End of main

//====================================================================
// FUNCTION DEFINITIONS
//====================================================================
void init_GPIOB(void)
{
	RCC  ->AHBENR |= 1<<18;
	GPIOB->MODER  |= 0x00505555;
	GPIOB->ODR     = 0b0000010000001111;
}
void Delay(void)
{
for(int i=0; i<1000; i++)
   for(int k=0; k<2000; k++);
}

// Initialise ADC
void init_ADC(void){
//configure analogue pins for line sensors
GPIOA -> MODER |= GPIO_MODER_MODER1; //set PA1, analogue mode
GPIOA -> MODER |= GPIO_MODER_MODER2; //set PA2, analogue mode
GPIOA -> MODER |= GPIO_MODER_MODER3; //set PA3, analogue mode
GPIOA -> MODER |= GPIO_MODER_MODER5; //set PA5, analogue mode
GPIOA -> MODER |= GPIO_MODER_MODER6; //set PA6, analogue mode
RCC -> APB2ENR |= RCC_APB2ENR_ADCEN; //enable clock for ADC
ADC1 -> CFGR1 |= ADC_CFGR1_RES_1; //8 bit resolution
ADC1 -> CFGR1 &= ~ADC_CFGR1_ALIGN; //right align
ADC1 -> CFGR1 &= ~ADC_CFGR1_CONT; //single conversion mode
ADC1 -> CR |= ADC_CR_ADEN; //enable ADC
while(ADC1->ISR & ADC_ISR_ADRDY == 0); //exits loop when ADRDY == 1
}

//Triggers and ADC conversion and returns sampled ADC value:
uint8_t sampleADC(void){
ADC1 -> CR |= ADC_CR_ADSTART; // sets ADSTART to begin conversion
while(ADC1->ISR & ADC_ISR_EOC == 0); // exits loop when EOC == 1
uint8_t ADC_value = ADC1->DR; // write value to ADC_value variable (for 8 bit resolution)
return ADC_value;
}

//Initialize PWM
void init_PWM(void) {
RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
//configure PB3 and PB10 to alternate function mode
GPIOB->MODER |= GPIO_MODER_MODER10_1; // PB10 = AF LEFT INPUT 1
GPIOB->MODER |= GPIO_MODER_MODER11_1; // PB11 = AF RIGHT INPUT 1
GPIOB->AFR[1] |=  (2 << (4*(10 - 8))); //map PB10 to TIM2 channel 3 using AF2 of AFRH
GPIOB->AFR[0] |=(2 << (4*(11 - 8))); //map PB11 to TIM2 channel 2 using AF2 of AFRL
TIM2->ARR = 8000; // set ARR for approximately 1kHz PWM frequency
// set PWM mode 1 for both channels
TIM2->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1); // PWM Mode 1 for PB10
TIM2->CCMR2 |= (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1);  // PWM Mode 1 for PB11
// enable output compare on channels 2 and 3 of TIM2
TIM2->CCER |= TIM_CCER_CC3E;//PB10
TIM2->CCER |= TIM_CCER_CC4E; //PB11
TIM2->CR1 |= TIM_CR1_CEN; //start TIM2

RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
//configure PB3 and PB10 to alternate function mode
GPIOB->MODER |= GPIO_MODER_MODER12_1; // PB12 = AF LEFT INPUT 2
GPIOB->MODER |= GPIO_MODER_MODER13_1; // PB13 = AF RIGHT INPUT 2
GPIOB->AFR[1] |= (2 << (4*(12 - 8))); //map PB12 to TIM2 channel 3 using AF2 of AFRH
GPIOB->AFR[0] |= (2 << (4*(13 - 8))) ; //map PB13 to TIM2 channel 2 using AF2 of AFRL
TIM2->ARR = 8000; // set ARR for approximately 1kHz PWM frequency
// set PWM mode 1 for both channels
TIM2->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1); // PWM Mode 1 for PB12
TIM2->CCMR2 |= (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1); // PWM Mode 1 for PB13
// enable output compare on channels 2 and 3 of TIM2
TIM2->CCER |= TIM_CCER_CC3E;//PB12
TIM2->CCER |= TIM_CCER_CC4E; //PB13
TIM2->CR1 |= TIM_CR1_CEN; //start TIM2
}

//Takes in ADC channel to be converted (1,2,3,4,5,6). Corresponding channel is selected
//and sampled. Sampled sensor value is returned. uint8_t return type is sufficient since
//ADC is configured to 8 bit resolution
uint8_t read_sensor(uint8_t channel){

switch(channel){
case 1:
ADC1 -> CHSELR = ADC_CHSELR_CHSEL1; // Channel 1 (mapped to f sensor)

break;
case 2:
ADC1 -> CHSELR = ADC_CHSELR_CHSEL2; // Channel 2 (mapped to mid_r sensor)

break;
case 3:
ADC1 -> CHSELR = ADC_CHSELR_CHSEL3; // Channel 3 (mapped to mid_l sensor)

break;
case 5:
ADC1 -> CHSELR = ADC_CHSELR_CHSEL5; // Channel 5 (mapped to c_r sensor)

break;
case 6:
ADC1 -> CHSELR = ADC_CHSELR_CHSEL6; // Channel 6 (mapped to c_l sensor)
break;

}
uint8_t val = sampleADC();
return val;}

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
