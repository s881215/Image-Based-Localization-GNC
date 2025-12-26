#include "usart_3.h"

void USART3_Init(float baudRate){
	float temp, fra_temp;
	uint16_t mantissa, fraction;
	
	RCC->APB2ENR |= (1<<4);				// Enable the PORTC clock interrupt
	RCC->APB2ENR |= (1<<0);				// Enable the Alternate Function I/O clock interface
	RCC->APB1ENR |= (1<<18);			// Enable the USART3 clock interface
	GPIOC->CRH &= ~(255<<8);			// Clear the original setting about PORTC
	GPIOC->CRH |= (8<<12);				// Input with pull-up/pull-down (Rx)
	GPIOC->CRH |= (11<<8);				// Output mode, max speed 50 MHz, Alternate function output Push-pull
	GPIOC->ODR |= (1<<10);				// According to the manual, TX should be "1" when being idle
	AFIO->MAPR |= (1<<4);					// GPIO remap: map Tx, Rx from PB10,11 to PC10,11
	

	RCC->APB1RSTR |= (1<<18);			// Reset USART3
	RCC->APB1RSTR &= ~(1<<18);		// Stop reseting USART3
	// AHPB1 Clock = 8 MHz in this case
	temp = 8000000/baudRate/16;				// 8000000 should be adjusted according to different PCLC1
	mantissa = temp;							// Get the integer part
	fra_temp = temp - mantissa;		// Get the decimal point part
	fraction = 16 * fra_temp;			// Convert to the real fractoin value;
	USART3->BRR = (mantissa<<4) | fraction;		// Transform to the form of USART3->BRR
	USART3->CR1 &= ~(3<<2);				// Disable Rx and Tx
	USART3->CR1 |= (1<<13);				// Enable USART3
}

void USART3_Receive_Interrupt(){
	Set_NVIC(1,1,0,USART3_IRQn);
	//NVIC_EnableIRQ(USART3_IRQn);
	USART3->CR1 |= (1<<2);				// Receiver enable 
	USART3->CR1 |= (1<<5);				// Enable receive interrupt
}

void USART3_send(uint8_t data){
	//USART3->DR=data;
	//USART3->CR1 |= (1<<3);
	//while(USART3->SR != (USART3->SR | (1<<7))){;}
	USART3->CR1 |= (1<<3);
	while(!(USART3->SR & (1<<7))){}
	USART3->DR=data;
	while(!(USART3->SR & (1<<6))){}
}