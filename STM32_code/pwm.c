#include "pwm.h"

void pwm_Init(uint16_t arr, uint16_t psc)
{
	// GPIO setting
	RCC->APB2ENR |= 3<<3;				// Enable PORTC clock
	GPIOB->CRL &= 0xFFFFFFF0;		// PortB 0 is genaral purpose output push-pull
	GPIOB->CRL |= 0x00000002;
	GPIOC->CRL &= 0xFFF0FFFF;		// PortC 4 is genaral purpose output push-pull
	GPIOC->CRL |= 0x00020000;
	GPIOC->CRL &= 0x00FFFFFF;		// PORTC 6,7 are alternate function output push-pull
	GPIOC->CRL |= 0xBB000000;		// output max speed = 50 MHz
	GPIOC->CRH &= 0xFFFFFFF0;		// PORTC 8 is alternate function output push-pull
	GPIOC->CRH |= 0x0000000B;		// output max speed = 50 MHz
	// Timer setting
	RCC->APB2ENR |= 1<<13;			// Enable TIMER8
	TIM8->CCMR1 &= 0xFCFC;			// 11111100  Set TIM8_CH1 and TIM8_CH2 as output
	TIM8->CCMR1 |= 6<<4;				// Set TIM8_CH1 as PWM mode  
	TIM8->CCMR1 |= 6<<12;				// Set TIM8_CH2 as PWM mode
	TIM8->CCMR2 |= 6<<4;				// Set TIM8_CH3 as PWM mode
															// In upcounting, CH1 is active when TIMx_CNT<TIMx_CCR1
															// In downcounting, CH1 is inactive(OC1REF = '0') when TIMx_CNT>TIMx_CCR
	TIM8->ARR = arr;						// Asign auto-reload register, namely the period of PWM
	TIM8->PSC = psc;						// Asign prescaler. timer clock = APB2 clock freq / (psc+1)
	TIM8->CCMR1 |= 1<<3;				// Enable TIM8_CH1 preload mode
	TIM8->CCMR1 |= 1<<11;				// Enable TIM8_CH2 preload mode
	TIM8->CCMR2 |= 1<<3;					// Enable TIM8_CH3 preload mode
	TIM8->CCER &= ~(1<<0);					// Enable TIM8_CH1
	TIM8->CCER &= ~(1<<4);					// Enable TIM8_CH2
	//TIM8->CCER |= 1<<8;					// Enable TIM8_CH3
	TIM8->CCER &= ~(1<<0);					// Disable TIM8_CH1 (PC6/motorA/motor_right)
	TIM8->CCER &= ~(1<<4);					// Disable TIM8_CH2 (PC7/motorB/motor_left)
	TIM8->BDTR |= 1<<15;				// Enable timer output if the corresponding enabling bits are set
	TIM8->CR1 |= 1<<7;					// Enable auto-reload preload
	TIM8->CR1 |= 1<<0;					// Enable counter mode
}

void pwm(Motor* m)
{
	uint16_t pwm;
	if(m->dir == FORWARD)
	{
		pwm = TIM8->ARR * m->pwm / 100;
		if(m->EC_num == 3)
			GPIOC->ODR &= ~(1<<4);	// change the level of C4 to low
		else
			GPIOB->ODR &= ~(1<<0); // B0	
	}
	else
	{
		pwm = TIM8->ARR * (100- m->pwm) / 100;
		//pwm=TIM8->ARR * m->pwm /100;
		if(m->EC_num == 3)
			GPIOC->ODR |= (1<<4);	// change the level of C4 to high
			//GPIOC->ODR &= ~(1<<4);
		else
			GPIOB->ODR |= (1<<0);
			//GPIOB->ODR &= ~(1<<0);
	}
	if(m->EC_num == 3)
	{
		TIM8->CCR1 = pwm;
		TIM8->CCER |= 1<<0;					// Enable TIM8_CH1 (PC6/motorA/motor_right)
	}
	else
	{
		TIM8->CCR2 = pwm;
		TIM8->CCER |= 1<<4;					// Enable TIM8_CH2 (PC7/motorB/motor_left)
	}	
	
}