#include "timer.h"
#include "NVIC.h"

void BasicTIMx_Init(TIM_TypeDef * TIMx, uint16_t freq)
{
	RCC->APB1ENR |= (3<<4);			// Enable TIM6 clock interface 
	TIMx->DIER |= (1<<0);			// Update interrupt enabled
	TIMx->SR &= ~(1<<0);			// Clear the update interrupt flag
	TIMx->PSC = 15;						// Internal clock = 8 MHz, then the timer clock = 0.5 MHz after PSC
	TIMx->ARR = 8000000/freq/(TIMx->PSC+1);	
	TIMx->DIER |= (1<<0);			// Update interrupt enabled
	//TIM6->SR &= ~(1<<0);			// Clear the update interrupt flag
	//TIM6->PSC = 15;						// Internal clock = 8 MHz, then the timer clock = 0.5 MHz after PSC
	//TIM6->ARR = 8000000/freq/(TIM6->PSC+1);
	//Set_NVIC(1,1,0,TIM6_IRQn);	// Enable TIM6 interrupt

	//DBGMCU->CR |= (3<<19);		// TIM6 and TIM7 are halted if user halted the core in debug mode
	
	if(TIMx == TIM6)
	{
		Set_NVIC(1,1,0,TIM6_IRQn);	// Enable TIM6 interrupt
	 	//RCC->APB1ENR |= 1<<4;			// Enable TIM6 clock interface 
	}
	else if(TIMx == TIM7)
	{
		Set_NVIC(1,1,0,TIM7_IRQn);	// Enable TIM7 interrupt
		//RCC->APB1ENR |= 1<<5;			// Enable TIM7 clock interface 
	}
}

void BasicTIMx_Start(TIM_TypeDef * TIMx)
{
	TIMx->CR1 |= (1<<0);			// Start the timer
}

void BasicTIMx_Stop(TIM_TypeDef * TIMx)
{
	TIMx->CR1 &= ~(1<<0);			// Stop the timer
}