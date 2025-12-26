#include <stm32f10x.h>
#include <stdio.h>
#include "MotorDrive.h"
uint32_t getMotorPWM(Motor* m){
	uint32_t p=0;
	float temp=0;
	if(m->dir==FORWARD){
		if(m->EC_num==2){
			temp=(m->v_d+4.5795)/1.4558;
			temp=(temp*800)/100;
			p=(uint32_t)temp;
		}else{
			temp=(m->v_d+4.3551)/1.4562;
			temp=(temp*800)/100;
			p=(uint32_t)temp;
		}
	}else{
		if(m->EC_num==2){
			temp=(m->v_d-143.8559)/(-1.4888);
			temp=(temp*800)/100;
			p=(uint32_t)temp;
		}else{
			temp=(m->v_d-146.4847)/(-1.5268);
			temp=(temp*800)/100;
			p=(uint32_t)temp;
		}
	}
	return p;
}
void driveMotor(Motor* m){
	if(m->EC_num==2){
		if(m->dir==FORWARD){
			GPIOB->ODR &= ~(1<<0);
		}else{
			GPIOB->ODR |= (1<<0);
		}
		TIM8->CCR2=m->pwm;		// motor A
		TIM8->CCER |= (1<<0);
	}else{
		if(m->dir==FORWARD){
			GPIOC->ODR &= ~(1<<4);
		}else{
			GPIOC->ODR |= (1<<4);
		}
		TIM8->CCR1=m->pwm;		// motor B
		TIM8->CCER |= (1<<4);
	}
	TIM8->CR1 |= (1<<0);
}