#include "type.h"

Motor* Motor_Init(uint8_t EC_num,bool direction){
	Motor* temp=(Motor*)malloc(sizeof(Motor));
	temp->EC_num=EC_num;
	temp->dir=direction;
	temp->pwm=0;
	temp->EC_val=0;
	temp->T=0.01;
	temp->kp=0.02;			//0.02
	temp->ki=0.1;				//0.1
	temp->kd=0.0015;		//0.0015
	temp->speed=0.0;
	temp->last_fb_dg=0.0;
	temp->last_EC_val=0;
	return temp;
}

positionReceiver* PR_Init(float x_init,float y_init,float theta_init){
	positionReceiver* temp=(positionReceiver*)malloc(sizeof(positionReceiver));
	temp->x=x_init;
	temp->y=y_init;
	temp->x_pre=0.0;
	temp->y_pre=0.0;
	temp->theta_pre=theta_init;
	return temp;
}

stateKeeper* SK_Init(float wml_ref_pre_init,float wmr_ref_pre_init){
	stateKeeper* temp=(stateKeeper*)malloc(sizeof(stateKeeper));
	temp->wmL_ref_pre=wml_ref_pre_init;
	temp->wmR_ref_pre=wmr_ref_pre_init;
	temp->wmrError_pre=0.0;
	temp->wmlError_pre=0.0;
	temp->wmlError_integral=0.0;
	temp->wmrError_integral=0.0;
	return temp;
}

HeadingPI* headingPI_init(){
	HeadingPI* temp=(HeadingPI*)malloc(sizeof(HeadingPI));
	temp->kp=0.003f;	//0.003
	temp->ki=0.002f;//0.002
	temp->i_term=0.0f;
	temp->i_max=0.3f;
	temp->wc_max=0.15f;
	temp->wc_min=0.15f;
	temp->err_pre=0.0f;
	return temp;
}

TrackingPI* trackingPI_init(){
	TrackingPI* temp=(TrackingPI*)malloc(sizeof(TrackingPI));
	temp->k_pv=0.4;				//0.4
	temp->k_iv=0.2;				//0.1
	temp->k_py=0.5;				//0.4
	temp->k_ptheta=0.4;		//0.4
	temp->k_dtheta=0.01;
	temp->i_term=0.0f;
	temp->i_max=0.3f;
	temp->wc_max=0.15f;
	temp->wc_min=0.15f;
	temp->err_pre=0.0f;
	temp->theta_desired=0.0f;
	return temp;
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
