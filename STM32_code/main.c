#include <stdlib.h>
#include <stdio.h>
#include "_HAL_GPIO.h"
#include <stm32f10x.h>
#include <string.h>
#include <stdarg.h>
#include <ctype.h>
#include "encoder.h"
//#include "type.h"
#include "usart_3.h"
#include "clock.h"
#include "controller.h"

#define RXBUFSZ 128
volatile uint8_t rxbuf[RXBUFSZ];
volatile uint16_t rx_head=0,rx_tail=0;

Motor* motorA;			// left
Motor* motorB;			// right
positionReceiver* ground;
positionReceiver* car;
stateKeeper* state;
HeadingPI* head;
TrackingPI* track;
volatile int rx_count=0;
uint8_t last_ch;
float gx,gy,cx,cy;
int headingOK=0;
int turnToHeading=0;
typedef enum{ IDLE_FIRST=0, IDLE=1, HEADING=2, TRACK=3} working_mode;
static working_mode workMode=IDLE_FIRST;
float dir=1;
bool TRACK_OVER=false;
int sample_cnt=0;
int sample_cnt2=0;
bool updateFlag=false;
bool getNewCarPos=false;
float desiredTheta;
bool headingFlag_specialCase=false;

int usart3_pending=0;
uint32_t check=0;
uint32_t check_bit;
uint16_t tim7_check=0;
uint8_t tim7_stop=0;
uint8_t sample=0;
uint32_t num=0;
uint32_t stop_flag=0;
uint32_t start_read_ec=0;
uint32_t fs = 1000; // 1000 Hz
uint32_t pwm_val=0;
volatile uint8_t cmd_received=0;
uint8_t read_ec_flag=0;
uint8_t user_cmd[20];						// Some user commands
float pi=3.14159265358979323846f;
void TIM6_IRQHandler(void);
void TIM7_TRQHandler(void);
void USART3_IRQHandler(void);
//void TIM6_IRQHandler(void);
void timer_init(){
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	TIM4->PSC=0;
	TIM4->ARR=72;				// 1-MHz
	TIM4->CR1 |= TIM_CR1_URS;		//only when counter overflow generates the update interrupt
	TIM4->DIER |= TIM_DIER_UIE;	//update interrupt enable
	TIM4->EGR |= TIM_EGR_UG;		//update generation
	NVIC_EnableIRQ(TIM4_IRQn);
	//Set_NVIC(1,1,0,TIM4_IRQn);	// Enable TIM4 interrupt
	//TIM4->CR1 |= TIM_CR1_CEN;
}
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
		Set_NVIC(0,1,0,TIM6_IRQn);	// Enable TIM6 interrupt
	 	//RCC->APB1ENR |= 1<<4;			// Enable TIM6 clock interface 
	}
	else if(TIMx == TIM7)
	{
		Set_NVIC(1,1,0,TIM7_IRQn);	// Enable TIM7 interrupt
		//RCC->APB1ENR |= 1<<5;			// Enable TIM7 clock interface 
	}
}
void pwm_Init(uint16_t arr, uint16_t psc)
{
	// GPIO setting
	RCC->APB2ENR |= 3<<3;				// Enable PORTC clock
	GPIOB->CRL &= 0xFFFFFFF0;		// PortB 0 is genaral purpose output push-pull  PB0 & PC4 for SAIN1 & SBIN1
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
	TIM8->PSC = psc;						// Asign prescaler. timer clock = APB2 clock freq / (psc+1) CK_CNT(counter clock freq.)=72MHz
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
	//TIM8->CR1 |= 1<<0;					// Enable counter mode
}

void stopAction(){
	GPIOC->ODR &= ~(1<<4);
	GPIOB->ODR &= ~(1<<0);
	motorA->pwm=0;
	motorB->pwm=0;
	TIM8->CCER &= ~(1<<0);
	TIM8->CCER &= ~(1<<4);
	TIM8->CR1 &= ~(1<<0);
	TIM6->CR1 &= ~(1<<0);
	TIM7->CR1 &= ~(1<<0);
	TIM2->CR1 &= ~(1<<0);
	TIM3->CR1 &= ~(1<<0);
	TIM2->CNT = 32768;
	TIM3->CNT = 32768;
}

// circular-buffer

static inline void rb_push(uint8_t b){
	uint16_t next=(rx_head+1)%RXBUFSZ;
	if(next!=rx_tail){
		rxbuf[rx_head]=b;
		rx_head=next;
	}
}
static int rb_pop(uint8_t* out){
	if(rx_tail==rx_head) return 0;
	*out=rxbuf[rx_tail];
	rx_tail=(rx_tail+1)%RXBUFSZ;
	return 1;
}
static int read_line(char* line,int maxlen){
	static int idx=0;
	uint8_t b;
	while(rb_pop(&b)){
		if(b=='\r') continue;
		if(b=='\n'){
			line[idx<maxlen-1 ? idx : maxlen-1]='\0';
			idx=0;
			return 1;
		}
		if(idx<maxlen-1) line[idx++]=(char)b;
		else idx=0;
	}
	return 0;
}

int main(){
	uint32_t pll = 1;
	Clock_Init(pll);							// HSE = SYSCLK = HCLK = APB1CLK = APB2CLK = 8 MHz
	Encoder_Init();
	uint16_t fs=100;							// period : 0.01 sec
	BasicTIMx_Init(TIM6,fs);
	BasicTIMx_Init(TIM7,fs);
	motorA=Motor_Init(2,FORWARD);
	motorB=Motor_Init(3,FORWARD);
	pwm_Init(800,0); 							// timer 8 =10kHz
	ground=PR_Init(0.0,0.0,0.0);
	car=PR_Init(0.0,0.0,3.1415926/2);
	state=SK_Init(0.0,0.0);
	head=headingPI_init();
	track=trackingPI_init();
	USART3_Init(115200);
	USART3_Receive_Interrupt();
	char line[64];
	TIM6->CR1 |= (1<<0);
	TIM2->CR1 |= (1<<0);
	TIM3->CR1 |= (1<<0);
	while(1){
		
		// 1019 programming
		
		
		/*
		//1016 test heading
		if(cmd_received==1){
			TIM6->CR1 |= (1<<0);
			TIM2->CR1 |= (1<<0);
			TIM3->CR1 |= (1<<0);
			if(sample==1){
				motorA->EC_val=Read_Encoder(motorA->EC_num);
				motorB->EC_val=Read_Encoder(motorB->EC_num);
				heading_controller(ground,car,motorA,motorB,head,state);
				driveMotor(motorA);
				driveMotor(motorB);
				sample=0;
				//cmd_received=0;
			}
			if(headingOK==1){
				break;
			}
		}
		*/
		
		// 1018 programing
		while(read_line(line,sizeof(line))){
			if(sscanf(line,"G%f,%fC%f,%f",&gx,&gy,&cx,&cy)==4){
				ground->x=gx;
				ground->y=gy;
				car->x=cx;
				car->y=cy;
				getNewCarPos=true;
			}
			if( (fabsf(ground->x-ground->x_pre)>0.00001) && (fabsf(ground->y-ground->y_pre)>0.00001) ){
				turnToHeading=1;
				ground->x_pre=gx;
				ground->y_pre=gy;
			}else if( (ground->x==ground->x_pre) && (ground->y>ground->y_pre) ){
					headingFlag_specialCase=true;
					desiredTheta = pi/2; 
					turnToHeading=1;
					ground->x_pre=gx;
					ground->y_pre=gy;
			}else if( (ground->x==ground->x_pre) && (ground->y<ground->y_pre) ){
					headingFlag_specialCase=true;
					desiredTheta = -pi/2; 
					turnToHeading=1;
					ground->x_pre=gx;
					ground->y_pre=gy;
			}else if( (ground->y==ground->y_pre) && (ground->x>ground->x_pre) ){
					headingFlag_specialCase=true;
					desiredTheta = 0; 
					turnToHeading=1;
					ground->x_pre=gx;
					ground->y_pre=gy;
			}else if( (ground->y==ground->y_pre) && (ground->x<ground->x_pre) ){
					headingFlag_specialCase=true;
					desiredTheta = pi; 
					turnToHeading=1;
					ground->x_pre=gx;
					ground->y_pre=gy;
			}
			cmd_received=1;
		}
		if(cmd_received==1){
			cmd_received=0;
			if((turnToHeading==1)){
				float ex,ey,Pey,thetac;
				workMode=HEADING;
				turnToHeading=0;
				thetac=car->theta_pre;
				ex=ground->x-car->x;
				ey=ground->y-car->y;
				Pey=-sinf(thetac)*ex + cosf(thetac)*ey;
				if(Pey>0){
					dir=-1.0;
				}else{
					dir=1.0;
				}
			}
		}
		if(sample==1){
			sample=0;
			motorA->EC_val=Read_Encoder(motorA->EC_num);
			motorB->EC_val=Read_Encoder(motorB->EC_num);
			
			switch(workMode){
				case IDLE_FIRST:
					break;
				case IDLE:
					break;
				case HEADING:
					heading_controller(ground,car,motorA,motorB,head,state);
					if(headingOK){
						headingOK=0;
						headingFlag_specialCase=false;
						workMode=TRACK;
					}
					break;
				case TRACK:
					//input_output_controller_v2_estimate(ground,car,state,motorA,motorB);
					//USART3_send(0x61);
					//USART3_send(0x0A);
					input_output_PID_controller(ground,car,track,state,motorA,motorB);
					//USART3_send(0x62);
					//USART3_send(0x0A);
					if(TRACK_OVER){
						TRACK_OVER=false;
						updateFlag=true;
						USART3_send(0x65);
						USART3_send(0x0A);
						workMode=IDLE;
					}
					break;
			}
			driveMotor(motorA);
			driveMotor(motorB);
			
		}
		
		/*
		while(1){
			TIM6->CR1 |= (1<<0);
			TIM2->CR1 |= (1<<0);
			TIM3->CR1 |= (1<<0);
			if(sample==1){
				if(cmd_received==1){
					cmd_received=0;
				}
				motorA->EC_val=Read_Encoder(motorA->EC_num);
				motorB->EC_val=Read_Encoder(motorB->EC_num);
				input_output_PID_controller(ground,car,state,motorA,motorB);
				driveMotor(motorA);
				driveMotor(motorB);
				sample=0;
			}
		}
		*/
		/*
		if(cmd_received==1){
			TIM6->CR1 |= (1<<0);
			TIM2->CR1 |= (1<<0);
			TIM3->CR1 |= (1<<0);
			float gx,gy,cx,cy;
			if(sscanf(rx_line,"G%f,%fC%f,%f",&gx,&gy,&cx,&cy)==4){
				ground->x=gx;
				ground->y=gy;
				car->x=cx;
				car->y=cy;
			}
			//input_output_controller_v2(ground,car,state,motorA,motorB);
			//driveMotor(motorA);
			//driveMotor(motorB);
			//cmd_received=0;
		}
		if(sample==1){
			motorA->EC_val=Read_Encoder(motorA->EC_num);
			motorB->EC_val=Read_Encoder(motorB->EC_num);
			//input_output_controller_v2(ground,car,state,motorA,motorB);
			//input_output_controller_v4(ground,car,state,motorA,motorB,head);
			//controller_test(3.0,motorA,motorB);
			PID_controller(0.15,0.15,motorA,motorB,state);
			//input_output_PID_controller(ground,car,state,motorA,motorB);
			driveMotor(motorA);
			driveMotor(motorB);
			sample=0;
		}
		*/
		/*
		if(cmd_received==1){
			TIM6->CR1 |= (1<<0);
			//TIM7->CR1 |= (1<<0);
			TIM2->CR1 |= (1<<0);
			TIM3->CR1 |= (1<<0);
			if(sample==1){
				// decode string
				float gx,gy,cx,cy;
				if(sscanf(rx_line,"G%f,%fC%f,%f",&gx,&gy,&cx,&cy)==4){
					ground->x=gx;
					ground->y=gy;
					car->x=cx;
					car->y=cy;
				}
				//
				//cmd_received=0;
				motorA->EC_val=Read_Encoder(motorA->EC_num);
				motorB->EC_val=Read_Encoder(motorB->EC_num);
				//input_output_controller_v4(ground,car,state,motorA,motorB,head);
				//controller_test(3.0,motorA,motorB);
				controller_test(3.0,motorA,motorB);
				driveMotor(motorA);
				driveMotor(motorB);
				ground->x_pre=gx;
				ground->y_pre=gy;
				sample=0;
				cmd_received=0;
			}
		}
		*/
	}
}
void TIM6_IRQHandler(void){
	TIM6->CR1 &= ~(1<<0);			// Stop the timer
	TIM6->DIER &= ~(1<<0);			// Update interrupt disabled
	TIM6->SR &= ~(1<<0);			// Clear the interrupt flag
	sample=1;
	TIM6->CNT = 0;
	TIM6->DIER |= (1<<0);			// Update interrupt enabled
	TIM6->CR1 |= (1<<0);			// Start the timer
}
/*
void USART3_IRQHandler(void){
	static uint8_t index = 0;
	USART3->CR1 &= ~(1<<5);				// Disable receive interrupt
	user_cmd[index] = USART3->DR; // USART3->DR : data reg.
	index += 1;
	if(index==6){									// "user-command" + "enter"
		index = 0;
		motorA->cmd = user_cmd[0];
		motorB->cmd = user_cmd[2];
		motorA->cmd2 = user_cmd[1];
		motorB->cmd2 = user_cmd[3];
		motorA->cmd3 = user_cmd[4];
		motorB->cmd3 = user_cmd[4];
		motorA->cmd_spin = user_cmd[5];
		motorB->cmd_spin = user_cmd[5];
		cmd_received=1;
		//TIM6->CR1 |= (1<<0);
	}
	//motorA->cmd=USART3->DR;
	//motorB->cmd=USART3->DR;
	USART3->CR1 |= (1<<5);				// Enable receive interrupt
}
*/
/*
// can work but we need to decide the ref-point by hand
void USART3_IRQHandler(void){
	volatile uint32_t sr=USART3->SR;
	uint8_t ch=USART3->DR;
	last_ch=ch;
	static uint16_t idx=0;
	if(cmd_received) return;
	if(ch=='\n' || ch=='\r'){
		rx_line[idx<32?idx:32-1]='\0';
		idx=0;
		cmd_received=1;
		if(sscanf(rx_line,"G%f,%fC%f,%f",&gx,&gy,&cx,&cy)==4){
			ground->x=gx;
			ground->y=gy;
			car->x=cx;
			car->y=cy;
		}
		if( (ground->x!=ground->x_pre) && (ground->y!=ground->y_pre) ){
			turnToHeading=1;
		}
		return;
	}
	if(idx<(32-1)){
		rx_line[idx++]=ch;
	}else{
		idx=0;
	}
}
*/

void USART3_IRQHandler(void){
	uint32_t sr=USART3->SR;
	uint8_t ch=USART3->DR;
	(void)sr;
	rb_push(ch);
}

/*
void USART3_IRQHandler(void){
	volatile uint32_t sr=USART3->SR;
	uint8_t ch=USART3->DR;
	static uint16_t idx=0;
	if(cmd_received){
		if(ch=='\n'){
			idx=0;
		}
		return;
	}
	if(ch=='\n'){
		rx_line[(idx<32)?idx:31]='\0';
		idx=0;
		cmd_received=1;
		if(sscanf(rx_line,"G%f,%fC%f,%f",&gx,&gy,&cx,&cy)==4){
			ground->x=gx;
			ground->y=gy;
			car->x=cx;
			car->y=cy;
		}
		if( (ground->x!=ground->x_pre) && (ground->y!=ground->y_pre) ){
			turnToHeading=1;
		}
	}else if(ch!='\r'){
		if(idx<31) rx_line[idx++]=ch;
		else idx=0;
	}
}
*/
void TIM7_IRQHandler(void){
	TIM7->CR1 &= ~(1<<0);
	TIM7->DIER &= ~(1<<0);
	TIM7->SR &= ~(1<<0);
	tim7_check++;
	if(tim7_check==500){
		tim7_check=0;
		tim7_stop=1;
		TIM7->CNT=0;
		TIM7->DIER |= (1<<0);
		TIM7->CR1 &= ~(1<<0);
	}else{
		TIM7->CNT=0;
		TIM7->DIER |= (1<<0);
		TIM7->CR1 |= (1<<0);
	}
}
