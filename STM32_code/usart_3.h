#include <stm32f10x.h>
#include <stdlib.h>
#include "NVIC.h"
void USART3_Init(float baudRate);
void USART3_Receive_Interrupt(void);
void USART3_send(uint8_t data);