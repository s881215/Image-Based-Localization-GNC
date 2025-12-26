#include "NVIC.h"
#include <stdlib.h>
#include <stm32f10x.h>

void Set_NVIC(uint8_t GroupPriority, uint8_t group, uint8_t SubPriority, uint8_t IRQ_Channel)
{
	uint32_t temp, temp1;
	temp1 = (~group) & 0x07;			// group=1  <==> PRIGROUP[2:0]= 110 , group=2  <==> PRIGROUP[2:0]= 101
																// group=3  <==> PRIGROUP[2:0]= 100 , group=4  <==> PRIGROUP[2:0]= 011
																// group=0  <==> PRIGROUP[2:0]= 111 , group>4  <==> Invalid
	temp1 <<= 8;
	temp = SCB->AIRCR;						// Read the previous grouping
	temp &= 0x0000F8FF;						// Clear the previous grouping
	temp |= 0x05FA0000;						// Write the key
	temp |= temp1;								// The overall AIRCR
	SCB->AIRCR = temp;						// Grouping setting completed
	
	temp = (GroupPriority << (4-group));		// Adjust the position of grouping priority controlling bits
	temp |= (SubPriority & (0x0F>>group));	// Adjust the position of subpriority controlling bits
	temp &= 0x0F;
	//NVIC_EnableIRQ(IRQ_Channel);
	NVIC->ISER[IRQ_Channel/32] |= (1<<(IRQ_Channel % 32));		// Enable the corresponding IRQ
	NVIC->IPR[IRQ_Channel] |= (temp<<4);												// Set the overall priority
}