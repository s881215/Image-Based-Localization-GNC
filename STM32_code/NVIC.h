#include <stdlib.h>
#include <stm32f10x.h>

void Set_NVIC(uint8_t GroupPriority, uint8_t group, uint8_t SubPriority, uint8_t IRQ_Channel);