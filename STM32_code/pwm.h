#include <stdlib.h>
#include <stdbool.h>
#include <stm32f10x.h>
#include "type.h"

void pwm_Init(uint16_t arr, uint16_t psc);
void pwm(Motor* m);