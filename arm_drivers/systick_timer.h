
#ifndef SYSTICK_TIMER_
#define SYSTICK_TIMER_

#include "stm32f4xx.h"

// You can only use pointer definitions to access absolute memory addr in gnu gcc
// Didn't find this in the include files, so I'll put it here! 
#define SYSTICK_VAL_REG         (*((volatile unsigned long *) 0xE000E018))

void init_systick_timer(void);

uint32_t micros_passed(void);
void SysTick_Handler(void); 
void delay_us(uint32_t microsToDelay);

#endif /* SYSTICK_TIMER_ */
