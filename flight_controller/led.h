/**
 *	@file led.h
 *	@author SurgeExperiments
 *
 *	@brief This file contains the defines and prototypes for using the leds on the stm32f411 discovery board.
 */

#ifndef LED_H_
#define LED_H_

#include "stm32f4xx.h"

// Only use arg: 1 - 4
#define LED_ON(ledNumber) (GPIOD->BSRR |= (1U<<(ledNumber+11)))
#define LED_OFF(ledNumber) (GPIOD->BSRR |= (1U<<((ledNumber+1)+16)))

#define LED1_TOGGLE (GPIOD->ODR ^= (1<<12));
#define LED2_TOGGLE (GPIOD->ODR ^= (1<<13));
#define LED3_TOGGLE (GPIOD->ODR ^= (1<<14));
#define LED4_TOGGLE (GPIOD->ODR ^= (1<<15));

#define LED_TOGGLE_ALL (GPIOD->ODR ^= ((1<<12) | (1<<13) | (1<<14) | (1<<15))); 

void led_init(void);

#endif /* LED_H_ */
