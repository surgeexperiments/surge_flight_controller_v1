/**
 *	@file led.c
 *	@author SurgeExperiments
 *
 *	@brief This file contains the functions for using the leds on the stm32f411 discovery board.
 */

#include "led.h"
#include "../arm_drivers/gpio_driver.h"

/**
 * @author SurgeExperiments
 */
void led_init(void)
{
	gpio_init_clock(GPIOD);

	/* We use pin 12 for timer 4 for now  */
	gpio_init_pins_io(GPIOD, 13, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW);
	gpio_init_pins_io(GPIOD, 14, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW);
	gpio_init_pins_io(GPIOD, 15, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW);
	
	LED_OFF(1);
	LED_OFF(2);
	LED_OFF(3);
	LED_OFF(4);
}

