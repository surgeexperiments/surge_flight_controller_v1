/*
 * select the correct pins for button pins and init them , test that the timers are working
 * and check that pinRead works . simply set up the entire thing! 
 * for now: using PA9-PA12 for the LCD menu: the r not using any mandatory functions we need for now
 */ 

#include "../arm_drivers/timing_driver.h"
#include "buttons_pins.h"


void button_pins_setup(void){
	gpio_init_pins_io(GPIOA, 9, GPIO_MODE_INPUT, GPIO_TYPE_PUSH_PULL, GPIO_PULLUP, GPIO_SPEED_MEDIUM);
	gpio_init_pins_io(GPIOA, 10, GPIO_MODE_INPUT, GPIO_TYPE_PUSH_PULL, GPIO_PULLUP, GPIO_SPEED_MEDIUM);
	gpio_init_pins_io(GPIOA, 11, GPIO_MODE_INPUT, GPIO_TYPE_PUSH_PULL, GPIO_PULLUP, GPIO_SPEED_MEDIUM);
	gpio_init_pins_io(GPIOA, 12, GPIO_MODE_INPUT, GPIO_TYPE_PUSH_PULL, GPIO_PULLUP, GPIO_SPEED_MEDIUM);
}


static int pin_read(GPIO_TypeDef* GPIOx, int my_pin){
	if((GPIOx->IDR & (1U << my_pin)) == 1) {
		return 1;
	}
	else {
		return 0; 
	}
}


/**
 * @author SurgeExperiments
 * @brief button press detector with debouncing
*/
void button_handle_presses(buttonHandling_st *button_values){
	int x, current_input_state; 

	/**
	 * Debounce logic: if the input state has been constant for long enough,
	 * change the actual state to this state.
	 */
	for(x = 0; x < 4; x++) {
		current_input_state = pin_read(GPIOD, button_values->pinMapping[x]);

		/* State has changed, start timer */
		if(current_input_state != button_values->lastInputState[x]){
			button_values->lastButtonChangeTime[x] = tim9_ms_passed();
		}
		
		/*  
		 * If the current state has been held for long enough,
		 * change the actual state to the current state
		 */
		if((tim9_ms_passed() - (long)(button_values->lastButtonChangeTime[x])) > button_values->debounceCutoff){
			/* If input-state went from low to high, generate an event */
			if(current_input_state == 0 && button_values->inputState[x] == 1) {
				button_values->buttonPressEvent[x] = 1;
			}
			/* Change our actual inputState */
			button_values->inputState[x] = current_input_state;
		}
			
		/* Set last state to current state */
		button_values->lastInputState[x] = current_input_state;
	}
}

