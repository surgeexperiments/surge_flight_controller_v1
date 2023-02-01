

#include "error.h"
#include "../arm_drivers/dbg_swo_driver.h"
#include "../arm_drivers/timing_driver.h"
#include "led.h"

/**
 * @author SurgeExperiments
 * Warning!! Calling this from the main loop while flying will make the quad crash! 
 * Only use this when testing either w/o esc's connected or no propellers on the quad
 */
void infinite_error_loop(void) {
	print_str_to_dbg_port("infinite error loop entered!\n");
	while(1){
		LED_TOGGLE_ALL;
		tim10_delay_ms(100);
		LED_TOGGLE_ALL;
		tim10_delay_ms(50);
	}
}
