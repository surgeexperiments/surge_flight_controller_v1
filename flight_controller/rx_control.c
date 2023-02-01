/**
 *	@file rx_control.c
 *	@author SurgeExperiments
 *
 *	@brief This file contains the functions for receiving PWM signals from
 *		   the RX controller on GPIO pins
 */

#include <stdint.h>

#include "rx_control.h"
#include "../arm_drivers/pwm_read.h"

/**
 * @author SurgeExperiments
 * @brief function that sets up the PWM read capability on 4 GPIO pins
 * 		  so all 4 channels from the RX control can be read
 *
 * Check the functions in PWM_read.c to see which GPIO pins these functions
 * will activate. You can also change which pins are coupled to a timer in
 * that file.
 */
void rx_setup_timers_for_receiver_read(void)
{
    init_pwm_read_timer_1();
    init_pwm_read_timer_3();
    init_pwm_read_timer_4();
    init_pwm_read_timer_5();
}
