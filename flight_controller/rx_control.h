/**
 *	@file rx_control.h
 *	@author SurgeExperiments
 *
 *	@brief This file contains the prototypes for receiving PWM signals from
 *		   the RX controller on GPIO pins
 */

#ifndef RX_CONTROL_H_
#define RX_CONTROL_H_

#include "structs.h"

void rx_setup_timers_for_receiver_read(void);

#endif /* RX_CONTROL_H_ */
