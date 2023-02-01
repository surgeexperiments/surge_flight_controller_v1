/**
 *	@file esc.h
 *	@author Surge
 *
 *	@brief This file contains the defines and prototypes for handling the ESC's.
 */

#ifndef ESC_H_
#define ESC_H_

#include "stm32f4xx.h"
#include "enums.h"
#include "structs.h"

#define ESC_RUNNING_MAX_PULSE 2000
#define ESC_RUNNING_MIN_PULSE 1200
#define ESC_KEEPALIVE_PULSE 1000
#define ESC_VOLTAGE_COMPENSATION_MINIMUM 1250

void esc_init_hardware_pwm_gen(void);
void esc_calculate_pulse_length(const flyMode_et mode,
                                uint8_t use_voltage_compensation,
                                int throttle,
                                const int battery_voltage,
                                const pidData_st pid_output_data,
                                escOutput_st *esc_output);

void esc_generate_hw_pwm_motors(TIM_TypeDef *TIMx, const escOutput_st esc_output);
void esc_set_motors_to_standby_speed(escOutput_st *outputData);

#endif /* ESC_H_ */
