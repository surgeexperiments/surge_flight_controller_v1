/**
 *	@file pwm_gen.h
 *	@author SurgeExperiments
 *
 *	@brief  This file contains prototypes for functions that allows generation
 *			of PWM on GPIO pins via timers
 */

#ifndef PWM_GEN_
#define PWM_GEN_

#include "stm32f4xx.h"

void enable_pwm_timer_2(uint8_t prescalerValue, uint16_t pwmCycleLength);
void enable_pwm_timer_3(uint8_t prescaler_value, uint16_t pwm_cycle_length);

void set_pwm_freq(TIM_TypeDef *TIMx,
                  uint16_t channel1,
                  uint16_t channel2,
                  uint16_t channel3,
                  uint16_t channel4);

#endif /* PWM_GEN_ */
