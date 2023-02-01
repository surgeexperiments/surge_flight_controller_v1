/**
 *	@file timing_driver.h
 *	@author Surge
 *
 *	@brief This file contains prototypes for the timing driver that enables
 *		   delay and stopwatch type functionality for TIM2, TIM9 and TIM10.
 */

#ifndef TIMING_DRIVER_
#define TIMING_DRIVER_

#include "stm32f4xx.h"

void tim2_setup_for_interrupt_timing(void);
void tim2_setup_for_delay_count_ms(void);
void tim2_setup_for_delay_count_us(void);
void tim2_reset_count(void);
uint32_t tim2_read_count(void);
void tim2_delay_ms(uint32_t millis_to_delay);
void tim2_delay_us(uint32_t micros_to_delay);
uint32_t tim2_ms_passed(void);
uint32_t tim2_us_passed(void);

void tim9_setup_for_interrupt_timing(void);
void tim9_setup_for_delay_count_ms(void);
void tim9_setup_for_delay_count_us(void);
void tim9_setup_delay_count_us_int_warn(void);
void tim9_reset_count(void);
uint32_t tim9_read_count(void);
void tim9_delay_ms(uint32_t millis_to_delay);
void tim9_delay_us(uint32_t micros_to_delay);
uint32_t tim9_ms_passed(void);
uint32_t tim9_us_passed(void);

void tim10_setup_for_delay_count_ms(void);
void tim10_setup_for_delay_count_us(void);
void tim10_setup_delay_count_us_int_warn(void);
void tim10_reset_count(void);
uint32_t tim10_read_count(void);
void tim10_delay_ms(uint32_t millis_to_delay);
void tim10_delay_us(uint32_t micros_to_delay);

// Experimental and untested (found online)
// uint64_t GetCycleCount64(void);
// uint64_t getTimeViaCycles(void);

/* INFO:
 *
 * : Upcounting mode

Upcounting mode In upcounting mode, the counter counts from 0 to the auto-reload value (content of the TIMx_ARR register),
then restarts from 0 and generates a counter overflow event.

An Update event can be generated at each counter overflow or by setting the UG bit in the TIMx_EGR register
(by software or by using the slave mode controller).

 Arm interrupts are not reentrant. This means that once the handler is executing, even if a milli has
passed and another sysTick interrupt occurs, the handler will exit and then re-enter the interrupt immediately
to service the blocked interrupt source.
In this case, msTicks will never increment, so the delay function will never exit and the whole thing locks up.
-> This is why you cannot use a delay() in an interrupt timing routing
 *
 */

#endif /* TIMING_DRIVER_ */
