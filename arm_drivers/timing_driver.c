/**
 *	@file timing_driver.c
 *	@author SurgeExperiments
 *
 *	@brief This file contains functions for stopwatch and delay type functionality
 *		   on timers TIM2, TIM9 and TIM10.
 *
 *  IMPORTANT: The "delayCount" type functions are only good for 65535 micros/65.5
 * 			   millis b4 overflow due to a 16 bit counter and 1000 counts per milli.
 *
 *  		   You can check if they overflowed by using the init functions
 * 			   ending with "withInterruptWarning" and checking if the global vars
 * 			   tim9_delaycount_err_overflow and tim10_delaycount_err_overflow are set to 1.
 *			   This functionality is not yet implemented in all functions (like the delay functions).
 *			   Check the functions you wanna use to see if they have this functionality implemented.
 *
 *			   Do NOT use the timer based on TIM9 when you use TIM1 break interrupts,
 *			   they are the same and time will be messed up!
 *			   Use the right millis_passed ect functions for the timer you initialized!
 *
 *			   Don't use the interrupt-style timers at this moment, they are not tested properly!
 *			   Only use the delay type timers!
 *
 *  INFO:	   The nice thing about a peripheral timer is that it an independent hardware
 * 			   timer and it is always keeping count, regardless of whether interrupts are disabled or not.
 *
 * 			   TIM2 on apb2 (50mhz)
 * 			   - For millis we prescale by 49999
 * 			   - For micros we prescale by 49
 *
 * 			   TIM9 is on apb1 (100mhz)
 * 			   - For millis we should have prescaled by 99999, but the 16 bit prescaler reg only
 * 				 takes 65535, so we prescale by 49999 and count 2 per millisecond
 * 				 (which makes the max delay 32ish millis before overflow)
 * 			   - For micros we prescale by 99
 *
 * 	TESTING:   Set pulseview to 500khz, 1M samples to get it right
 * 			   (if not the timing will be 2x what it really is):
 *			   pulseview is off sometimes w/salae thing
 */

#include "timing_driver.h"
#include "dbg_swo_driver.h"

static volatile uint32_t tim2_ms_since_startup = 0;
static volatile uint32_t tim9_ms_since_startup = 0;
static volatile uint8_t tim2_timer_interrupt_ran = 0;
static volatile uint8_t tim9_timer_interrupt_ran = 0;

/* if time 65.5ms "max" timers overflow, these vars will be set to 1 */
static volatile uint8_t tim9_delaycount_err_overflow = 0;
static volatile uint8_t tim10_delaycount_err_overflow = 0;

// static uint64_t last_cycle_count_64 = 0;

/*******************************************
 *			INTERRUPT HANDLERS			   *
 *										   *
 *******************************************/

/**
 * @author SurgeExperiments
 * @brief interruptHandler for TIM2 millis-count.
 */
void TIM2_IRQHandler(void)
{
    tim2_timer_interrupt_ran = 1;
    ++tim2_ms_since_startup;

    /* clear the interrupt flag */
    if (TIM2->SR & TIM_SR_UIF)
    {
        TIM2->SR &= ~TIM_SR_UIF;
    }
}

/**
 * @author SurgeExperiments
 * @brief interrupt that us used to debug delayCount.
 * 		  If this runs your code has gone past the 65.5ms max
 * 		  limit for a timer (ie you've fucked up).
 */
void TIM1_BRK_TIM9_IRQHandler(void)
{
    tim9_timer_interrupt_ran = 1;
    ++tim9_ms_since_startup;

    tim9_delaycount_err_overflow = 1;

    /* clear the interrupt flag */
    if (TIM9->SR & TIM_SR_UIF)
    {
        TIM9->SR &= ~TIM_SR_UIF;
    }
}

/**
 * @author SurgeExperiments
 * @brief interrupt that us used to debug delayCount.
 * 		  If this runs your code has gone past the 65.5ms
 * 		  max limit for a timer (ie you've fucked up)
 */
void TIM1_UP_TIM10_IRQHandler(void)
{
    tim10_delaycount_err_overflow = 1;
    if (TIM10->SR & TIM_SR_UIF)
    {
        TIM10->SR &= ~TIM_SR_UIF;
    }
}

/****************************************************
 *					TIM2 functions 					*
 *													*
 ****************************************************/

/**
 * @author SurgeExperiments
 * @brief function that handles setting up TIM2 for interrupt timing.
 * 		  NOTE: This function is not tested properly.
 *
 * 		  This function does the math to get the frequency right
 * 		  regardless of sysclock.
 */
void tim2_setup_for_interrupt_timing(void)
{
    /* enable the clock for TIM2 (bit 0) */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* equation:
     * update_event=sysClock/((psc+1)*(arr+1)*(RCR+1)
     * prescaler=99, arr=999, gives: 10^8 / ((99+1)*(999+1)) == 1000
     */

    TIM2->PSC = 99;
    TIM2->ARR = 999;

    /* enable uie interrupt (TIM_DIER_UIE) */
    TIM2->DIER = (1U << 0);

    /* Reload the prescaler immediately */
    TIM2->EGR = 1;
    TIM2->CR1 = TIM_CR1_CEN;

    /* Set the prescaler and arr to generate an overflow event every 1ms */
    NVIC_EnableIRQ(TIM2_IRQn);
}

/**
 * @author SurgeExperiments
 * @brief function that handles setting up TIM2 for delay timing.
 */
void tim2_setup_for_delay_count_ms(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 49999;

    /* Reload the prescaler immediately */
    TIM2->EGR = 1;
    TIM2->CR1 = TIM_CR1_CEN;
}

/**
 * @author SurgeExperiments
 * @brief function that handles setting up TIM2 for delay timing.
 */
void tim2_setup_for_delay_count_us(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 49;
    TIM2->EGR = 1;
    TIM2->CR1 = TIM_CR1_CEN;
}

/**
 * @author SurgeExperiments
 * @brief Use this when you want the timer to start counting from 0 again.
 */
void tim2_reset_count(void)
{
    TIM2->CNT = 0;
}

/**
 * @author SurgeExperiments
 * @brief Use this to read how much time has passed since the last timer-reset
 */
uint32_t tim2_read_count(void)
{
    return TIM2->CNT;
}

/**
 * @author SurgeExperiments
 * @brief Use this when you want a delay between 0 and 65.5 seconds.
 *
 * 		  NOTE: This function disables TIM2 afterwards,
 * 				don't use it while using TIM2 for something else
 */
void tim2_delay_ms(uint32_t ms_to_delay)
{
    tim2_setup_for_delay_count_ms();
    while ((TIM2->CNT) < ms_to_delay)
        ;

    /* Disable the timer afterwards */
    TIM2->CR1 &= ~TIM_CR1_CEN;
}

/**
 * @author SurgeExperiments
 * @brief Use this when you want a delay between 0 and 65.5 ms.
 *
 * 		  If the timer went beyond 65.5ms
 * 		  tim2_delaycount_err_overflow will be set to 1.
 *
 * 		  This function also disables TIM2 afterwards,
 * 		  don't use it while using TIM2 for something else
 */
void tim2_delay_us(uint32_t us_to_delay)
{
    tim2_setup_for_delay_count_us();
    while ((TIM2->CNT) < us_to_delay)
        ;

    /* Disable the timer afterwards */
    TIM2->CR1 &= ~TIM_CR1_CEN;
}

/**************************
 * INTERRUPT-BASED TIMERS *
 *************************/

/**
 * @author SurgeExperiments
 * @brief NOTE: This function is not tested properly.
 */
uint32_t tim2_ms_passed(void)
{
    return tim2_ms_since_startup;
}

/**
 * @author SurgeExperiments
 * @brief NOTE: This function is not tested properly.
 */
uint32_t tim2_us_passed(void)
{
    /*
     * Bit 0 UIF: Update interrupt flag ? This bit is set by hardware on an update event.
     * It is cleared by software. 0: No update occurred. 1: Update interrupt pending.
     * This bit is set by hardware when the registers are updated: ? At overflow or
     * underflow (for TIM2 to TIM5) and if UDIS=0 in the TIMx_CR1 register.
     */
    tim2_timer_interrupt_ran = 0;
    uint32_t us_passed = tim2_ms_since_startup * 1000;
    uint32_t current_reg_count = (uint32_t)TIM2->CNT;

    /* Millis updated in between, must reupdate */
    if (tim2_timer_interrupt_ran == 1)
    {
        us_passed = tim2_ms_since_startup * 1000;
        /* Since it did an overflow, we need this to be the new (and lower) value */
        current_reg_count = (uint32_t)TIM2->CNT;
    }

    /* 1000 timer ticks per millis means one per micro */
    return us_passed + current_reg_count;
}

/*************************************
 *			TIM9 functions 			 *
 *									 *
 *************************************/

/**
 * @author SurgeExperiments
 * @brief function that handles setting up TIM9 for interrupt timing.
 * 		  NOTE: This function is not tested properly.
 *
 * This function does the math to get the frequency right regardless of sysclock
 */
void tim9_setup_for_interrupt_timing(void)
{
    /* enable the clock for TIM2 (bit 0) */
    RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;

    /* on another bus than TIM2, needs a different prescaler */
    TIM9->PSC = 199;
    TIM9->ARR = 999;

    /* enable uie interrupt */
    TIM9->DIER = (1U << 0);

    /* bit 2 urs: 1: Only counter overflow generates an update interrupt if enabled. */
    TIM9->CR1 |= (1U << 2);
    TIM9->CR1 |= TIM_CR1_CEN;

    /* Set the prescaler and arr to generate an overflow event every 1ms (or 0.5ms if needed) */
    NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
}

/**
 * @author SurgeExperiments
 * @brief function that handles setting up TIM2 for delay timing. Works well.
 *
 * IMPORTANT: Due to prescaler-limits this timer will tick twice per ms
 * 	      	  So multiply your desired millis when comparing with the reg with 2.
 * 			  No more than 32.2 ms
 */
void tim9_setup_for_delay_count_ms(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;

    /* This should have been 99999 for 1 count per ms, but the reg can only go to 65535 */
    TIM9->PSC = 49999;

    TIM9->EGR = 1;
    TIM9->CNT = 0; // santity check
    TIM9->CR1 = TIM_CR1_CEN;
}

/**
 * @author SurgeExperiments
 * @brief function that handles setting up TIM9 for delay timing.
 */
void tim9_setup_for_delay_count_us(void)
{
    /* enable the clock for TIM2 (bit 0) */
    RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;

    /* one count per micro, will overflow at 65.535 millis */
    TIM9->PSC = 99;

    TIM9->EGR = 1;
    TIM9->CNT = 0;
    TIM9->CR1 = TIM_CR1_CEN;
}

/**
 * @author SurgeExperiments
 * @brief function that handles setting up TIM9 for delay timing with overflows detectable
 *        by checking if tim9_delaycount_err_overflow==1
 */
void tim9_setup_delay_count_us_int_warn(void)
{
    /* enable the clock for TIM2 (bit 0) */
    RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;

    /* one count per micro, will overflow at 65.535 millis */
    TIM9->PSC = 99;

    TIM9->DIER = (1U << 0);

    /* bit 2 urs: 1: Only counter overflow generates an update interrupt if enabled. */
    TIM9->CR1 |= (1U << 2);

    TIM9->EGR = 1;
    TIM9->CNT = 0;
    /* Enable timer */
    TIM9->CR1 = TIM_CR1_CEN;

    /* If this interrupt happens the counter had an overflow, ie critical error */
    NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
}

/**
 * @author SurgeExperiments
 * @brief Use this when you want the timer to start counting from 0 again.
 */
void tim9_reset_count(void)
{
    TIM9->CNT = 0;
}

/**
 * @author SurgeExperiments
 * @brief Use this to read how much time has passed since the last timer-reset
 */
uint32_t tim9_read_count(void)
{
    return TIM9->CNT;
}

/**
 * @author SurgeExperiments
 * @brief Use this when you want a delay between 0 and 32.4 seconds.
 *
 * Do not use this for delays bigger than 32 seconds,
 * which counts to 64000, close to 65535 which is the max limit
 */
void tim9_delay_ms(uint32_t ms_to_delay)
{
    tim9_setup_for_delay_count_ms();

    /* 100mhz prescaled by 50k means 2k counts / sec */
    while ((TIM9->CNT) < ms_to_delay * 2)
        ;

    /* Disable the timer afterwards */
    TIM9->CR1 &= ~TIM_CR1_CEN;
}

/**
 * @author SurgeExperiments
 * @brief Use this when you want a delay between 0 and 32.4ms.
 */
void tim9_delay_us(uint32_t us_to_delay)
{
    tim9_setup_for_delay_count_us();
    while ((TIM9->CNT) < us_to_delay)
        ;
    TIM9->CR1 &= ~TIM_CR1_CEN;
}

/**************************
 * INTERRUPT-BASED TIMERS *
 *************************/

/**
 * @author SurgeExperiments
 * @brief NOTE: This function is not tested properly.
 */
uint32_t tim9_ms_passed(void)
{
    return tim9_ms_since_startup;
}

/**
 * @author SurgeExperiments
 * @brief NOTE: This function is not tested properly.
 */
uint32_t tim9_us_passed(void)
{

    /*
     * Bit 0 UIF: Update interrupt flag ? This bit is set by hardware on an update event.
     * It is cleared by software. 0: No update occurred. 1: Update interrupt pending.
     * This bit is set by hardware when the registers are updated: ? At overflow or
     * underflow (for TIM2 to TIM5) and if UDIS=0 in the TIMx_CR1 register.
     */
    tim9_timer_interrupt_ran = 0;
    uint32_t us_passed = tim9_ms_since_startup;
    uint32_t current_reg_count = TIM9->CNT;

    /* Millis updated in between, must reupdate */
    if (tim9_timer_interrupt_ran == 1)
    {
        us_passed += 1;
        /* Since it did an overflow, we need this to be the new (and lower) value */
        current_reg_count = TIM9->CNT;
    }

    /* 1000 timer ticks per millis means one per micro */
    return us_passed * 1000 + current_reg_count;
}

/********************************************************
 *					TIM10 functions 					*
 *											  			*
 ********************************************************/

/**
 * @author SurgeExperiments
 * @brief function that handles setting up TIM9 for interrupt timing.
 * 		  NOTE: This function is not tested properly.
 *
 * Due to prescaler-limits this timer will tick twice per ms
 * So multiply your desired millis when comparing with
 * the reg with 2. No more than 32.2 ms
 */
void tim10_setup_for_delay_count_ms(void)
{
    /* enable the clock for TIM2 (bit 0) */
    RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;

    /* This should have been 99999 for 1 count per ms, but the reg can only go to 65535 */
    TIM10->PSC = 49999;

    TIM10->EGR = 1;
    TIM10->CNT = 0;
    TIM10->CR1 = TIM_CR1_CEN;
}

/**
 * @author SurgeExperiments
 * @brief function that handles setting up TIM10 for delay timing. Works well.
 */
void tim10_setup_for_delay_count_us(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;

    /* one count per micro, will overflow at 65.535 millis */
    TIM10->PSC = 99;

    TIM10->EGR = 1;
    TIM10->CNT = 0;
    TIM10->CR1 = TIM_CR1_CEN;
}

/**
 * @author SurgeExperiments
 * @brief function that handles setting up TIM10 for delay timing. Works well.
 *
 * This sets up timing and tim10_delaycount_err_overflow==1 if the timer had an overflow.
 */
void tim10_setup_delay_count_us_int_warn(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;

    /* one count per micro, will overflow at 65.535 millis */
    TIM10->PSC = 99;

    TIM10->DIER = (1U << 0); // | (1U << 2);

    /* bit 2 urs: 1: Only counter overflow generates an update interrupt if enabled. */
    TIM10->CR1 |= (1U << 2);

    TIM10->EGR = 1;
    TIM10->CNT = 0;
    TIM10->CR1 = TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

/**
 * @author SurgeExperiments
 * @brief Use this when you want the timer to start counting from 0 again.
 */
void tim10_reset_count(void)
{
    TIM10->CNT = 0;
}

/**
 * @author SurgeExperiments
 * @brief Use this to read how much time has passed since the last timer-reset
 */
uint32_t tim10_read_count(void)
{
    return TIM10->CNT;
}

/**
 * @author SurgeExperiments
 * @brief Use this when you want a delay between 0 and 32.2 seconds.
 *
 * This function disables TIM10 afterwards,
 * don't use it while using TIM10 for something else
 */
void tim10_delay_ms(uint32_t ms_to_delay)
{
    tim10_setup_for_delay_count_ms();

    /* 100mhz prescaled by 50k means 2k counts / sec */
    while ((TIM10->CNT) < ms_to_delay * 2)
        ;
    TIM10->CR1 &= ~TIM_CR1_CEN;
}

/**
 * @author SurgeExperiments
 * @brief Use this when you want a delay between 0 and 65.5 ms.
 *
 * This function also disables TIM10 afterwards,
 * don't use it while using TIM10 for something else
 */
void tim10_delay_us(uint32_t us_to_delay)
{
    tim10_setup_for_delay_count_us();
    while ((TIM10->CNT) < us_to_delay)
        ;
    TIM10->CR1 &= ~TIM_CR1_CEN;
}

/****************
 * OTHER TIMERS *
 ***************/

/**
 * Call at least every 2^32 cycles (every 59.6 seconds @ 72 MHz).
 * Do not call from interrupt context!
 */

/*
uint64_t GetCycleCount64(void) {

  last_cycle_count_64 += DWT->CYCCNT - (uint32_t)(last_cycle_count_64);
  return last_cycle_count_64;
}


uint64_t getTimeViaCycles(void) {
  last_cycle_count_64 += DWT->CYCCNT - (uint32_t)(last_cycle_count_64);
  return last_cycle_count_64/100;
}
*/
