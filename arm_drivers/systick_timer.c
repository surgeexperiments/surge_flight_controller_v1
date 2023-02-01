#include "systick_timer.h"

/**
 * NOTE: This file is depricated. Use timing_driver.c instead.
 */

/**
 * An uint32_t will overflow in 50 days of continous operation.
 * We can't use uint64_t as it isn't atomic on a 32 bit system.
 * If you need bigger space use two vars for lower and upper bits.
 */
static volatile uint32_t systick_overflow_count = 0;
static uint32_t systick_to_milliseconds = 0;

void init_systick_timer(void)
{
    systick_to_milliseconds = (SystemCoreClock / (1000 * 100));
    SysTick_Config(SystemCoreClock / 1000);
    NVIC_SetPriority(SysTick_IRQn, 0);
}

void SysTick_Handler(void)
{
    ++systick_overflow_count;
}

uint32_t micros_passed(void)
{
    /* sysTick counts down so (1000 � SysTick->VAL/sysTickToMilliseconds)
     * gives you the current microseconds
     */
    return systick_overflow_count * 1000 + (1000 - SYSTICK_VAL_REG / systick_to_milliseconds);
}

void delay_ms(uint32_t millisToDelay)
{
    uint32_t millis_passed_start = systick_overflow_count;
    while ((systick_overflow_count - millis_passed_start) < millisToDelay)
        ;
}

void delay_us(uint32_t microsToDelay)
{
    uint32_t micros_passed_start = micros_passed();
    while ((micros_passed() - micros_passed_start) < microsToDelay)
        ;
}
