/**
 *  @file pwm_gen.c
 *  @author SurgeExperiments
 *
 *  @brief This file contains functions generate PWM on GPIO pins via timers.
 *
 *         init with enable_pwm_timer_2/3 () and then use set_pwm_freq() to
 *         change the frequency generated real-time.
 *
 * TODO:
 * - parameterize enable_pwm_timer_3() to allow vals of
 *   TIMx->PSC and TIMx->ARR to b set based on
 *   the current PLL settings (for now it only works with 100mhz)
 */

#include "pwm_gen.h"
#include "gpio_driver.h"

/**
 * @author SurgeExperiments
 *
 * @brief function that sets PWM-generation for a timer.
 *        Read the manual on the specific timer before using this.
 *        TODO: Verify that the assumption about apb running @ sysclock/2 holds
 *              for every chip in the STM32F4 series
 *              when generalizing this code to the entire series.
 *
 * @param TIMx CMSIS struct that represents which timer to use
 * @param prescaler_value check the reference manual.
 * @param esc_cycle_length check the reference manual + the manual for
 * 						   the esc (hint: [1000-2000 us])
 */
static void enable_timer_pwm_gen(TIM_TypeDef *TIMx,
                                 uint16_t prescaler_value,
                                 uint16_t esc_cycle_length)
{
    /*
     * these settings are based on the apb running off a 100mhz sysclock which
     * means apb speed of sysclock/2 (see system_stm32f4xx.c)
     * the output compare thing runs from the apb,
     * which is sysclock/2, so we divide the prescaler in half to get the 2040micros pwm
     */
    TIMx->PSC = prescaler_value;
    TIMx->ARR = esc_cycle_length;

    /* Set PWM duty cycle for channels 1-4 */
    TIMx->CCR1 = 0;
    TIMx->CCR2 = 0;
    TIMx->CCR3 = 0;
    TIMx->CCR4 = 0;

    /* PWM mode: channel 1 and 2 */
    TIMx->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;

    /* PWM mode: channel 3 and 4 */
    TIMx->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;

    /* Set compare mode on channels 1-4 */
    TIMx->CCER = TIM_CCER_CC4E | TIM_CCER_CC3E | TIM_CCER_CC2E | TIM_CCER_CC1E;

    /* Enable timer */
    TIMx->CR1 = TIM_CR1_CEN;
}

/**
 *  @author SurgeExperiments
 *
 *  @brief Function to enable PWM-generation on connected GPIO pins on Timer2.
 *         Args are described in detail in the reference manual.
 *
 *         For the flight controller we use this to control the ESC's.
 *         NOTE: for ESC's running simonK firmware:
 *               use: prescalerValue: 49, pwmCycleLength: 2040 or more
 *
 *         TODO: Turn pin selectors for gpio_init_pins_alt_func()
 *               if it's needed in the future. For now it would just cause clutter.
 *
 * @param prescaler_value Explained in the reference manual.
 * @param pwm_cycle_length Explained in the reference manual.
 */
void enable_pwm_timer_2(uint8_t prescaler_value, uint16_t pwm_cycle_length)
{
    /* enable clock */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    enable_timer_pwm_gen(TIM2, prescaler_value, pwm_cycle_length);

    /* Pins are at PC6 - PC9 */
    gpio_init_clock(GPIOA);
    gpio_init_pins_alt_func(GPIOA, 15, (uint8_t)GPIO_AF_TIM2, GPIO_SPEED_MEDIUM, GPIO_TYPE_PUSH_PULL, GPIO_NOPULL);
    gpio_init_pins_alt_func(GPIOA, 1, (uint8_t)GPIO_AF_TIM2, GPIO_SPEED_MEDIUM, GPIO_TYPE_PUSH_PULL, GPIO_NOPULL);
    gpio_init_pins_alt_func(GPIOA, 2, (uint8_t)GPIO_AF_TIM2, GPIO_SPEED_MEDIUM, GPIO_TYPE_PUSH_PULL, GPIO_NOPULL);
    gpio_init_pins_alt_func(GPIOA, 3, (uint8_t)GPIO_AF_TIM2, GPIO_SPEED_MEDIUM, GPIO_TYPE_PUSH_PULL, GPIO_NOPULL);
}

/**
 * @author SurgeExperiments
 * @brief function to set PWM-gen with Timer 3 with set values.
 *        (convenience function which sets the right pins for
 *        gpio_init_pins_alt_func())
 */
void enable_pwm_timer_3(uint8_t prescaler_value, uint16_t pwm_cycle_length)
{
    /* enable clock (T3) */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    enable_timer_pwm_gen(TIM3, prescaler_value, pwm_cycle_length);

    /* INFO:  Pins are at PC6 - PC9 */
    gpio_init_clock(GPIOC);
    gpio_init_pins_alt_func(GPIOC, 6, (uint8_t)GPIO_AF_TIM3, GPIO_SPEED_MEDIUM, GPIO_TYPE_PUSH_PULL, GPIO_NOPULL);
    gpio_init_pins_alt_func(GPIOC, 7, (uint8_t)GPIO_AF_TIM3, GPIO_SPEED_MEDIUM, GPIO_TYPE_PUSH_PULL, GPIO_NOPULL);
    gpio_init_pins_alt_func(GPIOC, 8, (uint8_t)GPIO_AF_TIM3, GPIO_SPEED_MEDIUM, GPIO_TYPE_PUSH_PULL, GPIO_NOPULL);
    gpio_init_pins_alt_func(GPIOC, 9, (uint8_t)GPIO_AF_TIM3, GPIO_SPEED_MEDIUM, GPIO_TYPE_PUSH_PULL, GPIO_NOPULL);
}

/**
 * @author SurgeExperiments
 * @brief function that sets the length of the PWM power part.

 * params are 1000-2000, which is the microsecond length of the "power" part of the PWM wave.
 *
 * @param TIMx which timer to use
 * @param channel1 Volt length of channel 1: [1000, 2000]
 * @param channel1 Volt length of channel 2: [1000, 2000]
 * @param channel1 Volt length of channel 3: [1000, 2000]
 * @param channel1 Volt length of channel 4: [1000, 2000]
 */
void set_pwm_freq(TIM_TypeDef *TIMx,
                  uint16_t channel1,
                  uint16_t channel2,
                  uint16_t channel3,
                  uint16_t channel4)
{
    TIMx->CCR1 = channel1;
    TIMx->CCR2 = channel2;
    TIMx->CCR3 = channel3;
    TIMx->CCR4 = channel4;
}
