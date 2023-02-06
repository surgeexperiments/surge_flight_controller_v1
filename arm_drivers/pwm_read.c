/**
 * @file pwm_read.c
 * @author SurgeExperiments
 *
 * @brief functions that sets up PWM-read on a timer/GPIO combination.
 *        Since a timer can be connected to several GPIO pins
 *        you can select the pin in the functions by changing defines.
 *        (the need for parameters here is non-existent for the time being)
 *
 *        READING INTERRUPT DATA: The pulse-length can be read from either the global
 *                                uint32_t variables or the RX_channels struct.
 *                                Globals are not popular, but in this case they
 *                                are needed.
 *
 *        Most arm interrupts (except systick) must be cleared by software,
 *        which is done in each interrupt handler in this module.
 *
 *        The HW timer is not influenced by these interrupts
 *
 *        INTERRUPT NAMES: These have to follow an existing naming pattern.
 *
 * Current setup for the flight controller:
 * - PA0: read PWM tim5_chn1: Pulse four
 * - PA8: read PWM with tim1_chn1: Pulse one
 * - PB4: read PWM with tim3_chn1: Pulse two
 * - PD4/PB6: read PWM with tim4_chn1: Pulse three
 */

#include "stm32f4xx.h"

#include "pwm_read.h"
#include "gpio_driver.h"
#include "../flight_controller/structs.h"

/**
 * Globals used by interrupts.
 * They need to be volatile to avoid shadowing.
 */
volatile uint32_t g_pwm_tot_pulse_length_tim1 = 0,
                  g_pwm_tot_pulse_length_tim3 = 0,
                  g_pwm_tot_pulse_length_tim4 = 0,
                  g_pwm_tot_pulse_length_tim5 = 0;

/* Global containing the latest RX channel pulse lengths in microseconds */
rxChannels_st g_rx_channel_pulses;

/**
 * @author SurgeExperiments
 * @brief interrupt-handler for PWM read on TIM1
 */
void TIM1_CC_IRQHandler(void)
{
    g_pwm_tot_pulse_length_tim1 = TIM1->CCR1;
    g_rx_channel_pulses.one = TIM1->CCR2;

    /* Clear the capture-event interrupt */
    if (TIM1->SR & (1U << 1))
    {
        TIM1->SR &= ~(1U << 1);
    }
}

/**
 * @author SurgeExperiments
 * @brief interrupt-handler for PWM read on TIM3
 */
void TIM3_IRQHandler(void)
{
    g_pwm_tot_pulse_length_tim3 = TIM3->CCR1;
    g_rx_channel_pulses.two = TIM3->CCR2;

    /* Clear the capture-event interrupt */
    if (TIM3->SR & (1U << 1))
    {
        TIM3->SR &= ~(1U << 1);
    }
}

/**
 * @author SurgeExperiments
 * @brief interrupt-handler for PWM read on TIM4
 */
void TIM4_IRQHandler(void)
{
    g_pwm_tot_pulse_length_tim4 = TIM4->CCR1;
    g_rx_channel_pulses.three = TIM4->CCR2;

    /* Clear the capture-event interrupt */
    if (TIM4->SR & (1U << 1))
    {
        TIM4->SR &= ~(1U << 1);
    }
}

/**
 * @author SurgeExperiments
 * @brief interrupt-handler for PWM read on TIM5
 */
void TIM5_IRQHandler(void)
{
    g_pwm_tot_pulse_length_tim5 = TIM5->CCR1;
    g_rx_channel_pulses.four = TIM5->CCR2;

    /* Clear the capture-event interrupt */
    if (TIM5->SR & (1U << 1))
    {
        TIM5->SR &= ~(1U << 1);
    }
}

/**
 * @author SurgeExperiments
 *
 * @brief: Function that sets up timer registers to read PWM
 *         on GPIO pins.
 *
 *         IMPORTANT: Run the processor at 100mhz to get the
 *                    correct timing.
 *                    If you run the processor at other speeds
 *                    you need to change the prescalers.
 *                    (see below: prescalers based on the PLL config coming)
 *
 *         NOTE: LIB ERROR: the define for TIM_CCMR1_CC1S is erroneous,
 *                          so (1U << 0) is used instead to clearify that.
 */
static void set_timer_registers_pwm_read(TIM_TypeDef *TIMx)
{
    /* REFERENCE MANUAL:
     * PWM input mode This mode is a particular case of input capture mode.
     * The procedure is the same except:
     * : Two ICx signals are mapped on the same TIx input.
     * : These 2 ICx signals are active on edges with opposite polarity.
     * : One of the two TIxFP signals is selected as trigger input and
     * 	 the slave mode controller is configured in reset mode.
     * For example, the user can measure the period (in TIMx_CCR1 register)
     * and the duty cycle (in TIMx_CCR2 register) of the
     * PWM applied on TI1 using the following procedure (depending on CK_INT
     * frequency and prescaler value):
     */

    TIMx->ARR = 0xFFFF - 1;

    /* TIM1 is on apb2 so it runs at a different speed on the
     * capture/compare stuff, thus we need a different prescaler
     */
    if (TIMx == TIM1)
    {
        TIMx->PSC = 99;
    }
    /* TIM2-5 are on the same bus, so they'll all use the same prescaler */
    else
    {
        TIMx->PSC = 49;
    }

    /* REFERENCE MANUAL:
     * Select the active input for TIMx_CCR1: write the CC1S bits to
     *  01 in the TIMx_CCMR1 register (TI1 selected).
     *: 01 : CC1 channel is configured as input, IC1 is mapped on TI1
     * NOTE: TIM_CCMR1_CC1S); LIB ERROR: THIS WAS AN ERRONEOUS DEFINE!
     */
    TIMx->CCMR1 |= (1U << 0);

    /* REFERENCE MANUAL:
     * Select the active polarity for TI1FP1 (used both for capture
     * in TIMx_CCR1 and counter clear): write the CC1P to �0�
     * and the CC1NP bit to '0' (active on rising edge).
     * Reset TIM4->CCMR1 |= TIM_CCMR1_CC2S_1; // 0x0200: sets the
     * two bits to 10ets both to 0, nothing needs to be done.
     */

    /* REFERENCE MANUAL:
     * Select the active input for TIMx_CCR2: write the CC2S bits to 10
     * in the TIMx_CCMR1 register (TI1 selected).
     * 10: CC2 channel is configured as input, IC2 is mapped on TI1.
     */

    TIMx->CCMR1 |= TIM_CCMR1_CC2S_1; // 0x0200: sets the two bits to 10

    /* REFERENCE MANUAL:
     * Select the active polarity for TI1FP2 (used for capture in TIMx_CCR2):
     * write the CC2P bit to '1' and the CC2NP bit
     * to '0'(active on falling edge).
     */
    TIMx->CCER = TIM_CCER_CC2P; // 0x0020, set bit 5 to 1

    /* REFERENCE MANUAL:
     * Select the valid trigger input: write the TS bits
     * to 101 in the TIMx_SMCR register (TI1FP1 selected).
     */
    /* write 101 starting at bit 4 to set mode 101: Filtered Timer Input 1 (TI1FP1) */
    TIMx->SMCR |= 0x50;

    /* REFERENCE MANUAL:
     * Configure the slave mode controller in reset mode:
     * write the SMS bits to 100 in the TIMx_SMCR register.
     * 100: Reset Mode - Rising edge of the selected trigger input (TRGI)
     * reinitializes the counter and generates an update of the registers.
     */
    TIMx->SMCR |= 0x4;

    /* REFERENCE MANUAL:
     * Enable the captures: write the CC1E and CC2E bits to
     * 1 in the TIMx_CCER register.
     */
    TIMx->CCER |= (1U << 0) | (1U << 4);

    /* REFERENCE MANUAL:
     * When TI1 rises, the counter is cleared and restarts from 0. In the meantime,
     * the trigger flag is set (TIF bit in the TIMx_SR register)
     * and an interrupt request, or a DMA request can be sent if enabled
     * (depending on the TIE and TDE bits in TIMx_DIER register).
     */

    /* Enable capture-event interrupts (this is the only interrupt we need) */
    TIMx->DIER |= (1U << 1);

    /* Enable timer */
    TIMx->CR1 |= TIM_CR1_CEN;
}

/**
 * @author SurgeExperiments
 * @brief init PWM-read on Timer 1. Read on PA8.
 *
 * IMPORTANT: can't use this and have an interrupt on TIM9.
 */
void init_pwm_read_timer_1(void)
{
    /* enable clock */
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    set_timer_registers_pwm_read(TIM1);
    NVIC_EnableIRQ(TIM1_CC_IRQn);
    gpio_init_clock(GPIOA);
    gpio_init_pins_alt_func(GPIOA, 8,
                            (uint8_t)GPIO_AF_TIM1,
                            GPIO_SPEED_MEDIUM,
                            GPIO_TYPE_PUSH_PULL,
                            GPIO_NOPULL);
}

/**
 * @author SurgeExperiments
 * @brief init PWM-read on Timer 1. Read on PB4.
 */
void init_pwm_read_timer_3(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    set_timer_registers_pwm_read(TIM3);
    NVIC_EnableIRQ(TIM3_IRQn);
    gpio_init_clock(GPIOB);
    gpio_init_pins_alt_func(GPIOB, 4,
                            (uint8_t)GPIO_AF_TIM3,
                            GPIO_SPEED_MEDIUM,
                            GPIO_TYPE_PUSH_PULL,
                            GPIO_NOPULL);
}

/**
 * @author SurgeExperiments
 * @brief init PWM-read on Timer 4. Select between PD4 and PB6
 */
void init_pwm_read_timer_4(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    set_timer_registers_pwm_read(TIM4);
    NVIC_EnableIRQ(TIM4_IRQn);
    gpio_init_clock(GPIOD);
    gpio_init_pins_alt_func(GPIOD, 12,
                            (uint8_t)GPIO_AF_TIM4,
                            GPIO_SPEED_MEDIUM,
                            GPIO_TYPE_PUSH_PULL,
                            GPIO_NOPULL);
}

/**
 * @author SurgeExperiments
 * @brief init PWM-read on Timer 1.
 */
void init_pwm_read_timer_5(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    set_timer_registers_pwm_read(TIM5);
    NVIC_EnableIRQ(TIM5_IRQn);
    gpio_init_clock(GPIOA);
    gpio_init_pins_alt_func(GPIOA, 0,
                            (uint8_t)GPIO_AF_TIM5,
                            GPIO_SPEED_MEDIUM,
                            GPIO_TYPE_PUSH_PULL,
                            GPIO_NOPULL);
}
