/**
 *	@file gpio_driver.c
 *	@author SurgeExperiments
 *
 *	@brief This file contains functions to init GPIO pins in various ways.
 */

#include "exti_p.h"
#include "gpio_driver.h"
#include "nvic_driver.h"

/**
 * @author SurgeExperiments
 *
 * 	@brief Function that activates the clock for GPIO A - E
 *
 * 	@param GPIOx describes which GPIO port to use
 */
void gpio_init_clock(GPIO_TypeDef *GPIOx)
{
    if (GPIOx == GPIOA)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    }
    else if (GPIOx == GPIOB)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    }
    else if (GPIOx == GPIOC)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    }
    else if (GPIOx == GPIOD)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    }
    else if (GPIOx == GPIOE)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    }
}

/**
 * @author SurgeExperiments
 *
 * @brief Function that does the entire init process for a GPIO
 * 		  pin with alternative function.
 *
 * 	  	  Many functions in the STM, like timer functions with GPIO requires these.
 * 	  	  NOTE: This function is a bit of a hack-job at the moment but it works
 * 				reliably for every mode.
 *
 * @param[out] GPIOx which gpio port
 * @param pin_number the pin number on the specific gpio port (usually 0-15)
 * @param gpio_af_selector which alternative function.
 * 						   See gpio_driver.h: defines named GPIO_AF_.
 * @param pin_speed See gpio_driver.h: defines named GPIO_SPEED_
 * @param outputType See gpio_driver.h: defines named GPIO_TYPE_
 * @param pullType See gpio_driver.h: defines GPIO_NOPULL, GPIO_PULLUP or GPIO_PULLDOWN
 */
void gpio_init_pins_alt_func(GPIO_TypeDef *GPIOx,
                             uint8_t pin_number,
                             uint8_t gpio_af_selector,
                             uint8_t pin_speed,
                             uint8_t output_type,
                             uint8_t pull_type)
{
    uint8_t afr_array_index = 0, afr_pin_shift_base = pin_number;

    /* GPIO_AF_I2C1 == b100, means AF4. shift left by 4 to set the second port to AF4 too */
    if (pin_number >= 8)
    {
        afr_array_index = 1;
        afr_pin_shift_base -= 8;
    }

    GPIOx->AFR[afr_array_index] |= (gpio_af_selector << afr_pin_shift_base * 4);
    GPIOx->MODER |= (GPIO_MODE_AF << (pin_number * 2));
    GPIOx->OSPEEDR |= (pin_speed << (pin_number * 2));
    GPIOx->OTYPER |= (output_type << pin_number);
    GPIOx->PUPDR |= (pull_type << (pin_number * 2));
}

/**
 * @author SurgeExperiments
 *
 * @brief Function that does the entire init process for a GPIO pin
 *
 * @param[out] GPIOx which gpio port
 * @param pin_number the pin number on the specific gpio port (32 bit reg: 0-15)
 * @param input_or_output See gpio_driver.h: defines named GPIO_MODE_INPUT or GPIO_MODE_OUTPUT
 * @param output_type See gpio_driver.h: defines named GPIO_TYPE_ for potential args
 * @param pull_type: See gpio_driver.h: defines GPIO_NOPULL, GPIO_PULLUP or GPIO_PULLDOWN
 * @param pin_speed See gpio_driver.h: defines named GPIO_SPEED_ for potential args
 */
void gpio_init_pins_io(GPIO_TypeDef *GPIOx,
                       uint8_t pin_number,
                       uint8_t input_or_output,
                       uint8_t output_type,
                       uint8_t pull_type,
                       uint8_t pin_speed)
{

    /* Input or output pin? */
    GPIOx->MODER |= (input_or_output << (pin_number * 2));

    /* Set the output type */
    GPIOx->OTYPER |= (output_type << pin_number);

    /* Set pull up resistor on PB8, PB9: shift 16 and 18 */
    // (GPIO_PULLUP << GPIO_PUPDR_PUPD8)|(GPIO_PULLUP << GPIO_PUPDR_PUPD9);
    GPIOx->PUPDR |= (pull_type << (pin_number * 2));

    /* Set the ospeed reg (put last?) */
    GPIOx->OSPEEDR |= (pin_speed << (pin_number * 2));
}

/**
 * @brief Test function to configure an interrupt on high->low on PB12
 * 		  (from stm lib)
 */
void ConfigurePB12_extiRising(void)
{
    gpio_init_clock(GPIOB);

    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    /* Set clock for syscfg */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    /* Set pin as input */
    GPIOB->MODER |= (GPIO_MODE_INPUT << 12 * 2);
    GPIOB->OTYPER |= (GPIO_TYPE_PUSH_PULL << 12);
    GPIOB->PUPDR |= (GPIO_NOPULL << 12 * 2);
    GPIOB->OSPEEDR |= (GPIO_SPEED_FAST << 12 * 2);

    /* Tell system that you will use PB12 for EXTI_Line12 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource12);

    /* PB12 is connected to EXTI_Line12 */
    EXTI_InitStruct.EXTI_Line = EXTI_Line12;
    /* Enable interrupt */
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Triggers on rising and |falling edge */
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising; // EXTI_Trigger_Rising;

    EXTI_Init(&EXTI_InitStruct);

    /* Add IRQ vector to NVIC */
    /* PB12 is connected to EXTI_Line12, which has EXTI15_10_IRQn vector */
    NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
    /* Set priority */
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
    /* Set sub priority */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
    /* Enable interrupt */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStruct);
}

/**
 * @author SurgeExperiments
 * @brief Test function to configure an interrupt on high->low on PE2
 */
void ConfigurePE2_extiRising(void)
{
    gpio_init_clock(GPIOE);

    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    /* Set clock for syscfg */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    /* Set pin as input */
    GPIOB->MODER |= (GPIO_MODE_INPUT << 2 * 2); //: input is default mode
    GPIOB->OTYPER |= (GPIO_TYPE_PUSH_PULL << 2);
    // GPIOB->OTYPER |= (GPIO_TYPE_OPEN_DRAIN << 2);
    // GPIOB->PUPDR |=  (GPIO_PULLUP << 12*2);
    GPIOB->PUPDR |= (GPIO_PULLDOWN << 2 * 2);
    GPIOB->OSPEEDR |= (GPIO_SPEED_FAST << 2 * 2);

    /* Tell system that you will use PE2 for EXTI_Line2 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource2);

    /* PE12 is connected to EXTI_Line2 */
    EXTI_InitStruct.EXTI_Line = EXTI_Line2;
    /* Enable interrupt */
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Triggers on rising edge */
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling; // EXTI_Trigger_Rising;
    /* Add to EXTI */
    EXTI_Init(&EXTI_InitStruct);

    /* Add IRQ vector to NVIC */
    /* PE2 is connected to EXTI_Line12, which has EXTI15_10_IRQn vector */
    NVIC_InitStruct.NVIC_IRQChannel = EXTI2_IRQn; // Assumption : this is correct
    /* Set priority */
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
    /* Set sub priority */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
    /* Enable interrupt */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    /* Add to NVIC */

    NVIC_Init(&NVIC_InitStruct);
}
