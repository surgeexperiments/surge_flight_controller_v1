/**
 *	@file gpio_driver.h
 *	@author Surge
 *	@date 1/1/2019
 *
 *	@brief  This file contains prototypes for GPIO functions
 *			and defines used with GPIO.
 */

#ifndef GPIO_DRIVER_
#define GPIO_DRIVER_

#include "stm32f4xx.h"

void gpio_init_clock(GPIO_TypeDef* GPIOx);

void gpio_init_pins_alt_func(GPIO_TypeDef* GPIOx,
                             uint8_t pinNumber,
                             uint8_t gpioAFSelector,
                             uint8_t pinSpeed,
                             uint8_t outputType,
                             uint8_t pullType);

void gpio_init_pins_io(GPIO_TypeDef* GPIOx,
                       uint8_t pinNumber,
                       uint8_t inputOrOutput,
                       uint8_t outputType,
                       uint8_t pullType,
                       uint8_t pinSpeed);

void ConfigurePB12_extiRising(void); 
void ConfigurePE2_extiRising(void);
void SYSCFG_EXTILineConfig(uint8_t EXTI_PortSourceGPIOx,
                           uint8_t EXTI_PinSourcex);


#define EXTI_PortSourceGPIOA       ((uint8_t)0x00)
#define EXTI_PortSourceGPIOB       ((uint8_t)0x01)
#define EXTI_PortSourceGPIOC       ((uint8_t)0x02)
#define EXTI_PortSourceGPIOD       ((uint8_t)0x03)
#define EXTI_PortSourceGPIOE       ((uint8_t)0x04)
#define EXTI_PortSourceGPIOF       ((uint8_t)0x05)
#define EXTI_PortSourceGPIOG       ((uint8_t)0x06)
#define EXTI_PortSourceGPIOH       ((uint8_t)0x07)
#define EXTI_PortSourceGPIOI       ((uint8_t)0x08)
#define EXTI_PortSourceGPIOJ       ((uint8_t)0x09)
#define EXTI_PortSourceGPIOK       ((uint8_t)0x0A)

#define EXTI_PinSource0            ((uint8_t)0x00)
#define EXTI_PinSource1            ((uint8_t)0x01)
#define EXTI_PinSource2            ((uint8_t)0x02)
#define EXTI_PinSource3            ((uint8_t)0x03)
#define EXTI_PinSource4            ((uint8_t)0x04)
#define EXTI_PinSource5            ((uint8_t)0x05)
#define EXTI_PinSource6            ((uint8_t)0x06)
#define EXTI_PinSource7            ((uint8_t)0x07)
#define EXTI_PinSource8            ((uint8_t)0x08)
#define EXTI_PinSource9            ((uint8_t)0x09)
#define EXTI_PinSource10           ((uint8_t)0x0A)
#define EXTI_PinSource11           ((uint8_t)0x0B)
#define EXTI_PinSource12           ((uint8_t)0x0C)
#define EXTI_PinSource13           ((uint8_t)0x0D)
#define EXTI_PinSource14           ((uint8_t)0x0E)
#define EXTI_PinSource15           ((uint8_t)0x0F)

/*
The GPIOx_MODER register is used to select the I/O direction (input, output, AF, analog). 
The GPIOx_OTYPER and GPIOx_OSPEEDR registers are used to select the output type (push-pull or open-drain) and speed 
(the I/O speed pins are directly connected to the corresponding GPIOx_OSPEEDR register bits whatever the I/O direction). 
The GPIOx_PUPDR register is used to select the pull-up/pull-down whatever the I/O direction.

Each GPIO has two 16-bit memory-mapped data registers: input and output data registers (GPIOx_IDR and GPIOx_ODR). 
GPIOx_ODR stores the data to be output, it is read/write accessible. 
The data input through the I/O are stored into the input data register (GPIOx_IDR), a read-only register. 

Using the GPIOx_BSRR register to change the values of individual bits in GPIOx_ODR is a �one-shot� effect that does not lock the GPIOx_ODR bits. 
The GPIOx_ODR bits can always be accessed directly. The GPIOx_BSRR register provides a way of performing atomic bitwise handling.

Alternate function configuration When the I/O port is programmed as alternate function: 
� The output buffer can be configured as open-drain or push-pull 
� The output buffer is driven by the signal coming from the peripheral (transmitter enable and data) 
� The Schmitt trigger input is activated 
� The weak pull-up and pull-down resistors are activated or not depending on the value in the GPIOx_PUPDR register 
� The data present on the I/O pin are sampled into the input data register every AHB clock cycle 
� A read access to the input data register gets the I/O state 


PB8: TIM4_CH3, TIM10_CH1, I2C1_SCL
PB9: TIM4_CH4, TIM11_CH1, I2C1_SDA, SPI2_NSS/I2S2_WS, USB_FS_SDA, SDIO_D5
Assumption: GPIOB8 and 9
TM_GPIO_InitAlternate(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, TM_GPIO_OType_OD, TM_GPIO_PuPd_UP, TM_GPIO_Speed_Medium, GPIO_AF_I2C1);
*/

#define GPIO_AF_I2C1          ((uint8_t)0x04)  /* I2C1 Alternate Function mapping */
#define GPIO_AF_I2C2          ((uint8_t)0x04)  /* I2C2 Alternate Function mapping */
#define GPIO_AF_I2C3          ((uint8_t)0x04)  /* I2C3 Alternate Function mapping */

#define GPIO_AF_TIM1          ((uint8_t)0x01)  /* TIM1 Alternate Function mapping */
#define GPIO_AF_TIM2          ((uint8_t)0x01)  /* TIM2 Alternate Function mapping */
#define GPIO_AF_TIM3          ((uint8_t)0x02)
#define GPIO_AF_TIM4          ((uint8_t)0x02)  /* TIM4 Alternate Function mapping */
#define GPIO_AF_TIM5          ((uint8_t)0x02)  /* TIM5 Alternate Function mapping */

#define GPIO_AF_SPI1          ((uint8_t)0x05)  /* SPI1/I2S1 Alternate Function mapping */
#define GPIO_AF_SPI2          ((uint8_t)0x05)  /* SPI2/I2S2 Alternate Function mapping */
#define GPIO_AF_SPI3          ((uint8_t)0x06)  /* SPI3/I2S3 Alternate Function mapping */
#define GPIO_AF_SPI4          ((uint8_t)0x05)  /* SPI4/I2S4 Alternate Function mapping */

#define GPIO_NOPULL						((uint8_t)0x00)
#define GPIO_PULLUP 			    ((uint8_t)0x01)
#define GPIO_PULLDOWN			    ((uint8_t)0x02)
#define GPIO_MODE_INPUT   		((uint8_t)0x00)
#define GPIO_MODE_OUTPUT   		((uint8_t)0x01)
#define GPIO_MODE_AF 				  ((uint8_t)0x02)
#define GPIO_TYPE_PUSH_PULL  	((uint8_t)0x00)
#define GPIO_TYPE_OPEN_DRAIN  ((uint8_t)0x01)


#define GPIO_SPEED_LOW				((uint8_t)0x00)
#define GPIO_SPEED_MEDIUM			((uint8_t)0x01)
#define GPIO_SPEED_FAST				((uint8_t)0x02)
#define GPIO_SPEED_HIGH				((uint8_t)0x03)

/* Add legacy definition */
#define  GPIO_Speed_2MHz    GPIO_Low_Speed    
#define  GPIO_Speed_25MHz   GPIO_Medium_Speed 
#define  GPIO_Speed_50MHz   GPIO_Fast_Speed 
#define  GPIO_Speed_100MHz  GPIO_High_Speed  

#define GPIO_PIN_8		((uint16_t)0x0100)
#define GPIO_PIN_9		((uint16_t)0x0200)

/*
GPIO as Interrupt
Interrupt lines

I will show now how to configure GPIO pin to be an interrupt and how to handle it in your code with CMSIS function.

In section one (GPIOs) we have 16 interrupt lines. They are line0 to line15 and they also represent pin number. This means, 
PA0 is connected to Line0 and PA13 is connected to Line13.

You have to know that PB0 is also connected to Line0 and PC0 also and so on. This is for all pins on board, All Px0 
(where x is GPIO name) pins are connected to Line0 and let�s say all Px3 are connected to Line3 on the Interrupt channel.

All pins with same number are connected to line with same number. They are multiplexed to one line.

IMPORTANT: You can not use two pins on one line simultaneously:

    PA0 and PB0 and PC0 and so on, are connected to Line0, so you can use only one pin at one time to handle interrupt from there.
    PA0 and PA5 are connected to different lines, they can be used at the same time.

Each line can trigger an interrupt on rising, falling or rising_falling enge on signal.
Interrupt handlers

OK, now you have selected your pin you want to use. But you have to handle interrupt somehow. This process is described below.

STM32F4 has 7 interrupt handlers for GPIO pins. They are in table below:
Irq 	Handler 	Description
EXTI0_IRQn 	EXTI0_IRQHandler 	Handler for pins connected to line 0
EXTI1_IRQn 	EXTI1_IRQHandler 	Handler for pins connected to line 1
EXTI2_IRQn 	EXTI2_IRQHandler 	Handler for pins connected to line 2
EXTI3_IRQn 	EXTI3_IRQHandler 	Handler for pins connected to line 3
EXTI4_IRQn 	EXTI4_IRQHandler 	Handler for pins connected to line 4
EXTI9_5_IRQn 	EXTI9_5_IRQHandler 	Handler for pins connected to line 5 to 9
EXTI15_10_IRQn 	EXTI15_10_IRQHandler 	Handler for pins connected to line 10 to 15

This table show you which IRQ you have to set for NVIC (first column) and function names to handle your interrupts (second column). 
You have probably also figured, that only lines 0 to 4 have own IRQ handler. Yes, lines 5-9 have the same interrupt handler and this 
is also for lines 10 to 15.
*/

#endif /* GPIO_DRIVER_ */
