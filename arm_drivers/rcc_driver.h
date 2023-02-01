
#ifndef RCC_DRIVER_
#define RCC_DRIVER_

#include "stm32f4xx.h"

typedef struct
{
    uint32_t SYSCLK_Frequency; /*!<  SYSCLK clock frequency expressed in Hz */
    uint32_t HCLK_Frequency;   /*!<  HCLK clock frequency expressed in Hz   */
    uint32_t PCLK1_Frequency;  /*!<  PCLK1 clock frequency expressed in Hz  */
    uint32_t PCLK2_Frequency;  /*!<  PCLK2 clock frequency expressed in Hz  */
} RCC_ClocksTypeDef;

void RCC_GetClocksFreq(RCC_ClocksTypeDef *RCC_Clocks);

#define RCC_APB1Periph_I2C1 ((uint32_t)0x00200000)
#define RCC_APB1Periph_I2C2 ((uint32_t)0x00400000)
#define RCC_APB1Periph_I2C3 ((uint32_t)0x00800000)

/*
(#) Several clock sources can be used to drive the System clock (SYSCLK): HSI,
          HSE and PLL.
          The AHB clock (HCLK) is derived from System clock through configurable
          prescaler and used to clock the CPU, memory and peripherals mapped
          on AHB bus (DMA, GPIO...). APB1 (PCLK1) and APB2 (PCLK2) clocks are derived
          from AHB clock through configurable prescalers and used to clock
          the peripherals mapped on these busses. You can use
          "RCC_GetClocksFreq()" function to retrieve the frequencies of these clocks.
*/

#endif /* RCC_DRIVER_ */
