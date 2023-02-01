#include "rcc_driver.h"
#include "system_defines.h"


// Driver for the reset and clock control! RCC handles setting up the correct clocks
// This function is just copied from the ST standard perh lib and read through. No point in reinventing the wheel herer

static __I uint8_t APBAHBPrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};


// Configure the clock source(s) for peripherals which clocks are not
// derived from the System clock (I2S, RTC, ADC, USB OTG FS/SDIO/RNG)
// I2C is derived from the sys clock
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks)
{
  uint32_t tmp = 0, presc = 0, pllvco = 0, pllp = 2, pllsource = 0, pllm = 2;
#if defined(STM32F412xG) || defined(STM32F446xx)  
  uint32_t pllr = 2;
#endif /* STM32F412xG || STM32F446xx */
  
  /* Get SYSCLK source -------------------------------------------------------*/
  tmp = RCC->CFGR & RCC_CFGR_SWS;
  
  switch (tmp)
  {
  case 0x00:  /* HSI used as system clock source */
    RCC_Clocks->SYSCLK_Frequency = HSI_VALUE;
    break;
  case 0x04:  /* HSE used as system clock  source */
    RCC_Clocks->SYSCLK_Frequency = HSE_VALUE;
    break;
  case 0x08:  /* PLL P used as system clock  source */

    /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLN
    SYSCLK = PLL_VCO / PLLP
    */    
    pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
    pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
    
    if (pllsource != 0)
    {
      /* HSE used as PLL clock source */
      pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
			// pllvco == 400000000
			// sysclck: 400000000/4 == 100000000 == 100mhz
    }
    else
    {
      /* HSI used as PLL clock source */
      pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);      
    }
    
    pllp = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >>16) + 1 ) *2;
    RCC_Clocks->SYSCLK_Frequency = pllvco/pllp;
		// pllp: 4, pllvco == 400 
    break;
		
		/*
#if defined(STM32F412xG) || defined(STM32F446xx)
  case 0x0C:  // PLL R used as system clock  source
    // PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLN SYSCLK = PLL_VCO / PLLR 
    pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22;
    pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
    
    if (pllsource != 0)
    {
      // HSE used as PLL clock source
      pllvco = (HSE_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
    }
    else
    {
      // HSI used as PLL clock source
      pllvco = (HSI_VALUE / pllm) * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);      
    }
    
    pllr = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >>28) + 1 ) *2;
    RCC_Clocks->SYSCLK_Frequency = pllvco/pllr;    
    break;
#endif
		*/
		
  default:
    RCC_Clocks->SYSCLK_Frequency = HSI_VALUE;
    break;
  }
  /* Compute HCLK, PCLK1 and PCLK2 clocks frequencies ------------------------*/
  
  /* Get HCLK prescaler / ahb prescaler */
  tmp = RCC->CFGR & RCC_CFGR_HPRE;
	
  tmp = tmp >> 4; 
	// presc == table[ahbPrescaler], so presc == 0
  presc = APBAHBPrescTable[tmp];
  /* HCLK clock frequency */
	// 400000000
  RCC_Clocks->HCLK_Frequency = RCC_Clocks->SYSCLK_Frequency >> presc;
	
  /* Get PCLK1 prescaler */
  tmp = RCC->CFGR & RCC_CFGR_PPRE1;
  tmp = tmp >> 10;
	// 
  presc = APBAHBPrescTable[tmp];
  /* PCLK1 clock frequency_  */
  RCC_Clocks->PCLK1_Frequency = RCC_Clocks->HCLK_Frequency >> presc;

  /* Get PCLK2 prescaler */
  tmp = RCC->CFGR & RCC_CFGR_PPRE2;
  tmp = tmp >> 13;
  presc = APBAHBPrescTable[tmp];
  /* PCLK2 clock frequency */
  RCC_Clocks->PCLK2_Frequency = RCC_Clocks->HCLK_Frequency >> presc;
}


