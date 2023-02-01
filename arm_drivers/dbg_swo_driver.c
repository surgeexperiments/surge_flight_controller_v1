/**
 * @file dbg_swo_driver.c
 * @author Surge
 *
 * @brief Contains the DBG function that prints data to the swo dbg port.
 */


#include "stm32f4xx.h"
#include "dbg_swo_driver.h"


/**
 *  @author SurgeExperiments
 * 
 *  @brief Function to print a string to the swo debug port.
 *
 *  	   This function doesn't require calling any other function to
 *  	   initialize the functionality before using.
 * 
 *  	   Run STM32 ST-link's SWO viewer to see these strings.
 * 		   This function is useful for live reports from the running system.
 * 		   See the datasheet for information about the registers.
 *
 *  @param my_string is a null-terminated string to be sent to to the dbg swo terminal.
 *  @return number of characters written to the swo port or -1 on error
 */
int print_str_to_dbg_port(char *my_string)
{
	int counter=0;
	
	/* ITM enabled */
	if ((ITM->TCR & ITM_TCR_ITMENA_Msk) &&
		/* ITM Port #0 enabled */      
		 (ITM->TER & (1UL << 0)))				
	{
		while(my_string[counter] != '\0'){
			/* Wait until available */
			while (ITM->PORT[0].u32 == 0);
			/* Send character */			                
			ITM->PORT[0].u8 = (uint8_t) my_string[counter];			
			counter++; 
			}
		
		return counter;
	}
		 
	else {
		/* ITM error */
		return -1; 
	}
}

