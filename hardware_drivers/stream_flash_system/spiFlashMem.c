/**
 * @author SurgeExperiments 
 * @brief low level driver for flash mem W25Q64FV.
 * 	  	  A handler for writing data streams (lower level than files) are 
 * 	  	  also included. 
 *
 * NOTE: This file is a mess. It has not yet been ported.
 * 
 * TODO: this is not finished porting from avr to arm.
 */ 

#include "../arm_drivers/timing_driver.h"
#include "../arm_drivers/spi_driver.h"

#include "flashMem.h"
#include "streamWrite.h"
#include "spiFlashMem.h"

/**  
 * @brief init spi
 *
 * TODO: add a version for arm 
 */
void setupSpi(void){

    /* TODO: add spi functions from SPI_driver.c */
    SPI_InitTypeDef mySpiInitDef; 
}


/**  
 * @brief the name says it all
 * 	  Check the comment in the function and the reference manual 
 * 	  for specifics. 
 */
uint8_t isChipBusy(void){
	/* The Read Status Register instruction may be used at any time, even while a Program,
	 * Erase or Write Status Register cycle is in progress. 
	 * This allows the BUSY status bit to be checked to determine when the cycle is complete
	 * and if the device can accept another instruction. 
	 * The Status Register can be read continuously
	 */
	spiEnable();	// Sanity?
	ENABLE_CHIP;
	transferByte(READSTATUS1REG_INSTRUCTION);
	uint8_t statReg1Data = transferReceiveByte(0x00);
	DISABLE_CHIP; 
	// Is status-flag 1 cleared?
	if (statReg1Data & 1){
		// Chip is busy
		return 1;		
	}
	// Chip is not busy
	return 0;							
}


void waitUntilChipIsReadyNoTimeout(void){
	while(1){
		if(isChipBusy() == 0) break; 
	}
}


/**
 * @brief name says it all
 *
 * TODO: update for STM32F4
 *
 */
uint8_t waitUntilChipIsReadyWithTimeout(uint16_t thresholdTime){
	/* Return 0 if timeout, return 1 if chip is ready upon function exit */
	long initTime = millis_passed(); 
	
	while(1){
		if(isChipBusy() == 0) break;
		if((millis_passed() - initTime) > thresholdTime) return 0; 
	}
	return 1; 
}


uint8_t isChipWriteable(void){
	// TODO: use?
	return 1; 
}



/**
 * @brief self explanatory. 
 *
 * TODO: remove magic constants? Needed?   
 */
uint8_t isAddressValid(uint32_t memoryAddress){
	//32,768 pages (256 bytes per page)
	//  7FFFFFh
	if((memoryAddress < 0) || (memoryAddress > 0x7FFFFF)) {
		return 0;
	} else {
		return 1;
	}
}


/** 
 * @brief Returns the first memory page start address after the parameter addr
 *        If the param is the start of a memory address, then param is returned
 */
uint32_t getNextPageAddress(uint32_t memoryAddress){
	if((memoryAddress & 0xFF) == 0) {
		return memoryAddress; 
	}
	return ((memoryAddress + 0xFF) & 0x7FFF00); 
}


// Borrowed from another lib (convenience)
uint32_t page2address(uint16_t page_number, uint8_t offset){
	uint32_t address = page_number;
	return ((address << 8) + offset);
}



/* INFO_GATHERING FUNCTIONS  
*/
int getUniqueID(uint8_t *outputData){
	int myReturnValue = 0;
	myReturnValue = ((outputData[6] << 8) | outputData[7]);
	
	return myReturnValue;
}


void getUniqueIDArray(uint8_t *outputData){
	ENABLE_CHIP;
	transferByte(READUNIQUEID_INSTRUCTION);
    
    uint8_t n;     

	/* Flash out dummy-bytes (see datasheet page 22) */
    for(n=0; n < 4; n++) {
        transferReceiveByte(0x00);
    }

	/* Most significant bit first, 64 bits total */
    for(n=0; n < 8; n++) {
        outputData[n] = transferReceiveByte(0x00);
    }
	DISABLE_CHIP;
}


void getJDECId(uint8_t *IDInfo){
	
	// Select chip
	ENABLE_CHIP;
	transferReceiveByte(GETJDECID_INSTRUCTION);
	IDInfo[0] = transferReceiveByte(0x00); 
	IDInfo[1] = transferReceiveByte(0x00);
	IDInfo[2] = transferReceiveByte(0x00);
	
	// Deselect chip
	DISABLE_CHIP;
	//USART_sendString("entered\n");
}



/* CHANGE SETTINGS FUNCTIONS 
*/
 void writeDisable(void){
	 waitUntilChipIsReadyNoTimeout(); 
	 ENABLE_CHIP;
	 transferByte(WRITEDISABLE_INSTRUCTION);
	 DISABLE_CHIP;
	 delay_ms(1); // dbg sanity
 }


 void writeEnable(void){
	 waitUntilChipIsReadyNoTimeout(); 
	 ENABLE_CHIP;
	 transferByte(WRITEENABLE_INSTRUCTION);
	 DISABLE_CHIP;
	 delay_ms(1); // dbg sanity
 }



/* CHIP FUNCTIONS 
*/
void eraseChip(void){
	writeEnable();
	ENABLE_CHIP;
	transferByte(ERASECHIP_INSTRUCTION);
	DISABLE_CHIP;
	// Do not return from this function until the erase function is complete
	waitUntilChipIsReadyNoTimeout(); 
}


void powerDownChip(void){
	// TODO: Add some additional checks or settings here?
	waitUntilChipIsReadyNoTimeout(); 
	ENABLE_CHIP;
	transferByte(POWERDOWN_INSTRUCTION);
	DISABLE_CHIP;
}


void powerUpChip(void){
	// TODO: Add some additional checks or settings here?
	// TODO: Add a 5 ms delay here to conform to the datasheet, or is waitUntilChipIsReadyNoTimeout() sufficient? test
	ENABLE_CHIP;
	transferByte(POWERUP_INSTRUCTION);
	DISABLE_CHIP;
	waitUntilChipIsReadyNoTimeout();	// Should b about 5 millis b4 writing should occur
	writeEnable();						// Power-up disables writing
}


/** 
 * @author SurgeExperiments
 * 
 * @brief method that utilizes the winbond fast-read method 
 * 	  (see the reference manual). 
 *
 * 	  IMPORTANT: fastread runs @ 104mhz, regular read @ 80. 
 * 	  	         If you screw this up data can b trashed. 
 *
 * @retval 0 on error, else unsigned num of bytes read 
 */
uint32_t fastReadBytes(uint32_t address, uint8_t *buffer, uint32_t numberOfBytesToRead)
{
	
	/* The chip's addr increments automatically with each read, no size restrictions besides not going above the last addr
	 * From the reference manual:
	 * The Read Data instruction sequence is shown in figure 10. If a Read Data instruction is issued while an Erase, 
	 * Program or Write cycle is in process (BUSY=1) the instruction is ignored and will not have any effects on the current cycle.
	 */
	
	/* Cannot read to an address outside the address space or read less than 1 byte */
	if((address < 0) || (address > LASTFLASH_MEMADDR) || (numberOfBytesToRead < 1)) return 0;
	
	/* Since the first read starts @ the addr (0) we subtract 1 from (addr + stringlength) to get the last addr written to */
	if((address + numberOfBytesToRead - 1) > LASTFLASH_MEMADDR) {
		/* Need to set the stringlength so it only writes bytes in the interval [addr, lastaddr]
		 * NOTE: This works when we loop from (i=0; i < stringLength; i++), ie the last index must NOT be inclusive! 
		 */
		numberOfBytesToRead  = LASTFLASH_MEMADDR - address + 1;
	}

	/* TODO: This timeout value is somewhat arbitrary (based on the ref manual?). 
         *       The quad can't handle too many of these if it blocks the main flight loop. 
     	 *       Need some kind of queue or DMA based transfer (won't take up much loop time) to do this safely.
     	 *       Should also block writes and log if too many of these happens regardless.  
     	 */	
	if(waitUntilChipIsReadyWithTimeout(500) == 0) return 0; 
    
	ENABLE_CHIP;
	
	transferByte(FASTREAD_INSTRUCTION); 
	
	/* Push the read address */
	transferByte(address >> 16);	// Instr: A23-A16
	transferByte(address >> 8);		// Instr: A15-A8
	transferByte(address);			// Instr: A7-A0
	
	/* Instr: dummy (required to get the clock synch working, see ref manual */
	transferByte(0x00);			

	uint16_t i; 
	for (i = 0; i < numberOfBytesToRead; i++) {
		buffer[i] = transferReceiveByte(0x00);
	}
		
	DISABLE_CHIP;
	
	/* This covers all bytes read */
	return numberOfBytesToRead; 
 }


/** 
 * @author SurgeExperiments
 * @brief normal read that runs @ 80mhz (don't screw that up or you'll get bogus data) 
 * 
 * TODO: add an err-retval for erroneous addresses? 
 *
 * @retval 0 on error, else num of bytes read
 */
uint32_t readBytes(uint32_t address, uint8_t *buffer, uint32_t numberOfBytesToRead) {
	
	/* The chip's addr increments automatically with each read, no size restrictions besides not going above the last addr
	* From the dataSheet:
	* The Read Data instruction sequence is shown in figure 10. If a Read Data instruction is issued while an Erase, 
	* Program or Write cycle is in process (BUSY=1) the instruction is ignored and will not have any effects on the current cycle.
	*/
	
	/* Cannot read to an address outside the address space or read less than 1 byte */
	if((address < 0) || (address > LASTFLASH_MEMADDR) || (numberOfBytesToRead < 1)) return 0;
	
	/* Since the first read starts @ the addr (0) we subtract 1 from (addr + stringlength) to get the last addr written to */
	if((address + numberOfBytesToRead - 1) > LASTFLASH_MEMADDR) {
		/* Need to set the stringlength so it only writes bytes in the interval [addr, lastaddr]
		 * NOTE: This works when we loop from (i=0; i < stringLength; i++), ie the last index must NOT be inclusive! 
		 */
		numberOfBytesToRead  = LASTFLASH_MEMADDR - address + 1;
	}
    
	/* Timeout is not arbitrary, any longer than this and the chip has malfunctioned 99% of the time */	
	if(waitUntilChipIsReadyWithTimeout(10) == 0) return 0; 
	
	ENABLE_CHIP;
	
	transferByte(READDATA_INSTRUCTION);
	
	/* Push the read address */
	transferByte(address >> 16);
	transferByte(address >> 8 );
	transferByte(address);
	 
	/* Get data */
	uint16_t i; 
	for (i = 0; i < numberOfBytesToRead; i++) {
		buffer[i] = transferReceiveByte(0x00);
	}
	
	DISABLE_CHIP;
	
	/* Either all or nothing */
	return numberOfBytesToRead; 
 }


/**
 * @author SURGE_EXPERIMENTS!
 * @brief this function handles incrementing pages
 *        When the function returns: Chip will be write-disabled here due to page-program being finished 
 * 
 * @retval number of bytes written on success, 0 on error (which includes attempts to write to invalid addresses)
 * 
 * INFO: (reference manual)
 * If an entire 256 byte page is to be programmed, the last address byte (the 8 least significant address bits) should be set to 0. 
 * If the last address byte is not zero, and the number of clocks exceed the remaining page length, the addressing will wrap 
 * to the beginning of the page. In some cases, less than 256 bytes (a partial page) can be programmed without having any effect on other 
 * bytes within the same page. One condition to perform a partial page program is that the number of clocks can not exceed 
 * the remaining page length. If more than 256 bytes are sent to the device the addressing will wrap to the beginning of the page and 
 * overwrite previously sent data. As with the write and erase instructions, the /CS pin must be driven high after the eighth bit of 
 * the last byte has been latched. If this is not done the Page Program instruction will not be executed. After /CS is driven high, 
 * the self-timed Page Program instruction will commence for a time duration of tpp (See AC Characteristics). While the Page Program 
 * cycle is in progress, the Read Status Register instruction may still be accessed for checking the status of the BUSY bit. 
 * The BUSY bit is a 1 during the Page Program cycle and becomes a 0 when the cycle is finished and the device is ready to accept other 
 * instructions again. 
 * After the Page Program cycle has finished the Write Enable Latch (WEL) bit in the Status Register is cleared to 0. The Page Program
 * instruction will not be executed if the addressed page is protected by the Block Protect (CMP, SEC, TB, BP2, BP1, and BP0) bits.
 * 
 * NOTE: You can ONLY WRITE ON A PREVIOUSLY ERASED MEMORY SECTION! Use sector-erase first if you need 2 overwrite data
 */

uint32_t writeBytes(uint32_t address, uint8_t *myString, uint32_t stringLength){	
	/* Cannot write to an address outside the address space or write less than 1 byte */
	if((address < 0) || (address > LASTFLASH_MEMADDR) || (stringLength < 1)) return 0; 
	
	/* Check if the stringLength is so long that it causes writes beyond the highest address */
	if((address + stringLength - 1) > LASTFLASH_MEMADDR) {
		/* Need to set the stringlength so it only writes bytes in the interval [addr, lastaddr]
		 * NOTE: This works when we loop from (i=0; i < stringLength; i++), ie the last index must NOT be inclusive! 
		 */
		stringLength  = LASTFLASH_MEMADDR - address + 1;
	}
	
	/* Chip needs a little time to finish the operations (reference manual) */
	if(waitUntilChipIsReadyWithTimeout(10) == 0) return 0; 
	
	uint32_t bytesThatCanBeWrittenOnthisPage, remainingBytesOnThisPage, remainingBytesToWrite, bytesWritten=0; // = nextPageAddr - address; 
	uint8_t *dataArray = myString; 
    
	while(stringLength > bytesWritten){
		
		/* Compute the remaining bytes that can be written on this page */
		remainingBytesOnThisPage = 256 - (address & 0xFF);
		remainingBytesToWrite = stringLength - bytesWritten; 
		bytesThatCanBeWrittenOnthisPage = (remainingBytesToWrite > remainingBytesOnThisPage) ? remainingBytesOnThisPage : remainingBytesToWrite; 
        
		/* TODO: this value is arbitrary and too high for the quad to use unless the 
         *       SPI-bus is controlled by the DMA (as this would block the flight loop)
         */
		if(waitUntilChipIsReadyWithTimeout(500) == 0) return bytesWritten; 
		
		/* Page program will disable writing */
		writeEnable();
		
		ENABLE_CHIP;
		
		transferByte(WRITEDATA_INSTRUCTION);
		transferByte(address >> 16);
		transferByte(address >> 8 );
		transferByte(address);
		
		uint32_t x; 
		for(x = 0; x < bytesThatCanBeWrittenOnthisPage; x++){
			transferByte(dataArray[x]);
		}
		
		DISABLE_CHIP;
		
		/* Get write-starting addr for the next loop iteration */
		address += bytesThatCanBeWrittenOnthisPage;
		bytesWritten += bytesThatCanBeWrittenOnthisPage; 
		/* Get the next byte to write in the data-array */
		dataArray += bytesThatCanBeWrittenOnthisPage; 
	}
    
	return bytesWritten; 
 }

/**
 * @brief This function handles incrementing pages ect. It also updates the "page written" space when it fills up a page
 * 
 * @retval 0 on err, 1 if success
 */
uint32_t writeBytesForStreamWrite(uint32_t address, uint8_t *myString, uint32_t stringLength, uint16_t *dim1Byte, uint16_t *dim2Byte) {
	/* Note: You can ONLY WRITE ON A PREVIOUSLY ERASED MEMORY SECTION! Use sector-erase first if you need 2 overwrite data */
	
	/* Cannot write to an address outside the address space or write less than 1 byte
	 * If you manage to OVERFLOW the addr or something as a caller: fuck off :P
	 */
    if((address < 0) || (address > STREAMWRITE_LASTMEMADDR) || (stringLength < 1)) return 0; 
	
	if((address + stringLength - 1) > STREAMWRITE_LASTMEMADDR) {
		/* Need to set the stringlength so it only writes bytes in the interval [addr, lastaddr]
		 * NOTE: This works when we loop from (i=0; i < stringLength; i++), ie the last index must NOT be inclusive! 
		 */
		stringLength  = STREAMWRITE_LASTMEMADDR - address + 1;
	}
	
	if(waitUntilChipIsReadyWithTimeout(10) == 0) return 0; 
	
	uint32_t bytesThatCanBeWrittenOnthisPage, remainingBytesOnThisPage, remainingBytesToWrite, bytesWritten=0; // = nextPageAddr - address; 
	uint8_t *dataArray = myString; 
	
	while(stringLength > bytesWritten){	

		/* Compute the remaining bytes that can be written on this page */
		remainingBytesOnThisPage = 256 - (address & 0xFF);
		remainingBytesToWrite = stringLength - bytesWritten; 
		bytesThatCanBeWrittenOnthisPage = (remainingBytesToWrite > remainingBytesOnThisPage) ? remainingBytesOnThisPage : remainingBytesToWrite; 
		
		/* Let's give the chip a bit of time to finish previous operations */
		if(waitUntilChipIsReadyWithTimeout(5) == 0) return bytesWritten; 
		
		/* Page program will disable writing, enable it on each run */
		writeEnable();
		
		ENABLE_CHIP;
		
		transferByte(WRITEDATA_INSTRUCTION);
		transferByte(address >> 16);
		transferByte(address >> 8 );
		transferByte(address);
		
		uint32_t x; 
		for(x = 0; x < bytesThatCanBeWrittenOnthisPage; x++){
			transferByte(dataArray[x]);
		}
		
		DISABLE_CHIP;
		
		/* Get write-starting addr for the next loop iteration */
		address += bytesThatCanBeWrittenOnthisPage;
		bytesWritten += bytesThatCanBeWrittenOnthisPage; 
		/* Get the next byte to write in the data-array */
		dataArray += bytesThatCanBeWrittenOnthisPage; 

		/* update freeSpaceTable: If the next free addr is the next page, it means that this entire page was filled
		 * addr can not b 0 if write succeeds, must be at least 1
         */
		if(address%0x100 == 0){ 
            /* TODO: clearify comments */
			/* Ex: If addr is 0x8200, first free page is 3
			-> nothing to update in dim1, dim2 page0 should get byte 1 written
			*/
				
			/* Each loop will only increment max one page. This can happen in two ways  
			 */
			
			/* dim1Byte and dim2Byte: Points to the first free byte
			 * If the last free page (from before) is 127, the current dim2 is filled, and we need to increment dim1 with 1 byte
			 * The write-loop itself won't go above the last addr, so a dim1Write won't overwrite dim2-data (not that one overwrite will matter)
			 */
            if(*dim2Byte >= 127){
				/* Since dim1Byte is incremented, we don't need to write the last byte in dim2Byte
				 * This saves cycles and millis
				 */
                writeBytes(*dim1Byte, (uint8_t *)"5", 1); 
				++(*dim1Byte);
				*dim2Byte = 0; 
			} else {
				// TODO: Write some more tests for this addr calculation? 
				writeBytes((*dim1Byte)*0x80 + *dim2Byte + 0x100, (uint8_t *)"5", 1); 
				++(*dim2Byte); 
			}
		} 
	} 
	
	/* Chip will be write-disabled here due to page-program being finished */
	return bytesWritten; 
 }



uint8_t isAddrPrevWritten(uint32_t address, uint32_t size){
	ENABLE_CHIP;
	transferByte(READDATA_INSTRUCTION);
	uint16_t i; 
	for (i = 0; i < size; i++) {
		if (transferReceiveByte(0x00) != 0xFF) {
			DISABLE_CHIP;
			return 0;
		}
	}
	DISABLE_CHIP;
	return 1;
}


uint8_t eraseSector(uint32_t address){
	// TODO: Add a write-enable check here? 
	if(waitUntilChipIsReadyWithTimeout(10) == 0) return 0; 
	
	writeEnable();
	
	ENABLE_CHIP; 
	transferByte(ERASESECTOR_INSTRUCTION); 
	transferByte(address >> 16);
	transferByte(address >> 8 );
	transferByte(address);
	//transferReceiveByte(0x00);  required?
	DISABLE_CHIP; 
	// Sector erase 4kb should take max 400ms
	if(waitUntilChipIsReadyWithTimeout(410) == 0) return 0; 
	return 1; 
	// TODO: Add a check that the chip is NOT busy after the required 400ms?
}


uint8_t erase32KBBlock(uint32_t address){
	// TODO: Add a write-enable check here?
	if(waitUntilChipIsReadyWithTimeout(10) == 0) return 0; 
	
	writeEnable();
	
	ENABLE_CHIP;
	transferByte(BLOCKERASE32KB_INSTRUCTION);
	transferByte(address >> 16);
	transferByte(address >> 8 );
	transferByte(address);
	//transferReceiveByte(0x00);  required?
	
	DISABLE_CHIP;
	
	//BLock erase 32kb should take max 1200ms
	if(waitUntilChipIsReadyWithTimeout(1210) == 0) return 0;
	
	return 1; 
	// TODO: Add a check that the chip is NOT busy after the required 400ms?
}


uint8_t erase64KBBlock(uint32_t address){
	// TODO: Add a write-enable check here?
	if(waitUntilChipIsReadyWithTimeout(10) == 0) return 0; 
	
	writeEnable();
	
	ENABLE_CHIP;
	transferByte(BLOCKERASE64KB_INSTRUCTION);
	transferByte(address >> 16);
	transferByte(address >> 8 );
	transferByte(address);
	//transferReceiveByte(0x00);  required?
	
	DISABLE_CHIP;
	
	//Block erase 64kb should take max  2000ms
	if(waitUntilChipIsReadyWithTimeout(2010) == 0) return 0;
	
	return 1; 
	// TODO: Add a check that the chip is NOT busy after the required 400ms?
}



/* UTILITY */
void readWriteStringMetaHandler(uint32_t address, char *readBuffer, char *string2Write, uint32_t string2WriteSize){
	
	// Read address-area before any write
	readBytes(address, (uint8_t *)readBuffer, string2WriteSize);
	USART_sendString("Memory before the write: ");
	USART_sendString(readBuffer);
	USART_sendString("\n");
	delay_ms(100); // Sanity
	
	// Write data to address
	writeBytes(address, (uint8_t *)string2Write, string2WriteSize);
	delay_ms(100); // Sanity
	
	// Read address again and print it
	readBytes(address, (uint8_t *)readBuffer, string2WriteSize);
	USART_sendString("Memory AFTER the write: ");
	USART_sendString(readBuffer);
	USART_sendString("\n");
	
}
