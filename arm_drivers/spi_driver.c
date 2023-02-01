/**
 *	@file spi_driver.c
 *	@author SurgeExperiments
 *
 *	@brief This file contains functions to init and use SPI.
 */

#include "spi_driver.h"
#include "gpio_driver.h"

/**
 * @brief set up a SPI-struct for the quad.
 *
 * TODO: move this into the HW driver, replace with generic
 */
static void SPI_StructInit(SPI_InitTypeDef *SPI_InitStruct)
{
    /*--------------- Reset SPI init structure parameters values -----------------*/
    /* Initialize the SPI_Direction member */
    SPI_InitStruct->SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    /* initialize the SPI_Mode member */
    SPI_InitStruct->SPI_Mode = SPI_Mode_Master;
    /* initialize the SPI_DataSize member */
    SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
    /* Initialize the SPI_CPOL member */
    SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
    /* Initialize the SPI_CPHA member */
    SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
    /* Initialize the SPI_NSS member */
    SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;
    /* Initialize the SPI_BaudRatePrescaler member */
    SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    /* Initialize the SPI_FirstBit member */
    SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
    /* Initialize the SPI_CRCPolynomial member */
    SPI_InitStruct->SPI_CRCPolynomial = 7;
}

/**
 * @author SurgeExperiments
 *
 * @brief init SPI for LGD20 on the STM32F411 discovery card
 * 		  NOTE: normally I would not add a sensor specific
 * 				init function in a library, however this
 * 				sensor is included on the dev board :)
 *
 * @param SPIx SPI-instance for the SPI port you wanna use
 * @param spiMode the spi mode you want to use (although
 * 				  the LGD20 lol has ONE mode rofl).
 * 				  See spi_driver.h for potential values.
 */
void spi_init_lgd20_devboard(SPI_TypeDef *SPIx, uint8_t spi_mode)
{
    if (SPIx == SPI1)
    {
        /* Init clock */
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

        /*
        SPI1:
        - PA4: SPI1_NSS
        - PA5: SPI1_SCK!
        - PA6: SPI1_MISO!
        - PA7: SPI1_MOSI !
        */
        gpio_init_clock(GPIOA);
        gpio_init_pins_alt_func(GPIOA, 5, GPIO_AF_SPI1, GPIO_SPEED_HIGH, GPIO_TYPE_PUSH_PULL, GPIO_NOPULL);
        gpio_init_pins_alt_func(GPIOA, 6, GPIO_AF_SPI1, GPIO_SPEED_HIGH, GPIO_TYPE_PUSH_PULL, GPIO_NOPULL);
        gpio_init_pins_alt_func(GPIOA, 7, GPIO_AF_SPI1, GPIO_SPEED_HIGH, GPIO_TYPE_PUSH_PULL, GPIO_NOPULL);
    }

    /* TODO: Add pin init if needed */
    else if (SPIx == SPI2)
    {
        RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    }
    else if (SPIx == SPI3)
    {
        RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
    }
    else if (SPIx == SPI4)
    {
        RCC->AHB2ENR |= RCC_APB2ENR_SPI4EN;
    }

    SPI_InitTypeDef spi_init_st;
    SPI_StructInit(&spi_init_st);

    /*
    The SPI clock is derived from the APB1 or the
    APB2 bus depending on which SPI channel you use.

    The APB1 and APB2 bus speed is set by dividing the sysclock by a prescaler
    APB2: RCC->CFGR |= RCC_CFGR_PPRE2_DIV2: runs at 50mhz when the sysclock is 100mhz
    APB1: RCC->CFGR |= RCC_CFGR_PPRE1_DIV4: Runs at 25 mhz when the sysclock is 100mhz

    Your prescaler for the SPI must divide these APB bus speeds
    to meet the desired speed of the slave, ect 10mhz max means APB2 must be
    divided by the smallest prescaler >=5
    : APB2freq/maxFreqForSlave == 50mhz/10mhz == 5
    */

    /* On a sysclock of 100mhz the APB2/div2 runs at 50mhz.
     * Using the SPI_8 prescaller means the SPI now runs at 6.25 mhz
     */
    spi_init_st.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;

    /* SPI mode */
    if (spi_mode == 0)
    {
        spi_init_st.SPI_CPOL = SPI_CPOL_Low;
        spi_init_st.SPI_CPHA = SPI_CPHA_1Edge;
    }
    else if (spi_mode == 1)
    {
        spi_init_st.SPI_CPOL = SPI_CPOL_Low;
        spi_init_st.SPI_CPHA = SPI_CPHA_2Edge;
    }
    else if (spi_mode == 2)
    {
        spi_init_st.SPI_CPOL = SPI_CPOL_High;
        spi_init_st.SPI_CPHA = SPI_CPHA_1Edge;
    }
    else if (spi_mode == 3)
    {
        spi_init_st.SPI_CPOL = SPI_CPOL_High;
        spi_init_st.SPI_CPHA = SPI_CPHA_2Edge;
    }

    /* Disable SPI before initialization */
    SPIx->CR1 &= ~SPI_CR1_SPE;

    uint16_t tmpreg = 0;

    /*---------------------------- SPIx CR1 Configuration ------------------------*/
    /* Get the SPIx CR1 value */
    tmpreg = SPIx->CR1;

    /* Clear BIDIMode, BIDIOE, RxONLY, SSM, SSI, LSBFirst, BR, MSTR, CPOL and CPHA bits */
    tmpreg &= SPI_CR1_CLEAR_MASK;

    /* Configure SPIx: direction, NSS management, first transmitted bit, BaudRate prescaler
        master/salve mode, CPOL and CPHA */
    /* Set BIDImode, BIDIOE and RxONLY bits according to SPI_Direction value */
    /* Set SSM, SSI and MSTR bits according to SPI_Mode and SPI_NSS values */
    /* Set LSBFirst bit according to SPI_FirstBit value */
    /* Set BR bits according to SPI_BaudRatePrescaler value */
    /* Set CPOL bit according to SPI_CPOL value */
    /* Set CPHA bit according to SPI_CPHA value */
    tmpreg |= (uint16_t)((uint32_t)spi_init_st.SPI_Direction | spi_init_st.SPI_Mode |
                         spi_init_st.SPI_DataSize | spi_init_st.SPI_CPOL |
                         spi_init_st.SPI_CPHA | spi_init_st.SPI_NSS |
                         spi_init_st.SPI_BaudRatePrescaler |
                         spi_init_st.SPI_FirstBit);

    /* Write to SPIx CR1 */
    SPIx->CR1 = tmpreg;

    /* Activate the SPI mode (Reset I2SMOD bit in I2SCFGR register) */
    SPIx->I2SCFGR &= (uint16_t) ~((uint16_t)SPI_I2SCFGR_I2SMOD);

    /*---------------------------- SPIx CRCPOLY Configuration --------------------*/
    /* Write to SPIx CRCPOLY */
    SPIx->CRCPR = spi_init_st.SPI_CRCPolynomial;

    /* Enable spi */
    SPIx->CR1 |= SPI_CR1_SPE;
}

/**
 * @author SurgeExperiments
 * @brief send a uint8_t via SPI
 *
 * @param SPIx struct telling which SPI bus
 * @param data which data to send
 */
uint8_t spi_send_u8(SPI_TypeDef *SPIx, uint8_t data)
{

    /* Wait for previous transmissions to complete if DMA TX enabled for SPI */
    SPI_WAIT(SPIx);

    /* Fill output buffer with data */
    SPIx->DR = data;
    /* Wait for transmission to complete */
    SPI_WAIT(SPIx);

    /* Return data from buffer */
    return SPIx->DR;
}

/* INFO:

� MISO: Master In / Slave Out data. This pin can be used to transmit data in slave mode and receive data in master mode.
� MOSI: Master Out / Slave In data. This pin can be used to transmit data in master mode and receive data in slave mode.
� SCK: Serial Clock output for SPI masters and input for SPI slaves.
� NSS: Slave select. This is an optional pin to select a slave device. This pin acts as a �chip select� to let the SPI master communicate with
       slaves individually and to avoid contention on the data lines. Slave NSS inputs can be driven by standard IO ports on the master device.
             The NSS pin may also be used as an output if enabled (SSOE bit) and driven low if the SPI is in master configuration. In this manner,
             all NSS pins from devices connected to the Master NSS pin see a low level and become slaves when they are configured in NSS hardware mode.
             When configured in master mode with NSS configured as an input (MSTR=1 and SSOE=0) and if NSS is pulled low, the SPI enters the master mode fault state:
             the MSTR bit is automatically cleared and the device is configured in slave mode

The master initiates the transfer.
Full duplex is supported as the slave can get data on MOSI while transferring on MISO

Slave select (NSS) pin management Hardware or software slave select management can be set using the SSM bit in the SPI_CR1 register.
� Software NSS management (SSM = 1) The slave select information is driven internally by the value of the SSI bit in the SPI_CR1 register.
  The external NSS pin remains free for other application uses.
� Hardware NSS management (SSM = 0) Two configurations are possible depending
    on the NSS output configuration (SSOE bit in register SPI_CR2). � NSS output enabled (SSM = 0, SSOE = 1)
    This configuration is used only when the device operates in master mode.
    The NSS signal is driven low when the master starts the communication and is kept low until the SPI is disabled.

Clock phase and clock polarity Four possible timing relationships may be chosen by software, using the CPOL and CPHA bits in the SPI_CR1 register.
The CPOL (clock polarity) bit controls the steady state value of the clock when no data is being transferred. This bit affects both master and slave modes.
If CPOL is reset, the SCK pin has a low-level idle state. If CPOL is set, the SCK pin has a high-level idle state. If the CPHA (clock phase) bit is set,
the second edge on the SCK pin (falling edge if the CPOL bit is reset, rising edge if the CPOL bit is set) is the MSBit capture strobe.
Data are latched on the occurrence of the second clock transition. If the CPHA bit is reset, the first edge on the SCK pin
(falling edge if CPOL bit is set, rising edge if CPOL bit is reset) is the MSBit capture strobe. Data are latched on the occurrence of the first clock transition.
The combination of the CPOL (clock polarity) and CPHA (clock phase) bits selects the data capture clock edge.

Figure 194, shows an SPI transfer with the four combinations of the CPHA and CPOL bits. The diagram may be interpreted as a master or slave timing diagram where the
SCK pin, the MISO pin, the MOSI pin are directly connected between the master and the slave device. Note: Prior to changing the CPOL/CPHA bits the SPI must
be disabled by resetting the SPE bit. Master and slave must be programmed with the same timing mode. The idle state of SCK must correspond to the polarity
selected in the SPI_CR1 register (by pulling up SCK if CPOL=1 or pulling down SCK if CPOL=0). The Data Frame Format (8- or 16-bit) is selected through the
DFF bit in SPI_CR1 register, and determines the data length during transmission/reception.

Configuring the SPI in master mode In the master configuration, the serial clock is generated on the SCK pin.
Procedure
    1. Select the BR[2:0] bits to define the serial clock baud rate (see SPI_CR1 register).
    2. Select the CPOL and CPHA bits to define one of the four relationships between the data transfer and the serial clock (see Figure 194).
    This step is not required when the TI mode is selected.
    3. Set the DFF bit to define 8- or 16-bit data frame format
    4. Configure the LSBFIRST bit in the SPI_CR1 register to define the frame format. This step is not required when the TI mode is selected.
    5. If the NSS pin is required in input mode, in hardware mode, connect the NSS pin to a high-level signal during the complete byte transmit sequence.
       In NSS software mode, set the SSM and SSI bits in the SPI_CR1 register. If the NSS pin is required in output mode, the SSOE bit only should be set.
         This step is not required when the TI mode is selected.
    6. Set the FRF bit in SPI_CR2 to select the TI protocol for serial communications.
    7. The MSTR and SPE bits must be set (they remain set only if the NSS pin is connected to a high-level signal).
       In this configuration the MOSI pin is a data output and the MISO pin is a data input. Transmit sequence The transmit sequence begins when a byte is written in the Tx Buffer.
         The data byte is parallel-loaded into the shift register (from the internal bus) during the first bit transmission and then shifted out serially to the MOSI pin MSB first
         or LSB first depending on the LSBFIRST bit in the SPI_CR1 register. The TXE flag is set on the transfer of data from the Tx Buffer to the shift register and an interrupt
         is generated if the TXEIE bit in the SPI_CR2 register is set.


Transmit sequence
The transmit sequence begins when a byte is written in the Tx Buffer. The data byte is parallel-loaded into the shift register
(from the internal bus) during the first bit transmission and then shifted out serially to the MOSI pin MSB first or LSB first depending
on the LSBFIRST bit in the SPI_CR1 register. The TXE flag is set on the transfer of data from the Tx Buffer to the shift register and an
interrupt is generated if the TXEIE bit in the SPI_CR2 register is set.

Receive sequence For the receiver, when data transfer is complete:
� The data in the shift register is transferred to the RX Buffer and the RXNE flag is set
� An interrupt is generated if the RXNEIE bit is set in the SPI_CR2 register At the last sampling clock edge the RXNE bit is set,
  a copy of the data byte received in the shift register is moved to the Rx buffer. When the SPI_DR register is read, the SPI peripheral
    returns this buffered value. Clearing the RXNE bit is performed by reading the SPI_DR register. A continuous transmit stream can
    be maintained if the next data to be transmitted is put in the Tx buffer once the transmission is started. Note that TXE flag should be
    �1 before any attempt to write the Tx buffer is made. Note: When a master is communicating with SPI slaves which need to be de-selected between
    transmissions, the NSS pin must be configured as GPIO or another GPIO must be used and toggled by software.

E-compass MEMS (ST MEMS LSM303DLHC) The LSM303DLHC is an ultra-compact low-power system-in-package featuring a
3D digital linear acceleration sensor and a 3D digital magnetic sensor. It includes a sensing element and an
IC interface able to provide the measured acceleration to the external world through an I2C serial interface.
The LSM303DLHC has dynamically user-selectable full scales of � 2g/� 8g and is capable of measuring the acceleration, and a
magnetic field full scale from �1.3 g to 8.1 g with an output data rate of 100 Hz or 400 Hz.
The STM32F411VET6 MCU controls this motion sensor through the I2C interface.

SCL: PB6/I2C1_SCL
SDA: PB9/I2C1_SDA
PE2: DataRdy

4.8 Gyroscope MEMS (ST MEMS L3GD20) The L3GD20 is an ultra-compact, low-power, three-axis angular rate sensor.
It includes a sensing element and an IC interface able to provide the measured angular rate to the external world
through the I2C/SPI serial interface. The L3GD20 has dynamically user-selectable full scales of � 250 dps/500 dps/�2000 dps and is capable of measuring rates.

SCL/SPC: PA5/SPI1_SCK
SDO: PA6/SPI1_MISO
SDA/SDI/SDO: PA7/SPI1_MOSI
PE0: INT1, PE1: INT2
PE3: CS_i2c/SPI

*/

// OType/OutputTypw: pushPull,
// No pull resistor
// Gpio speed high
// AF functions: GPIO_AF_SPI1

/* SPI pins:

SPI1:
- PA4: SPI1_NSS
- PA5: SPI1_SCK!
- PA6: SPI1_MISO!
- PA7: SPI1_MOSI !


SPI2:
- PB9: SPI2_NSS
- PB10: SPI2_SCK!
- PB12: SPI2_NSS
- PB13: SPI2_SCK!
- PB14: SPI2_MISO!
- PB15: SPI2_MOSI!
- PC2: SPI2_MISO!
- PC3: SPI2_MOSI!

SPI3:
- PA4: SPI3_NSS
- PA15: SPI3_NSS
- PB3: SPI3_SCK!
- PB4: SPI3_MISO!
- PB5: SPI3_MOSI!
- PC10: SPI3_SCK!
- PC11: SPI3_MISO	!
- PC12: SPI3_MOSI!

SPI4:
- PE11: SPI4_NSS
- PE12: SPI4_SCK!
- PE13: SPI4_MISO!
- PE14: SPI4_MOSI !

*/
