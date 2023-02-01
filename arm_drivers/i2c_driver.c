/**
 *	@file i2c_driver.c
 *	@author SurgeExperiments
 *
 *	@brief This file contains functions to init and use i2c.
 *		   To set up i2c transfers controlled by DMA: check the functions in DMA_driver.c
 */

#include "i2c_driver.h"
#include "gpio_driver.h"
#include "rcc_driver.h"

/****************************************************
 *					INIT FUNCTIONS					*
 *													*
 ****************************************************/

/**
 * @author SurgeExperiments
 * @brief function that loads the init struct for the flight controller.
 * 		  TODO: Define this somewhere else later? For now this is ok
 * 				since the controller only uses 400khz for the i2c bus
 * 				which is the only setting we care about.
 */
void i2c_load_init_struct_fc(I2C_InitTypeDef *instance)
{
    // For now it's hard-coded just to get things working
    instance->I2C_ClockSpeed = 400000;
    instance->I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    instance->I2C_Mode = I2C_Mode_I2C;
    instance->I2C_OwnAddress1 = 0x00;
    instance->I2C_Ack = I2C_Ack_Disable;
    instance->I2C_DutyCycle = I2C_DutyCycle_2;
}

/**
 * @author SurgeExperiments
 * @brief Function that sets up i2c (1-3) transfer for a GPIO pin.
 *
 *		  NOTE:One i2c bus can b connected to several pins.
 *		  	   For now one set of pins are selected in this
 *			   function in a way that fits with the use case
 *			   for this flight controller.
 *			   The need to change them is so rare that parameterizing
 *			   this function would only cause clutter.
 *
 * @param I2Cx struct describing which i2c bus to use
 * @retval 0 if success, 1 if fail (due to invalid value of I2Cx)
 */
uint8_t i2c_initialize(I2C_TypeDef *I2Cx)
{
    /* - Program the peripheral input clock in I2C_CR2 Register in order to generate correct timings
     * - Configure the clock control registers
     * - Configure the rise time register
     * -  Program the I2C_CR1 register to enable the peripheral
     * - Set the START bit in the I2C_CR1 register to generate a Start condition
     * - The peripheral input clock frequency must be at least:
     *   2 MHz in Sm mode � 4 MHz in Fm mode
     */

    if (I2Cx == I2C1)
    {
        /* RCC APB1 peripheral clock enable register (RCC_APB1ENR): Enable the clock for i2c1 */
        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

        /* PB8 + 9 */
        gpio_init_clock(GPIOB);
        gpio_init_pins_alt_func(GPIOB, 8, GPIO_AF_I2C1, GPIO_SPEED_MEDIUM, GPIO_TYPE_OPEN_DRAIN, GPIO_PULLUP);
        gpio_init_pins_alt_func(GPIOB, 9, GPIO_AF_I2C1, GPIO_SPEED_MEDIUM, GPIO_TYPE_OPEN_DRAIN, GPIO_PULLUP);
    }
    else if (I2Cx == I2C2)
    {
        RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

        /* PB3: I2C2_SDA + PB10: I2C2_SCL */
        gpio_init_clock(GPIOB);
        gpio_init_pins_alt_func(GPIOB, 3, GPIO_AF_I2C2, GPIO_SPEED_MEDIUM, GPIO_TYPE_OPEN_DRAIN, GPIO_PULLUP);
        gpio_init_pins_alt_func(GPIOB, 10, GPIO_AF_I2C2, GPIO_SPEED_MEDIUM, GPIO_TYPE_OPEN_DRAIN, GPIO_PULLUP);
    }
    else if (I2Cx == I2C3)
    {
        RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;

        /* PA8: I2C3_SCL +  PB4: I2C3_SDA */
        gpio_init_clock(GPIOA);
        gpio_init_clock(GPIOB);
        gpio_init_pins_alt_func(GPIOA, 8, GPIO_AF_I2C3, GPIO_SPEED_MEDIUM, GPIO_TYPE_OPEN_DRAIN, GPIO_PULLUP);
        gpio_init_pins_alt_func(GPIOB, 4, GPIO_AF_I2C3, GPIO_SPEED_MEDIUM, GPIO_TYPE_OPEN_DRAIN, GPIO_PULLUP);
    }
    else
    {
        return 1;
    }

    /* Disable i2c before initialization */
    I2Cx->CR1 &= ~I2C_CR1_PE;

    I2C_InitTypeDef I2C_InitTypeDefInstance;
    i2c_load_init_struct_fc(&I2C_InitTypeDefInstance);

    i2c_setupRegisters(I2Cx, &I2C_InitTypeDefInstance);

    /* Re-enable i2c */
    I2Cx->CR1 |= I2C_CR1_PE;

    return 0;
}

/**
 * @brief Function that sets up the i2c registers in various ways when initializing an i2c bus.
 *		  (borrowed from stdPerph library, very convenient for prototyping)

 * @param I2Cx describing which i2c bus to use
 * @param I2C_InitStruct gives desired settings for the i2c bus.
 * 						 Use i2c_load_init_struct_fc() to set it up.
 */
void i2c_setupRegisters(I2C_TypeDef *I2Cx, I2C_InitTypeDef *I2C_InitStruct)
{
    uint16_t tmpreg = 0, freqrange = 0;
    uint16_t result = 0x04;
    uint32_t pclk1 = 8000000;
    RCC_ClocksTypeDef rcc_clocks;

    /*---------------------------- I2Cx CR2 Configuration ------------------------*/
    /* Get the I2Cx CR2 value */
    tmpreg = I2Cx->CR2;
    /* Clear frequency FREQ[5:0] bits */
    tmpreg &= (uint16_t) ~((uint16_t)I2C_CR2_FREQ);
    /* Get pclk1 frequency value */
    RCC_GetClocksFreq(&rcc_clocks);

    // To use the I2C at 400 KHz (in fast mode), the PCLK1 frequency (I2C peripheral input clock) must be a multiple of 10 MHz.
    // Is 25000000 @ 100mhz, still works!
    pclk1 = rcc_clocks.PCLK1_Frequency;
    /* Set frequency bits depending on pclk1 value */
    freqrange = (uint16_t)(pclk1 / 1000000);
    tmpreg |= freqrange;
    /* Write to I2Cx CR2 */
    I2Cx->CR2 = tmpreg;

    /*---------------------------- I2Cx CCR Configuration ------------------------*/
    /* Disable the selected I2C peripheral to configure TRISE */
    I2Cx->CR1 &= (uint16_t) ~((uint16_t)I2C_CR1_PE);
    /* Reset tmpreg value */
    /* Clear F/S, DUTY and CCR[11:0] bits */
    tmpreg = 0;

    /* Configure speed in standard mode */
    if (I2C_InitStruct->I2C_ClockSpeed <= 100000)
    {
        /* Standard mode speed calculate */
        result = (uint16_t)(pclk1 / (I2C_InitStruct->I2C_ClockSpeed << 1));
        /* Test if CCR value is under 0x4*/
        if (result < 0x04)
        {
            /* Set minimum allowed value */
            result = 0x04;
        }
        /* Set speed value for standard mode */
        tmpreg |= result;

        /* Set Maximum Rise Time for standard mode */
        I2Cx->TRISE = freqrange + 1;
    }
    /* Configure speed in fast mode */
    /* To use the I2C at 400 KHz (in fast mode), the PCLK1 frequency (I2C peripheral
     input clock) must be a multiple of 10 MHz */
    else /*(I2C_InitStruct->I2C_ClockSpeed <= 400000)*/
    {
        if (I2C_InitStruct->I2C_DutyCycle == I2C_DutyCycle_2)
        {

            /* Fast mode speed calculate: Tlow/Thigh = 2 */
            result = (uint16_t)(pclk1 / (I2C_InitStruct->I2C_ClockSpeed * 3));
        }
        else /*I2C_InitStruct->I2C_DutyCycle == I2C_DutyCycle_16_9*/
        {
            /* Fast mode speed calculate: Tlow/Thigh = 16/9 */
            result = (uint16_t)(pclk1 / (I2C_InitStruct->I2C_ClockSpeed * 25));
            /* Set DUTY bit */
            result |= I2C_DutyCycle_16_9;
        }

        /* Test if CCR value is under 0x1*/
        if ((result & I2C_CCR_CCR) == 0)
        {
            /* Set minimum allowed value */
            result |= (uint16_t)0x0001;
        }
        /* Set speed value and set F/S bit for fast mode */
        tmpreg |= (uint16_t)(result | I2C_CCR_FS);
        /* Set Maximum Rise Time for fast mode */
        I2Cx->TRISE = (uint16_t)(((freqrange * (uint16_t)300) / (uint16_t)1000) + (uint16_t)1);
    }

    /* Write to I2Cx CCR */
    I2Cx->CCR = tmpreg;
    /* Enable the selected I2C peripheral */
    I2Cx->CR1 |= I2C_CR1_PE;

    /*---------------------------- I2Cx CR1 Configuration ------------------------*/
    /* Get the I2Cx CR1 value */
    tmpreg = I2Cx->CR1;
    /* Clear ACK, SMBTYPE and  SMBUS bits */
    tmpreg &= CR1_CLEAR_MASK;
    /* Configure I2Cx: mode and acknowledgement */
    /* Set SMBTYPE and SMBUS bits according to I2C_Mode value */
    /* Set ACK bit according to I2C_Ack value */
    tmpreg |= (uint16_t)((uint32_t)I2C_InitStruct->I2C_Mode | I2C_InitStruct->I2C_Ack);
    /* Write to I2Cx CR1 */
    I2Cx->CR1 = tmpreg;

    /*---------------------------- I2Cx OAR1 Configuration -----------------------*/
    /* Set I2Cx Own Address1 and acknowledged address */
    I2Cx->OAR1 = (I2C_InitStruct->I2C_AcknowledgedAddress | I2C_InitStruct->I2C_OwnAddress1);
}

/**
 * @author SurgeExperiments
 *
 * @brief function that checks the current status of an i2c transfer.
 *
 * This function is used heavily by other functions to detect errors.
 *
 * @param I2Cx which i2c bus to use (CMSIS struct)
 * @param STATUS_FLAG which i2c flag to test for. These are defined in i2c_driver.h
 * @retval 1 if STATUS_FLAG is set in the respective i2c bus, 0 if it's not.
 */
static uint8_t check_i2c_status(I2C_TypeDef *I2Cx, uint32_t STATUS_FLAG)
{
    uint32_t cr1 = 0, cr2 = 0, full_reg = 0;
    cr1 = I2Cx->SR1;
    cr2 = I2Cx->SR2;
    cr2 = (cr2 << 16);
    full_reg = (cr1 | cr2) & FLAG_MASK;
    if ((full_reg & STATUS_FLAG) == STATUS_FLAG)
    {
        return (uint8_t)1;
    }
    else
    {
        return (uint8_t)0;
    }
}

/********************************************************
 *					USEFUL FUNCTIONS					*
 *														*
 ********************************************************/

/**
 * @brief function to check if something is connected to the i2c port
 *
 * TODO: Finish
 *
 * @param I2Cx which i2c bus to use
 * @param address which i2c address to check
 */
uint8_t i2c_check_connectivity(I2C_TypeDef *I2Cx, uint8_t address)
{
    return 1;
}

/********************************************************
 *					READWRITE FUNCTIONS					*
 *														*
 ********************************************************/

/**
 * @author SurgeExperiments
 *
 * @brief function that starts an i2c transfer.
 *		  By default: clock stretching is enabled.
 *
 * 		  See the reference manual for more info about the
 * 		  specifics of the i2c implementation.
 *
 * @param I2Cx which i2c bus to use
 * @param target_address the address for the item you want to communicate with
 * @param am_i_receiver set True when being a MASTER_RECEIVER,
 * 					    else false/0. See ref manual for more info.
 * @retval 0 if success, 1 if a hw err leads to a timeout.
 */
uint8_t i2c_start(I2C_TypeDef *I2Cx, uint8_t target_address, uint8_t am_i_receiver)
{
    uint16_t time_out_counter = 0;

    /* Generate a start condition */
    I2Cx->CR1 |= I2C_CR1_START;

    /* Check success and time out (to avoid inf loops on failure) */
    while (check_i2c_status(I2Cx, I2C_EVENT_MASTER_MODE_SELECT) == 0)
    {
        if (++time_out_counter == TIMEOUT_THRESHOLD)
        {
            /* We timed out, error! */
            return (uint8_t)1;
        }
    }

    /* Enable ack for addr */
    I2Cx->CR1 |= I2C_CR1_ACK;

    /* Send addr */
    I2Cx->DR = target_address;

    time_out_counter = 0;
    if (am_i_receiver == 1)
    {
        /* We only check for this event in receiver mode */
        while (check_i2c_status(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == 0)
        {
            if (++time_out_counter == TIMEOUT_THRESHOLD)
            {
                /* We timed out, error! */
                return (uint8_t)1;
            }
        }
    }
    else
    {
        while (check_i2c_status(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == 0)
        {
            if (++time_out_counter == TIMEOUT_THRESHOLD)
            {
                /* We timed out, error! */
                return (uint8_t)1;
            }
        }
    }

    /* Reference manual: clearing addr bit:
     * This bit is cleared by software reading SR1 register
     * followed reading SR2, or by hardware when PE=0.
     */
    I2Cx->SR1;
    I2Cx->SR2;

    return (uint8_t)0;
}

/**
 * @author SurgeExperiments
 *
 * @brief function that finishes an i2c communication sequence, per i2c protocol.
 *
 * @param I2Cx which i2c bus to use (CMSIS struct)
 */
uint8_t i2c_stop(I2C_TypeDef *I2Cx)
{
    uint32_t time_out_counter = 0;

    /* Data register and byte register must be empty before we can send the stop condition */
    while ((!(I2Cx->SR1 & I2C_SR1_BTF)) || (!(I2Cx->SR1 & I2C_SR1_TXE)))
    {
        if (++time_out_counter == TIMEOUT_THRESHOLD)
        {
            /* We timed out, error! */
            return (uint8_t)1;
        }
    }

    /* Gen stop condition */
    I2Cx->CR1 |= I2C_CR1_STOP;

    return (uint8_t)0;
}

/**
 * @author SurgeExperiments
 *
 * @brief function to send a byte throgh the i2c bus.
 *
 * INFO: For info about the different parts of the function:
 * 		 check the reference manual, i2c section.
 *
 * TODO: add a timeout threshold for I2C_FLAG_AF? (for now not needed?)
 *
 * @param I2Cx which i2c bus to use
 * @param byte_to_send self explanatory
 * @retval 0 if success, 1-3 if the call fails (the number describes what went wrong)
 */
uint8_t i2c_write_byte(I2C_TypeDef *I2Cx, uint8_t byte_to_send)
{

    uint32_t time_out_counter = 0;

    /* Data register and byte register must be empty before we can send the stop condition */
    while (!(I2Cx->SR1 & I2C_SR1_TXE))
    {
        if (++time_out_counter == TIMEOUT_THRESHOLD)
        {
            /* We timed out, error! */
            return (uint8_t)1;
        }
    }

    /* Write the byte */
    I2Cx->DR = byte_to_send;

    time_out_counter = 0;

    /* Wait for required event */
    while ((check_i2c_status(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == 0))
    {
        if (++time_out_counter == TIMEOUT_THRESHOLD)
        {
            return (uint8_t)2;
        }
    }

    /* If we didnt get an ack, we return an error condition */
    if (check_i2c_status(I2Cx, I2C_FLAG_AF) == 1)
    {
        return (uint8_t)3;
    }

    return (uint8_t)0;
}

/**
 * @author SurgeExperiments
 *
 * @brief function to read a byte and ack back.
 *
 * For details check the reference manual
 *
 * @param[in] I2Cx which i2c bus to use
 * @param[out] store_here lol
 * @retval 0 on success, 1 if the transfer fails due to TIMEOUT_THRESHOLD
 *		   being reached.
 */
uint8_t i2c_read_ack(I2C_TypeDef *I2Cx, uint8_t *store_here)
{
    /* Enable ack */
    I2Cx->CR1 |= I2C_CR1_ACK;

    /* Wait until the i2c bus is ready to read */
    uint32_t time_out_counter = 0;
    while (check_i2c_status(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) == 0)
    {
        if (++time_out_counter == TIMEOUT_THRESHOLD)
        {
            return (uint8_t)1;
        }
    }

    *store_here = (uint8_t)I2Cx->DR;

    return 0;
}

/**
 * @author SurgeExperiments
 *
 * @brief function to read a byte and nak
 *
 * @param[in] I2Cx which i2c bus to use
 * @param[out] store_here pointer to uint8_t which stores the data received
 * @retval 0 if success, 1 if the function fails after TIMEOUT_THRESHOLD attempts
 */
uint8_t i2c_read_nak(I2C_TypeDef *I2Cx, uint8_t *store_here)
{
    /* We need to disable ack */
    I2Cx->CR1 &= (~I2C_CR1_ACK);

    /* FROM THE REFERENCE MANUAL:
     * 2. In order to generate the Stop/Restart condition,
     *    software must set the STOP/START bit after reading the second
     *    last data byte (after the second last RxNE event).
     * 3. In case a single byte has to be received, the Acknowledge disable
     *    is made during EV6 (before ADDR flag is cleared)
     *    and the STOP condition generation is made after EV6.
     *    : To handle this we simply send stop before receiving the last byte.
     */

    /* Generate stop */
    I2Cx->CR1 |= I2C_CR1_STOP;

    /* Wait until the i2c bus is ready to read */
    uint32_t time_out_counter = 0;

    while (check_i2c_status(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) == 0)
    {
        if (++time_out_counter == TIMEOUT_THRESHOLD)
        {
            return (uint8_t)1;
        }
    }

    *store_here = (uint8_t)I2Cx->DR;

    return 0;
}
