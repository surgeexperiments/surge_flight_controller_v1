/**
 * @file lsm303dlhc
 * @author SurgeExperiments
 */

#include "lsm303dlhc.h"
#include "../arm_drivers/gpio_driver.h"
#include "../arm_drivers/exti_p.h"
#include "../arm_drivers/dma_driver.h"
#include "../arm_drivers/dbg_swo_driver.h"
#include "../arm_drivers/timing_driver.h"

/*
 * I2C1 uses DMA1, stream 0. This is the interrupt handler for that DMA stream.
 * Interrupt pin on PE2
 */
uint8_t g_lsm303dlhc_dma_rx_buffer[6];
uint8_t g_lsm303dlhc_home_buffer[6];

/* This interrupt runs 1344 times per second, which is the update rate the LSM303 is configured to */
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
    lsm303_dma_read();

    /* clear the interrupt flag */
    if (TIM11->SR & TIM_SR_UIF)
    {
        TIM11->SR &= ~TIM_SR_UIF;
    }
}

/**
 * @author SurgeExperiments
 * I2C1 uses DMA1, stream 0. This is the interrupt handler for that DMA stream.
 */
void DMA1_Stream0_IRQHandler(void)
{

    if (DMA1->LISR & DMA_LISR_TCIF0)
    {

        /* Disable i2cDMA */
        I2C1->CR2 &= ~((uint32_t)I2C_CR2_DMAEN);
        I2C1->CR1 |= I2C_CR1_STOP;

        /* Disable the DMA channel: This will force a transfer complete interrupt if enabled */
        /* Prevent triggering the transfer done interrupt */
        DMA1_Stream0->CR &= ~((uint32_t)(1 << 4));
        DMA1_Stream0->CR &= ~((uint32_t)DMA_SxCR_EN);

        /**
         * We clear the DMA transfer complete flag in LISR by writing 1
         * to the complementary bit in LIFCR
         * (and-ing it isn't enough)
         */
        DMA1->LIFCR = 0x3D;

        uint8_t i;
        for (i = 0; i < 6; i++)
        {
            g_lsm303dlhc_home_buffer[i] = g_lsm303dlhc_dma_rx_buffer[i];
        }
    }
}

void EXTI2_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line2) != RESET)
    {
        lsm303_dma_read();
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}

/**
 * @author SurgeExperiments
 */
static void lsm303_i2c_init(void)
{
    I2C_InitTypeDef myStruct;
    /* For now it's hard-coded just to get things working */
    myStruct.I2C_ClockSpeed = 400000;
    myStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    myStruct.I2C_Mode = I2C_Mode_I2C;
    myStruct.I2C_OwnAddress1 = 0x00;
    myStruct.I2C_Ack = I2C_Ack_Disable;
    myStruct.I2C_DutyCycle = I2C_DutyCycle_2;

    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    gpio_init_clock(GPIOB);
    gpio_init_pins_alt_func(GPIOB, 6, GPIO_AF_I2C1, GPIO_SPEED_MEDIUM, GPIO_TYPE_OPEN_DRAIN, GPIO_PULLUP);
    gpio_init_pins_alt_func(GPIOB, 9, GPIO_AF_I2C1, GPIO_SPEED_MEDIUM, GPIO_TYPE_OPEN_DRAIN, GPIO_PULLUP);

    /* Disable i2c before initialization */
    I2C1->CR1 &= ~I2C_CR1_PE;

    I2C_InitTypeDef I2C_InitTypeDefInstance;
    i2c_load_init_struct_fc(&I2C_InitTypeDefInstance);

    i2c_setupRegisters(I2C1, &myStruct);

    /* Re-enable i2c */
    I2C1->CR1 |= I2C_CR1_PE;
}

/**
 * @author SurgeExperiments
 */
static uint8_t lsm303_init(void)
{
    /* Set normal mode, speed 1.344khz */
    if (lsm303_single_byte_write_sequence(LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x97) != 0)
        return 1;
    /* Init to +-8g */
    if (lsm303_single_byte_write_sequence(LSM303_REGISTER_ACCEL_CTRL_REG4_A, (1U << 5) | (1U << 3)) != 0)
        return 1;
    return 0;
}

/**
 * @author SurgeExperiments
 */
static uint8_t lsm303_init_dma(void)
{

    if (lsm303_single_byte_write_sequence(LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x97) != 0)
        return 1;

    /* Set interrupt1 on dataReady1 */
    if (lsm303_single_byte_write_sequence(LSM303_REGISTER_ACCEL_CTRL_REG3_A, (1U << 4)) != 0)
        return 1;

    /* Init to +-8g */
    if (lsm303_single_byte_write_sequence(LSM303_REGISTER_ACCEL_CTRL_REG4_A, (1U << 5)) != 0)
        return 1;

    /* Latch interrupt request on INT1_SRC register, with INT1_SRC register cleared by reading INT1_SRC itself */
    if (lsm303_single_byte_write_sequence(LSM303_REGISTER_ACCEL_CTRL_REG5_A, (1U << 2)) != 0)
        return 1;

    return 0;
}

/*************************************************************************************
 * COMPLETE SETUP FUNCTIONS (i2c, calibration ect)
 *
 *************************************************************************************/

uint8_t lsm303_init_normal(void)
{
    lsm303_i2c_init();
    if (lsm303_init() != 0)
        return 1;
    return 0;
}

uint8_t lsm303_init_dma_with_interrupt(void)
{
    ConfigurePE2_extiRising();
    dma_init_i2c(I2C1, g_lsm303dlhc_dma_rx_buffer, 6);
    lsm303_i2c_init();
    if (lsm303_init_dma() != 0)
        return 1;
    return 0;
}

uint8_t lsm303_init_dma_no_int_pin(void)
{
    dma_init_i2c(I2C1, g_lsm303dlhc_dma_rx_buffer, 6);
    lsm303_i2c_init();
    if (lsm303_init() != 0)
        return 1;
    return 0;
}

uint8_t lsm303_init_dma_with_timing(void)
{
    /* The thing runs on 1.344 khz, so we interrupt and run the DMA all those times */
    dma_init_i2c(I2C1, g_lsm303dlhc_dma_rx_buffer, 6);
    lsm303_i2c_init();
    if (lsm303_init() != 0)
        return 1;
    lsm303_setup_tim11_int_timing();
    return 0;
}

/*************************************************************************************
 * INITIALIZATION FUNCTIONS:
 *
 *************************************************************************************/

/**
 * @author SurgeExperiments
 */
void lsm303_setup_tim11_int_timing(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;

    /* on another bus than TIM2, needs a different prescaler */
    TIM11->PSC = 99;
    TIM11->ARR = 743;
    TIM11->DIER = (1U << 0);
    /* bit 2 urs: 1: Only counter overflow generates an update interrupt if enabled. */
    TIM11->CR1 |= (1U << 2);
    TIM11->CR1 |= TIM_CR1_CEN;

    /**
     * Set the prescaler and arr to generate an overflow event every 1ms (or 0.5ms if needed)
     * This doesn't conflict with TIM1 PWM-read as it only is the capture event, the Tim1_CC handler
     */
    NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
}

uint8_t lsm303_clear_int1(void)
{
    uint8_t myData;
    if (lsm303_byte_read_sequence(LSM303_REGISTER_ACCEL_INT1_SOURCE_A, &myData) != 0)
        return 1;
    return 0;
}

/*************************************************************************************
 * OFFSET FUNCTIONS
 *
 *************************************************************************************/

uint8_t lsm303_dlhc_compute_accel_offsets(accelData_st *accel_offsets)
{
    uint8_t raw_output[6];

    /* Sanity check */
    accel_offsets->pitch = 0;
    accel_offsets->roll = 0;
    accel_offsets->z = 0;

    accelData_st accelRaw;

    float accel_roll = 0, accel_pitch = 0, accel_z = 0;

    int counter;
    for (counter = 0; counter < 3000; counter++)
    {
        if (lsm303_imu_raw_to_structs(raw_output, &accelRaw) != 0)
        {
            return 1;
        }

        /* Hand packed for now: easy to get an overview */
        accel_roll += accelRaw.roll;
        accel_pitch += accelRaw.pitch;
        accel_z += accelRaw.z;

        /* We wait just above the interrupt-limit */
        tim10_delay_ms(3);
    }

    accel_offsets->roll = (accel_roll / 3000.0f);
    accel_offsets->pitch = (accel_pitch / 3000.0f);
    accel_offsets->z = (accel_z / 3000.0f);

    // Subtract the gravity vector value for the accelerometer (+-8g accel setting)
    accel_offsets->z -= 4096.0f;
    return 0;
}

/**
 * @author SurgeExperiments
 */
uint8_t lsm303_compute_accel_offsets_dma_no_int(accelData_st *accel_offsets)
{

    /* Sanity check */
    accel_offsets->pitch = 0;
    accel_offsets->roll = 0;
    accel_offsets->z = 0;

    accelData_st accel_raw;

    float accel_roll = 0, accel_pitch = 0, accel_z = 0;

    int counter;
    for (counter = 0; counter < 3000; counter++)
    {
        lsm303_dma_read();
        lsm303_fix_bytes(g_lsm303dlhc_home_buffer, &accel_raw);

        /* Hand packed for now: easy to get an overview */
        accel_roll += accel_raw.roll;
        accel_pitch += accel_raw.pitch;
        accel_z += accel_raw.z;

        tim10_delay_ms(3);
    }

    accel_offsets->roll = (accel_roll / 3000.0f);
    accel_offsets->pitch = (accel_pitch / 3000.0f);
    accel_offsets->z = (accel_z / 3000.0f);

    /* Subtract the gravity vector value for the accelerometer (+-8g accel setting) */
    accel_offsets->z -= 4096.0f;
    return 0;
}

/**
 * @author SurgeExperiments
 */
uint8_t lsm303_compute_accel_offsets_dma_with_int(accelData_st *accel_offsets)
{

    /* Sanity check */
    accel_offsets->pitch = 0;
    accel_offsets->roll = 0;
    accel_offsets->z = 0;

    accelData_st accel_raw;

    float accel_roll = 0, accel_pitch = 0, accel_z = 0;

    int counter;
    for (counter = 0; counter < 3000; counter++)
    {
        lsm303_imu_raw_to_structs_dma(&accel_raw);

        /* Hand packed for now: easy to get an overview */
        accel_roll += accel_raw.roll;
        accel_pitch += accel_raw.pitch;
        accel_z += accel_raw.z;

        tim10_delay_ms(3);
    }

    accel_offsets->roll = (accel_roll / 3000.0f);
    accel_offsets->pitch = (accel_pitch / 3000.0f);
    accel_offsets->z = (accel_z / 3000.0f);

    /* Subtract the gravity vector value for the accelerometer (+-8g accel setting) */
    accel_offsets->z -= 4096.0f;
    return 0;
}

uint8_t lsm303_verify_accel_offset_integrity(accelData_st accel_offsets)
{
    /* If accel offsets have values: 0, 0, -4096 the accelerometer has failed  */
    if ((accel_offsets.pitch == 0) && (accel_offsets.roll == 0) && (accel_offsets.z == -4096))
    {
        return 1;
    }

    return 0;
}

/*************************************************************************************
 * DATA FETCHING FUNCTIONS
 *
 *************************************************************************************/

void lsm303_dma_read(void)
{

    if (DMA1->LISR & DMA_LISR_TCIF0)
    {
    }

    /* We need to disable the DMA channel first */
    DMA1_Stream0->CR &= ~((uint32_t)DMA_SxCR_EN);

    DMA1_Stream0->NDTR = (uint16_t)6;

    /* This bit is used in master receiver mode to permit the generation of a
     * NACK on the last received data.
     */
    I2C1->CR2 |= I2C_CR2_LAST;

    /* S + (AD+W) + receive ACK, I2C_READ is 1 */
    i2c_start(I2C1, LSM303_ADDRESS_ACCEL_WRITE, 0);

    /*
     * In order to read multiple bytes, it is necessary to assert the most significant bit of the sub- address field.
     * In other words, SUB(7) must be equal to 1 while SUB(6-0) represents the address of the first register to be read.
     * ie: LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80, NOT LSM303_REGISTER_ACCEL_OUT_X_L_A
     */
    i2c_write_byte(I2C1, (uint8_t)LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80);

    /*  S + (AD+R) + receive ACK, I2C_READ is 1 */
    i2c_start(I2C1, LSM303_ADDRESS_ACCEL_READ, 1);

    /* Now the readAck/readNack is ready, which the DMA controller is configured to take over! */

    /* Start DMA to receive data from I2C */
    DMA1_Stream0->CR |= (1 << 4); // Enable DMA interrupts Prevent triggering the transfer done interrupt
    DMA1_Stream0->CR |= (uint32_t)DMA_SxCR_EN;

    /*  Enable the I2C dma requests */
    I2C1->CR2 |= I2C_CR2_DMAEN;

    /* The DMA is set up to run an interrupt upon completion. This interrupt
     * is void DMA1_Stream0_IRQHandler(void) in i2c_driver.c
     * The results can be read in the arrays in i2c_driver.c
     */
}

uint8_t lsm303_data_read(uint8_t *data_buffer, uint8_t number_of_items)
{
    /* S + (AD+W) + receive ACK, I2C_READ is 1 */
    if (i2c_start(I2C1, LSM303_ADDRESS_ACCEL_WRITE, 0) != 0)
    {
        return 1;
    }

    /*
    In order to read multiple bytes, it is necessary to assert the most significant bit of the sub- address field.
    In other words, SUB(7) must be equal to 1 while SUB(6-0) represents the address of the first register to be read.
    ie: LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80, NOT LSM303_REGISTER_ACCEL_OUT_X_L_A
    */
    if (i2c_write_byte(I2C1, (uint8_t)LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80) != 0)
    { //
        return 2;
    }

    /* S + (AD+R) + receive ACK, I2C_READ is 1 */
    if (i2c_start(I2C1, LSM303_ADDRESS_ACCEL_READ, 1) != 0)
    {
        return 3;
    }

    /* receive (number_of_items - 1) bytes and return an ACK */
    uint8_t itemNumber = 0;
    while (number_of_items-- > 1)
    {
        if (i2c_read_ack(I2C1, &data_buffer[itemNumber++]) != 0)
        {
            return 4;
        }
    }

    /*
     * Receive final byte and return a NACK. This function handles the stop condition too
     * (verified with a logic analyzer + pulseview)
     */
    if (i2c_read_nak(I2C1, &data_buffer[itemNumber]) != 0)
    {
        return 5;
    }

    return 0;
}

uint8_t lsm303_imu_raw_to_structs(uint8_t *data_buffer, accelData_st *accel_raw)
{
    if (lsm303_data_read(data_buffer, 6) != 0)
        return 1;
    lsm303_fix_bytes(data_buffer, accel_raw);
    return 0;
}

/* With DMA with interrupt: this is your reading function */
void lsm303_imu_raw_to_structs_dma(accelData_st *accel_raw)
{
    accel_raw->roll = (int16_t)(g_lsm303dlhc_home_buffer[0] | (g_lsm303dlhc_home_buffer[1] << 8));
    accel_raw->pitch = (int16_t)(g_lsm303dlhc_home_buffer[2] | (g_lsm303dlhc_home_buffer[3] << 8));
    accel_raw->z = (int16_t)(g_lsm303dlhc_home_buffer[4] | (g_lsm303dlhc_home_buffer[5] << 8));
}

void lsm303_fix_bytes(uint8_t *data_buffer, accelData_st *my_data)
{
    /* low byte first */
    my_data->roll = (int16_t)(data_buffer[0] | (data_buffer[1] << 8));
    my_data->pitch = (int16_t)(data_buffer[2] | (data_buffer[3] << 8));
    my_data->z = (int16_t)(data_buffer[4] | (data_buffer[5] << 8));
}

/************************************************************************************
 * I2C WRAPPER FUNCTIONS:
 *  These functions implements I2C exactly as the datasheet describes (incl the names)
 *  See the MPU6050 datasheet, page 33, for details
 *
 *************************************************************************************/

uint8_t lsm303_single_byte_write_sequence(uint8_t internal_register, uint8_t command)
{
    /* S + (AD+W) + receive ACK, I2C_WRITE is 0 */
    if (i2c_start(I2C1, LSM303_ADDRESS_ACCEL_WRITE, 0) != 0)
        return 1;

    /* RA + receive ACK,  i2c_write ret error if ack not received */
    if (i2c_write_byte(I2C1, internal_register) != 0)
        return 2;

    /* DATA + receive ACK, ev check for error (ack not received) */
    if (i2c_write_byte(I2C1, command) != 0)
        return 3;

    /* Send P (stop condition) */
    if (i2c_stop(I2C1) != 0)
        return 4;

    return 0;
}

uint8_t lsm303_byte_read_sequence(uint8_t internal_register, uint8_t *returnData)
{
    /* S + (AD+W) + receive ACK, I2C_READ is 1 */
    if (i2c_start(I2C1, LSM303_ADDRESS_ACCEL_WRITE, 0) != 0)
    {
        return 1;
    }
    /* RA + receive ACK */
    if (i2c_write_byte(I2C1, internal_register) != 0)
    {
        return 2;
    }

    /* S + (AD+R) + receive ACK, I2C_READ is 1 */
    if (i2c_start(I2C1, LSM303_ADDRESS_ACCEL_READ, 1) != 0)
    {
        return 3;
    }

    /* Receive Data, send NACK (don't put an ack here, even if the MPU6050 wants it!) */
    if (i2c_read_nak(I2C1, returnData) != 0)
    {
        return 4;
    }

    return 0;
}
