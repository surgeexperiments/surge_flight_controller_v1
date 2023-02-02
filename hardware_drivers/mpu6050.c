/**
 * @file MPU6050.c
 * @author Surge
 * @date 1/1/2019
 *
 * See the datasheet for instructions on using i2c with this device
 * The MPU6050 functions are bare-minimum atm. There are tons of stash that could b implemented and useful.
 * This hardware driver handles all i2c and DMA stuff for the MPU.
 *
 * IMPORTANT: Hardware
 * - The MPU6050 crashes if you have a pullup-resistor on the MCU @ the interrupt pin, use a pulldown!
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "../arm_drivers/timing_driver.h"
#include "../arm_drivers/dbg_swo_driver.h"
#include "../arm_drivers/timing_driver.h"
#include "../arm_drivers/exti_p.h"
#include "../arm_drivers/dma_driver.h"
#include "../arm_drivers/i2c_driver.h"
#include "../arm_drivers/gpio_driver.h"

#include "mpu6050.h"

/**
 * @brief main buffers for DMA
 */
uint8_t g_mpu6050_dma_rx_buffer[14];
uint8_t g_mpu6050_result_buffer[14];

/**
 * @brief Buffer for snprintf for debu
 */
char g_mpu6050_data_buffer_print[40];

/*******************************
 *        DMA FUNCTIONS        *
 *                             *
 ******************************/

/**
 * NOTE: Commented out because both the LSM303 and the MPU6050
 * requires the same interrupt handler. You can't have both names
 * active.
 * TODO: Fix this, it's clumsy however it works for prototyping.
 */

/* NOTE:
 * I2C1 uses DMA1, stream 0. This is the interrupt handler for that DMA stream.
 *

void DMA1_Stream0_IRQHandler(void)
  {
    //++dmaRanNumber;
    // Check if the DMA transfer is finished
  if (DMA1->LISR & DMA_LISR_TCIF0 )
    {

        //if((++dmaRanNumber%10000) == 0){
        //	sprintf(g_mpu6050_data_buffer_print, "DMA:%d\n", dmaRanNumber);		\
        //	print_str_to_dbg_port(g_mpu6050_data_buffer_print);
        //}

        // Disable i2cDMA
        I2C1->CR2 &= ~((uint32_t)I2C_CR2_DMAEN);

        // Gen stop condition
        I2C1->CR1 |= I2C_CR1_STOP;

        // Disable the DMA channel: This will force a transfer complete interrupt if enabled
        DMA1_Stream0->CR &= ~((uint32_t)(1<<4));  // Prevent triggering the transfer done interrupt
        DMA1_Stream0->CR &= ~((uint32_t)DMA_SxCR_EN);

        // We clear the DMA transfer complete flag in LISR by writing 1 to the complementary bit in LIFCR
        // (and-ing it isn't enough)
        DMA1->LIFCR = 0x3D; //(1<<5); //DMA_LIFCR_CTCIF0

        uint8_t i;
    for(i=0; i<14; i++) {
            // TODO: ADD A CONVERSION HERE!
            g_mpu6050_result_buffer[i] = g_mpu6050_dma_rx_buffer[i];
        }
    }
}
*/

/**
 * @brief Interrupt handler for GPIO based interrupt: MPU6050
 * 	 	  sets 5V when data is ready on this pin
 *
 * STATUS: Unused with current copnfiguration.
 */
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line12) != RESET)
    {
        mpu6050_dma_read();
        EXTI_ClearITPendingBit(EXTI_Line12);
    }
}

/************************************************************************************
 * I2C WRAPPER FUNCTIONS:
 *  These functions implements I2C exactly as the reference manual describes (incl the names)
 *  See the MPU6050 ref manual, page 33, for details
 *
 *************************************************************************************/

/**
 * @author SurgeExperiments
 *
 * @brief write a single byte to the MPU6050.
 * 		  A direct replica of the i2c comm function in the reference manual.
 *
 * @retval 0 if success, 1-4 if it fails
 */
static uint8_t single_byte_write_sequence(uint8_t internal_register, uint8_t command)
{

    /* S + (AD+W) + receive ACK, I2C_WRITE is 0 */
    if (i2c_start(I2C1, MPU6050_WRITE, 0) != 0)
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

/**
 * @author SurgeExperiments
 *
 * @brief write one or more bytes of data to the MPU6050.
 *
 * @param[in] internal_register the symbolic name of the register to write to.
 * 								(see the .h file)
 * @param[in] data_size We don't need more than a uint8_t as the MPU will
 * 						not require anywhere close to 255 bytes of data.
 *
 * @param[out] *data ptr to array where the return data will be saved.
 */
static void burst_write_sequence(uint8_t internal_register,
                                 uint8_t *data,
                                 uint8_t data_size)
{

    /* S + (AD+W) + receive ACK, I2C_WRITE is 0 */
    i2c_start(I2C1, MPU6050_WRITE, 0);

    /* RA + receive ACK */
    i2c_write_byte(I2C1, internal_register);

    /* DATA + receive ACK */
    while (data_size-- > 0)
    {
        i2c_write_byte(I2C1, *data++);
    }

    /* Send P (stop condition) */
    i2c_stop(I2C1);
}

/**
 * @author SurgeExperiments
 * @brief read single byte from the MPU6050
 * 	      See the reference manual for info about these steps (i2c chapter)
 *
 * TODO: RENAME.
 *
 * @param[in] internal_register to read from, see mpu6050.h for symbolic names (or check the reference manual)
 * @param[out] returnData data returned from the MPU stored here
 * @retval 0 if success, 1-4 if error (read the code for details)
 */
static uint8_t single_byte_read_sequence(uint8_t internal_register, uint8_t *return_data)
{

    /* S + (AD+W) + receive ACK, I2C_READ is 1 */
    if (i2c_start(I2C1, MPU6050_WRITE, 0) != 0)
    {
        return 1;
    }
    /* RA + receive ACK */
    if (i2c_write_byte(I2C1, internal_register) != 0)
    {
        return 2;
    }

    /* S + (AD+R) + receive ACK, I2C_READ is 1 */
    if (i2c_start(I2C1, MPU6050_READ, 1) != 0)
    {
        return 3;
    }

    /* Receive Data, send NACK (don't put an ack here, even if the MPU6050 wants it!) */
    if (i2c_read_nak(I2C1, return_data) != 0)
    {
        return 4;
    }

    /* Send P (stop condition): ignored for now, check the ref manual */
    return 0;
}

/**
 * @author SurgeExperiments
 * @brief as the name suggests: function that reads multiple bits from the MPU
 *
 * IMPORTANT: the data_buffer must contain at least sizeof(uint8_t)*number_of_items number of bytes.
 *
 *
 * @param[in] internal_register the register you want to read from. See mpu6050.h for symbolic names.
 * @param[out] dataBuffer ptr to array comtaining at least number_of_items items
 * @param[in] numberOfItems number of items to be read, will b put into data_buffer
 * @retval 0 if success, 1-5 if error (read the code for details)
 */
static uint8_t burst_read_sequence(uint8_t internal_register, uint8_t *data_buffer, uint8_t num_of_items)
{

    /* S + (AD+W) + receive ACK, I2C_READ is 1 */
    if (i2c_start(I2C1, MPU6050_WRITE, 0) != 0)
    {
        return 1;
    }

    /* RA + receive ACK */
    if (i2c_write_byte(I2C1, internal_register) != 0)
    {
        return 2;
    }

    /* S + (AD+R) + receive ACK, I2C_READ is 1 */
    if (i2c_start(I2C1, MPU6050_READ, 1) != 0)
    {
        return 3;
    }

    /* receive (number_of_items - 1) bytes and return an ACK */
    uint8_t item_number = 0;
    while (num_of_items-- > 1)
    {
        if (i2c_read_ack(I2C1, &data_buffer[item_number++]) != 0)
        {
            return 4;
        }
    }

    /* Receive final byte and return a NACK. This function handles the stop condition too
     * (verified with a logic analyzer + pulseview)
     */
    if (i2c_read_nak(I2C1, &data_buffer[item_number]) != 0)
    {
        return 5;
    }

    return 0;
}

/**
 * @author SurgeExperiments
 *
 * @brief init the MPU6050 for normal mode (no DMA) after I2C is ready.
 *
 * TODO: Make settings be args later? For now these are tried and true settings
 * 		 so adding args are not needed.
 */
static uint8_t setup_no_dma(void)
{
    /* Set all bits in PWR_MGMT_1 reg to 0 */
    if (single_byte_write_sequence(MPU6050_REG_PWR_MGMT_1, 0x00) != 0)
        return 1;

    /* Set gyro for 500dps in: GYRO_CONFIG register */
    if (single_byte_write_sequence(MPU6050_REG_GYRO_CONFIG, 0x08) != 0)
        return 2;
    /* Set accel for : +-8g (af_sel == 2) */
    if (single_byte_write_sequence(MPU6050_REG_ACCEL_CONFIG, 0x10) != 0)
        return 3;
    /* Set digital low-pass filter to 43hz */
    if (single_byte_write_sequence(MPU6050_REG_CONFIG, 0x03) != 0)
        return 4;

    /* Gyro startup-delay */
    tim10_delay_ms(200);

    return 0;
}

/**
 * @author SurgeExperiments
 * @brief set up the MPU6050 with the GPIO interrupt
 *
 * @retval 0 if success, 1-10 if error
 *
 * IMPORTANT:
 * - If you get these settings wrong the MPU6050 crashes spectacularly after a few seconds
 * - Requires I2C to be initialized before starting
 */
static uint8_t setup_with_interrupt(void)
{

    // Reset the MPU
    if (single_byte_write_sequence(MPU6050_REG_PWR_MGMT_1, 1U << 7) != 0)
        return 1;

    tim10_delay_ms(75);

    /*
    Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
    However, it is highly recommended that the device be configured to use one of the gyroscopes
    (or an external clock source) as the clock reference for improved stability.
    */

    if (single_byte_write_sequence(MPU6050_REG_PWR_MGMT_1, 0x3) != 0)
        return 1;

    if (single_byte_write_sequence(SMPLRT_DIV, 0x01) != 0)
        return 3;

    /* Set gyro for 500dps in: GYRO_CONFIG register */
    if (single_byte_write_sequence(MPU6050_REG_GYRO_CONFIG, 0x08) != 0)
        return 4;

    /* Set accel for : +-8g (af_sel == 2) */
    if (single_byte_write_sequence(MPU6050_REG_ACCEL_CONFIG, 0x10) != 0)
        return 5;

    /* Set digital low-pass filter to 43hz (check if this is effective!) */
    if (single_byte_write_sequence(MPU6050_REG_CONFIG, 0x03) != 0)
        return 6;

    /* reset gyro and accel sensor */
    if (single_byte_write_sequence(SIGNAL_PATH_RESET, 0x07) != 0)
        return 7;

    /*
    When set to 1, this bit enables the Data Ready interrupt, which occurs each time a write operation to
    all of the sensor registers has been completed.
    */

    if (single_byte_write_sequence(INT_PIN_CFG, 0x1 << 4) != 0)
        return 9;
    if (single_byte_write_sequence(INT_ENABLE, 0x1) != 0)
        return 10;

    /* Gyro startup-delay */
    tim10_delay_ms(200);

    return 0;
}

/**
 * @brief Sets up i2c and the MPU6050 in normal mode.
 *
 * @retval 0 if success 1 if fail
 */
uint8_t mpu_6050_init_normal(void)
{
    if (i2c_initialize(I2C1) != 0)
    {
        print_str_to_dbg_port("I2C1 init failed, exiting!\n");
        return 1;
    }

    if (setup_no_dma() != 0)
    {
        print_str_to_dbg_port("setup_MPU6050_quad() failed!\n");
        return 1;
    }

    return 0;
}

/**
 * @brief inits I2C with DMA, and then sets up the MPU6050
 *
 * IMPORTANT: also requires exti interrupt on PB12 (GPIO_driver.c)
 *
 * @retval 0 if success 1 if fail
 */
uint8_t mpu6050_init_dma(void)
{
    // Using the one global here
    dma_init_i2c(I2C1, g_mpu6050_dma_rx_buffer, 14);
    if (i2c_initialize(I2C1) != 0)
    {
        print_str_to_dbg_port("I2C1 init failed, exiting!\n");
        return 1;
    }

    if (setup_with_interrupt() != 0)
    {
        print_str_to_dbg_port("setup_MPU6050_quad_with_interrupt() failed!\n");
        return 1;
    }

    /* Enable the interrupt to run on PB12 */
    ConfigurePB12_extiRising();

    return 0;
}

/**
 * NOTE: this is a function found online. NOT tested.
 */
void mpu6050_init_for_madgewick(int Gscale, int Ascale)
{
    // wake up device-don't need this here if using calibration function below
    //  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
    //  delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

    // get stable time source
    single_byte_write_sequence(PWR_MGMT_1, 0x01); // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

    // Configure Gyro and Accelerometer
    // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
    // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
    // Maximum delay time is 4.9 ms corresponding to just over 200 Hz sample rate
    // TODO: Check if you can reduce this to get a 2ms maximum delay time!
    single_byte_write_sequence(CONFIG, 0x03);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    single_byte_write_sequence(SMPLRT_DIV, 0x04); // Use a 200 Hz rate; the same rate set in CONFIG above

    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    uint8_t c;
    single_byte_read_sequence(GYRO_CONFIG, &c);
    single_byte_write_sequence(GYRO_CONFIG, c & ~0xE0);       // Clear self-test bits [7:5]
    single_byte_write_sequence(GYRO_CONFIG, c & ~0x18);       // Clear AFS bits [4:3]
    single_byte_write_sequence(GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro

    // Set accelerometer configuration
    single_byte_read_sequence(ACCEL_CONFIG, &c);
    single_byte_write_sequence(ACCEL_CONFIG, c & ~0xE0);       // Clear self-test bits [7:5]
    single_byte_write_sequence(ACCEL_CONFIG, c & ~0x18);       // Clear AFS bits [4:3]
    single_byte_write_sequence(ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
    single_byte_write_sequence(INT_PIN_CFG, 0x22);
    single_byte_write_sequence(INT_ENABLE, 0x01); // Enable data ready (bit 0) interrupt
}

/*************************************************************************************
 * DIAGNOSTIC FUNCTIONS
 * Todo: Add self-test
 *************************************************************************************/

uint8_t mpu6050_get_device_id(void)
{
    uint8_t devId;
    if (single_byte_read_sequence(MPU6050_REG_WHOAMI, &devId) != 0)
        return 0;
    return devId;
}

/*************************************************************************************
 * DATA FETCHING FUNCTIONS
 *
 *************************************************************************************/

/**
 * @author SurgeExperiments
 */
static uint8_t fetch_gyro_xyz(uint8_t *data_buffer)
{
    if (burst_read_sequence(MPU6050_REG_GYRO, data_buffer, 6) != 0)
        return 1;
    return 0;
}

/**
 * @author SurgeExperiments
 * Get accel_XYZ, temp and gyro_XYZ data
 * Requires a 14 bits uint8_t array for data_buffer
 */
static uint8_t fetch_raw_imu_data_acc_tmp_gyro(uint8_t *data_buffer)
{
    if (burst_read_sequence(MPU6050_REG_ACCEL, data_buffer, 14) != 0)
        return 1;
    return 0;
}

/**
 * @author SurgeExperiments
 */
uint8_t mpu6050_gyro_raw_to_structs(uint8_t *mpu6050_raw_output, gyroData_st *gyro_output)
{

    if (fetch_gyro_xyz(mpu6050_raw_output) != 0)
        return 1;

    gyro_output->roll = (int16_t)(mpu6050_raw_output[0] << 8 | mpu6050_raw_output[1]);
    gyro_output->pitch = (int16_t)(mpu6050_raw_output[2] << 8 | mpu6050_raw_output[3]);
    gyro_output->yaw = (int16_t)(mpu6050_raw_output[4] << 8 | mpu6050_raw_output[5]);

    return 0;
}

/**
 * @author SurgeExperiments
 * @brief This doesn't do any axis reversal!
 */
uint8_t mpu6050_imu_raw_to_structs(uint8_t *mpu6050_raw_output,
                                   accelData_st *accel_output,
                                   temperatureData_st *temp_output,
                                   gyroData_st *gyro_output)
{
    if (fetch_raw_imu_data_acc_tmp_gyro(mpu6050_raw_output) != 0)
    {
        return 1;
    }

    accel_output->roll = (int16_t)(mpu6050_raw_output[0] << 8 | mpu6050_raw_output[1]);
    accel_output->pitch = (int16_t)(mpu6050_raw_output[2] << 8 | mpu6050_raw_output[3]);
    accel_output->z = (int16_t)(mpu6050_raw_output[4] << 8 | mpu6050_raw_output[5]);

    temp_output->temp = (int16_t)(mpu6050_raw_output[6] << 8 | mpu6050_raw_output[7]);

    gyro_output->roll = (int16_t)(mpu6050_raw_output[8] << 8 | mpu6050_raw_output[9]);
    gyro_output->pitch = (int16_t)(mpu6050_raw_output[10] << 8 | mpu6050_raw_output[11]);
    gyro_output->yaw = (int16_t)(mpu6050_raw_output[12] << 8 | mpu6050_raw_output[13]);

    return 0;
}

/*************************************************************************************
 * OFFSET FUNCTIONS
 *
 *************************************************************************************/
/**
 * @author SurgeExperiments
 */
void mpu6050_compute_gyro_offsets(gyroData_st *gyro_offsets)
{
    uint8_t mpu6050_raw_output[6];
    float gyro_roll = 0, gyro_pitch = 0, gyro_yaw = 0;
    int16_t _gyro_roll = 0, _gyro_pitch = 0, _gyro_yaw = 0;

    int counter;
    for (counter = 0; counter < 3000; counter++)
    {
        fetch_gyro_xyz(mpu6050_raw_output);
        _gyro_roll = (int16_t)(mpu6050_raw_output[0] << 8 | mpu6050_raw_output[1]);
        _gyro_pitch = (int16_t)(mpu6050_raw_output[2] << 8 | mpu6050_raw_output[3]);
        _gyro_yaw = (int16_t)(mpu6050_raw_output[4] << 8 | mpu6050_raw_output[5]);

        gyro_roll += _gyro_roll;
        gyro_pitch += _gyro_pitch;
        gyro_yaw += _gyro_yaw;
        tim10_delay_ms(3);
    }

    gyro_roll /= 3000;
    gyro_pitch /= 3000;
    gyro_yaw /= 3000;
    gyro_offsets->roll = gyro_roll;
    gyro_offsets->pitch = gyro_pitch;
    gyro_offsets->yaw = gyro_yaw;
}

/**
 * @author SurgeExperiments
 */
uint8_t mpu6050_compute_imu_offsets(gyroData_st *gyro_offsets, accelData_st *accel_offsets)
{

    uint8_t mpu6050_raw_output[14];

    /* Sanity check */
    gyro_offsets->pitch = 0;
    gyro_offsets->roll = 0;
    gyro_offsets->yaw = 0;
    accel_offsets->pitch = 0;
    accel_offsets->roll = 0;
    accel_offsets->z = 0;

    float accel_roll = 0, accel_pitch = 0, accel_z = 0, gyro_roll = 0, gyro_pitch = 0, gyro_yaw = 0;
    int16_t _accel_roll = 0, _accel_pitch = 0, _accel_z = 0, _gyro_roll = 0, _gyro_pitch = 0, _gyro_yaw = 0;
    int counter;
    for (counter = 0; counter < 3000; counter++)
    {
        if (fetch_raw_imu_data_acc_tmp_gyro(mpu6050_raw_output) != 0)
        {
            return 1;
        }

        _accel_roll = (int16_t)(mpu6050_raw_output[0] << 8 | mpu6050_raw_output[1]);
        _accel_pitch = (int16_t)(mpu6050_raw_output[2] << 8 | mpu6050_raw_output[3]);
        _accel_z = (int16_t)(mpu6050_raw_output[4] << 8 | mpu6050_raw_output[5]);

        _gyro_roll = (int16_t)(mpu6050_raw_output[8] << 8 | mpu6050_raw_output[9]);
        _gyro_pitch = (int16_t)(mpu6050_raw_output[10] << 8 | mpu6050_raw_output[11]);
        _gyro_yaw = (int16_t)(mpu6050_raw_output[12] << 8 | mpu6050_raw_output[13]);

        accel_roll += _accel_roll;
        accel_pitch += _accel_pitch;
        accel_z += _accel_z;

        gyro_roll += _gyro_roll;
        gyro_pitch += _gyro_pitch;
        gyro_yaw += _gyro_yaw;

        /* We wait just above the interrupt-limit */
        tim10_delay_ms(3);
    }

    gyro_roll /= 3000;
    gyro_pitch /= 3000;
    gyro_yaw /= 3000;
    gyro_offsets->roll = gyro_roll;
    gyro_offsets->pitch = gyro_pitch;
    gyro_offsets->yaw = gyro_yaw;

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
void mpu6050_compute_imu_offsets_dma(gyroData_st *gyro_offsets, accelData_st *accel_offsets)
{
    /* Sanity check */
    gyro_offsets->pitch = 0;
    gyro_offsets->roll = 0;
    gyro_offsets->yaw = 0;
    accel_offsets->pitch = 0;
    accel_offsets->roll = 0;
    accel_offsets->z = 0;

    float accel_roll = 0, accel_pitch = 0, accel_z = 0, gyro_roll = 0, gyro_pitch = 0, gyro_yaw = 0;
    int16_t _accel_roll = 0, _accel_pitch = 0, _accel_z = 0, _gyro_roll = 0, _gyro_pitch = 0, _gyro_yaw = 0;

    int counter;
    for (counter = 0; counter < 3000; counter++)
    {

        /* Hand packed for now: easy to get an overview */
        _accel_roll = (int16_t)(g_mpu6050_result_buffer[0] << 8 | g_mpu6050_result_buffer[1]);
        _accel_pitch = (int16_t)(g_mpu6050_result_buffer[2] << 8 | g_mpu6050_result_buffer[3]);
        _accel_z = (int16_t)(g_mpu6050_result_buffer[4] << 8 | g_mpu6050_result_buffer[5]);

        // temp_raw.temp = (g_mpu6050_result_buffer[6]<<8) | g_mpu6050_result_buffer[7];

        _gyro_roll = (int16_t)(g_mpu6050_result_buffer[8] << 8 | g_mpu6050_result_buffer[9]);
        _gyro_pitch = (int16_t)(g_mpu6050_result_buffer[10] << 8 | g_mpu6050_result_buffer[11]);
        _gyro_yaw = (int16_t)(g_mpu6050_result_buffer[12] << 8 | g_mpu6050_result_buffer[13]);

        /* Hand packed for now: easy to get an overview */
        accel_roll += _accel_roll;
        accel_pitch += _accel_pitch;
        accel_z += _accel_z;

        gyro_roll += _gyro_roll;
        gyro_pitch += _gyro_pitch;
        gyro_yaw += _gyro_yaw;

        /*  We wait just above the interrupt-limit */
        tim10_delay_ms(3);
    }

    gyro_roll /= 3000;
    gyro_pitch /= 3000;
    gyro_yaw /= 3000;
    gyro_offsets->roll = gyro_roll;
    gyro_offsets->pitch = gyro_pitch;
    gyro_offsets->yaw = gyro_yaw;

    accel_offsets->roll = (accel_roll / 3000.0f);
    accel_offsets->pitch = (accel_pitch / 3000.0f);
    accel_offsets->z = (accel_z / 3000.0f);

    /* Subtract the gravity vector value for the accelerometer (+-8g accel setting) */
    accel_offsets->z -= 4096.0f;
}

/**
 * NOTE: found online. NOT tested properly.
 */
void mpu6050_calibrate_for_madgewick(float *dest1, float *dest2)
{
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    // reset device, reset all registers, clear gyro and accelerometer bias registers
    single_byte_write_sequence(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    tim10_delay_ms(100);

    // get stable time source
    // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    single_byte_write_sequence(PWR_MGMT_1, 0x01);
    single_byte_write_sequence(PWR_MGMT_2, 0x00);
    tim10_delay_ms(200);

    // Configure device for bias calculation
    single_byte_write_sequence(INT_ENABLE, 0x00);   // Disable all interrupts
    single_byte_write_sequence(FIFO_EN, 0x00);      // Disable FIFO
    single_byte_write_sequence(PWR_MGMT_1, 0x00);   // Turn on internal clock source
    single_byte_write_sequence(I2C_MST_CTRL, 0x00); // Disable I2C master
    single_byte_write_sequence(USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    single_byte_write_sequence(USER_CTRL, 0x0C);    // Reset FIFO and DMP
    tim10_delay_ms(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    single_byte_write_sequence(CONFIG, 0x01);     // Set low-pass filter to 188 Hz
    single_byte_write_sequence(SMPLRT_DIV, 0x00); // Set sample rate to 1 kHz
    // single_byte_write_sequence(GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    single_byte_write_sequence(GYRO_CONFIG, 0x00);
    single_byte_write_sequence(ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    uint16_t gyrosensitivity = 131;    // = 131 LSB/degrees/sec
    uint16_t accelsensitivity = 16384; // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    single_byte_write_sequence(USER_CTRL, 0x40); // Enable FIFO
    single_byte_write_sequence(FIFO_EN, 0x78);   // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
    tim10_delay_ms(80);                          // accumulate 80 samples in 80 milliseconds = 960 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    single_byte_write_sequence(FIFO_EN, 0x00); // Disable gyro and accelerometer sensors for FIFO
    burst_read_sequence(FIFO_COUNTH, data, 2);
    // readBytes(MPU6050_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++)
    {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        burst_read_sequence(FIFO_R_W, data, 12);
        // readBytes(MPU6050_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
        accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]); // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
        accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
        gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
        gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
        gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

        accel_bias[0] += (int32_t)accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t)accel_temp[1];
        accel_bias[2] += (int32_t)accel_temp[2];
        gyro_bias[0] += (int32_t)gyro_temp[0];
        gyro_bias[1] += (int32_t)gyro_temp[1];
        gyro_bias[2] += (int32_t)gyro_temp[2];
    }
    accel_bias[0] /= (int32_t)packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t)packet_count;
    accel_bias[2] /= (int32_t)packet_count;
    gyro_bias[0] /= (int32_t)packet_count;
    gyro_bias[1] /= (int32_t)packet_count;
    gyro_bias[2] /= (int32_t)packet_count;

    if (accel_bias[2] > 0L)
    {
        accel_bias[2] -= (int32_t)accelsensitivity;
    } // Remove gravity from the z-axis accelerometer bias calculation
    else
    {
        accel_bias[2] += (int32_t)accelsensitivity;
    }

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0] / 4) & 0xFF;      // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
    data[3] = (-gyro_bias[1] / 4) & 0xFF;
    data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
    data[5] = (-gyro_bias[2] / 4) & 0xFF;

    // Push gyro biases to hardware registers
    single_byte_write_sequence(XG_OFFS_USRH, data[0]); // might not be supported in MPU6050
    single_byte_write_sequence(XG_OFFS_USRL, data[1]);
    single_byte_write_sequence(YG_OFFS_USRH, data[2]);
    single_byte_write_sequence(YG_OFFS_USRL, data[3]);
    single_byte_write_sequence(ZG_OFFS_USRH, data[4]);
    single_byte_write_sequence(ZG_OFFS_USRL, data[5]);

    dest1[0] = (float)gyro_bias[0] / (float)gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
    dest1[1] = (float)gyro_bias[1] / (float)gyrosensitivity;
    dest1[2] = (float)gyro_bias[2] / (float)gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    burst_read_sequence(XA_OFFSET_H, data, 2);
    // readBytes(MPU6050_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int16_t)((int16_t)data[0] << 8) | data[1];
    burst_read_sequence(YA_OFFSET_H, data, 2);
    // readBytes(MPU6050_ADDRESS, YA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[1] = (int16_t)((int16_t)data[0] << 8) | data[1];
    burst_read_sequence(ZA_OFFSET_H, data, 2);
    // readBytes(MPU6050_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[2] = (int16_t)((int16_t)data[0] << 8) | data[1];

    uint32_t mask = 1uL;             // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

    for (ii = 0; ii < 3; ii++)
    {
        if (accel_bias_reg[ii] & mask)
            mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1] / 8);
    accel_bias_reg[2] -= (accel_bias[2] / 8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0]) & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1]) & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2]) & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

    // Push accelerometer biases to hardware registers
    single_byte_write_sequence(XA_OFFSET_H, data[0]); // might not be supported in MPU6050
    single_byte_write_sequence(XA_OFFSET_L_TC, data[1]);
    single_byte_write_sequence(YA_OFFSET_H, data[2]);
    single_byte_write_sequence(YA_OFFSET_L_TC, data[3]);
    single_byte_write_sequence(ZA_OFFSET_H, data[4]);
    single_byte_write_sequence(ZA_OFFSET_L_TC, data[5]);

    // Output scaled accelerometer biases for manual subtraction in the main program
    dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
    dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
    dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;
}

void mpu6050_set_accel_offsets(accelData_st *accel_offsets,
                               float accel_roll_offset,
                               float accel_pitch_offset,
                               float accel_Z_offset)
{
    accel_offsets->roll = accel_roll_offset;
    accel_offsets->pitch = accel_pitch_offset;
    accel_offsets->z = accel_Z_offset;
}

/**
 * @author SurgeExperiments
 *
 * @brief read 14 bytes (3x gyro, temp, 3x accel) from the MPU6050.
 *	  TODO: turn this into a define?
 *
 * IMPORTANT:
 * - for now this works for I2C1 (but it can easily be extended to work on all 3 I2C bus).
 * - Since the MPU requires about 480 microseconds for a full i2c transfer, you can't call
 *   this function more often than that unless you want to really screw this up.
 *   : Given normal update-rates (and the flight loop speed) once every 2ms is a good rate.
 * - On average this function (and the interrupt triggering after the DMA is finished)
 *   requires about 17 microseconds of active CPU time, compared to 480 when not using the DMA.
 *   : Given a 2000 microseconds flight loop that is saving about 26% of the flight loop time, which
 *     can be used for other computations, a significant saving of cycles.
 * - You can initiate this function in several ways.
 *   : The MPU has a function where it will send 5V to an
 *     output pin when new data is ready (which you can couple to a GPIO interrupt that triggers the DMA read).
 *     The downside of this is that this requires using an additional GPIO pin.
 *   : A simpler alternative is to initiate this function with a regular timer interrupt. The downside of this
 *     is that you're occupying a timer.
 *
 * @retval global array in i2c_driver.c that stores the bytes fetched from the IMU.
 */
void mpu6050_dma_read(void)
{
    if (DMA1->LISR & DMA_LISR_TCIF0)
    {
        /*
         * If this test is true you've seriously fucked up!
         * This event needs to be logged immediately or you'll
         * have a series of mysterious crashes on your hands.
         *
         * Since the MPU6050 requires about 480 microseconds to
         * transfer all the data via i2c (DMA or not) this should NEVER
         * happen if you init a DMA read at a "safe" interval like every
         * 2ms unless you've got a HW error on your hands or you totally
         * fucked up your interval for calling or your interrupts.
         */
    }

    /* We need to disable the DMA channel first */
    DMA1_Stream0->CR &= ~((uint32_t)DMA_SxCR_EN);

    /* Set the DMA counter to 14
     * TODO: symbol size for sizeof(data) to fetch?
     */
    DMA1_Stream0->NDTR = (uint16_t)14;

    /* Datasheet: This bit is used in master receiver mode to permit the generation of a NACK on the last received data. */
    I2C1->CR2 |= I2C_CR2_LAST;

    /* S + (AD+W) + receive ACK, I2C_READ is 1 */
    i2c_start(I2C1, MPU6050_WRITE, 0);

    /* RA + receive ACK */
    i2c_write_byte(I2C1, MPU6050_REG_ACCEL);

    /* S + (AD+R) + receive ACK, I2C_READ is 1 */
    i2c_start(I2C1, MPU6050_READ, 1);

    /* Now the readAck/readNack is ready, which the DMA controller is configured to take over! */

    /* Start DMA to receive data from I2C */
    DMA1_Stream0->CR |= (1 << 4); // Enable DMA interrupts Prevent triggering the transfer done interrupt
    DMA1_Stream0->CR |= (uint32_t)DMA_SxCR_EN;

    /* Enable the I2C dma requests */
    I2C1->CR2 |= I2C_CR2_DMAEN;

    /* The DMA is set up to run an interrupt upon completion.
     * This interrupt is void DMA1_Stream0_IRQHandler(void) in i2c_driver.c
     * The data fetched can be read in the global arrays in i2c_driver.c
     */
}

/**
 * @author SurgeExperiments
 *
 * @brief Function to read the current value of the IMU when using DMA read.
 *
 * When DMA is running it updates the array uint8_t homeBuffer[14], so you can simply read that
 */
void mpu6050_imu_raw_to_structs_dma(accelData_st *accel_output, temperatureData_st *temp_output, gyroData_st *gyro_output)
{
    /* Hand packed for now: easy to get an overview */
    accel_output->roll = (int16_t)(g_mpu6050_result_buffer[0] << 8 | g_mpu6050_result_buffer[1]);
    accel_output->pitch = (int16_t)(g_mpu6050_result_buffer[2] << 8 | g_mpu6050_result_buffer[3]);
    accel_output->z = (int16_t)(g_mpu6050_result_buffer[4] << 8 | g_mpu6050_result_buffer[5]);

    temp_output->temp = (int16_t)(g_mpu6050_result_buffer[6] << 8 | g_mpu6050_result_buffer[7]);

    gyro_output->roll = (int16_t)(g_mpu6050_result_buffer[8] << 8 | g_mpu6050_result_buffer[9]);
    gyro_output->pitch = (int16_t)(g_mpu6050_result_buffer[10] << 8 | g_mpu6050_result_buffer[11]);
    gyro_output->yaw = (int16_t)(g_mpu6050_result_buffer[12] << 8 | g_mpu6050_result_buffer[13]);
}

/*************************************************************************************
 * DATA CONVERSION FUNCTIONS
 *************************************************************************************/

static void gyro_raw_to_deg_per_second(const gyroData_st gyro_input, gyroData_st *gyro_output)
{
    /* Scale gyro data to deg/seconds [now you can use USART_send on them] with FS_SEL == 1 */
    gyro_output->roll = gyro_input.roll / (float)65.5;
    gyro_output->pitch = gyro_input.pitch / (float)65.5;
    gyro_output->yaw = gyro_input.yaw / (float)65.5;
}

static void temp_raw_to_celcius(const temperatureData_st temp_input, temperatureData_st *temp_output)
{
    /* datasheet: Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53 */
    temp_output->temp = temp_input.temp / (float)340 + (float)36.53;
}

/**
 * @brief avoid this crap, doesn't scale based on how the accel is set up so it's often bound to
 * 		  be wrong.
 */
void accel_raw_to_g_per_second(const accelData_st accel_input, accelData_st *accel_output)
{
    /* Scale accel to g/s with AF_SEL==2*/
    accel_output->roll = accel_input.roll / 4096.0f;
    accel_output->pitch = accel_input.pitch / 4096.0f;
    accel_output->z = accel_input.z / 4096.0f;
}
