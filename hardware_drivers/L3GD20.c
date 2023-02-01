#include "L3GD20.h"
#include "../arm_drivers/timing_driver.h"
#include "../arm_drivers/dbg_swo_driver.h"
#include "../arm_drivers/gpio_driver.h"

/* TODO
- Add error handling
*/

/*
- Max clock frequency: 10 mHz
- SPI mode: slave
- The serial interface interacts with the outside world through 4 wires: CS, SPC, SDI and SDO
- CS is the Serial Port Enable and is controlled by the SPI master. It goes low at the start
  of the transmission and goes back high at the end. SPC is the Serial Port Clock and
    it is controlled by the SPI master. It is stopped high when CS is high (no transmission).
    SDI and SDO are respectively the Serial Port Data Input and Output.
    Those lines are driven at the falling edge of SPC and should be captured at the rising edge of SPC.
    Both the Read Register and Write Register commands are completed in
    16 clock pulses or in multiples of 8 in case of multiple bytes read/write.
    Bit duration is the time between two falling edges of SPC. The first bit (bit 0) starts at the
    first falling edge of SPC after the falling edge of CS while the last bit (bit 15, bit 23, ...)
    starts at the last falling edge of SPC just before the rising edge of CS.

- bit 0: RW bit. When 0, the data DI(7:0) is written to the device. When 1, the data DO(7:0) from
  the device is read. In the latter case, the chip will drive SDO at the
  start of bit 8. bit 1: MS bit. When 0, the address remains unchanged in multiple read/write commands.
  When 1, the address will be auto-incremented in multiple read/write
  commands. bit 2-7: address AD(5:0). This is the address field of the indexed register. bit 8-15: data
  DI(7:0) (write mode). This is the data that will be written to the device
  (MSb first). bit 8-15: data DO(7:0) (read mode). This is the data that will be
  read from the device (MSb first). In multiple read/write commands, further blocks of 8 clock periods
  will be added. When the MS bit is 0, the address used to read/write data remains
  the same for every block. When the MS bit is 1, the address used to read/write data is incremented
  at every block. The function and the behavior of SDI and SDO remain unchanged.

- The SPI read command is performed with 16 clock pulses. The multiple byte read command is
  performed by adding blocks of 8 clock pulses to the previous one.
  bit 0: READ bit. The value is 1. bit 1: MS bit. When 0 do not increment address;
  when 1 increment address in multiple reading.

  bit 2-7: address AD(5:0). This is the address field of the indexed register.
  bit 8-15: data DO(7:0) (read mode). This is the data that will be read from the device (MSb first).
  bit 16-... : data DO(...-8). Further data in multiple byte reading.

- The SPI Write command is performed with 16 clock pulses.
  The multiple byte write command is performed by adding blocks of 8 clock pulses to the previous one.
  bit 0: WRITE bit. The value is 0. bit 1: MS bit. When 0, do not increment address;
  when 1, increment address in multiple writing.

  bit 2 -7: address AD(5:0). This is the address field of the indexed register.
  bit 8-15: data DI(7:0) (write mode). This is the data that will be written to the device (MSb first).
  bit 16-... : data DI(...-8). Further data in multiple byte writing.

*/

static uint8_t l3gd20_fetch_data(uint8_t address)
{
    uint8_t data;

    SET_CS_LOW;

    /* Send address with read command */
    spi_send_u8(SPI1, address | 0x80);

    /* Read data */
    data = spi_send_u8(SPI1, 0xFF);

    SET_CS_HIGH;

    return data;
}

static uint8_t l3gd20_write_data(uint8_t address, uint8_t data)
{
    SET_CS_LOW;

    /* Send address with read command */
    spi_send_u8(SPI1, address);

    /* Read data */
    spi_send_u8(SPI1, data);

    SET_CS_HIGH;

    return data;
}

static void init_cs_pin(void)
{
    gpio_init_clock(GPIOE);
    gpio_init_pins_io(GPIOE, 3, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_PULLUP, GPIO_SPEED_LOW);

    SET_CS_HIGH;
}

uint8_t l3gd20_init(void)
{
    spi_init_lgd20_devboard(SPI1, 0);
    init_cs_pin();

    if (l3gd20_fetch_data(L3GD20_REG_WHO_AM_I) != L3GD20_WHO_AM_I)
    {
        print_str_to_dbg_port("Failed reading who am I register\n");
        return 1;
    }
    else
    {
        print_str_to_dbg_port("Succeeded reading who am I register\n");
    }

    /* Enable L3GD20 Power bit */
    l3gd20_write_data(L3GD20_REG_CTRL_REG1, 0xFF);

    /* Set L3GD20 scale */
    l3gd20_write_data(L3GD20_REG_CTRL_REG4, 0x10);

    /* Set high-pass filter settings */
    l3gd20_write_data(L3GD20_REG_CTRL_REG2, 0x00);

    /* Enable high-pass filter */
    l3gd20_write_data(L3GD20_REG_CTRL_REG5, 0x10);

    return 0;
}

void l3gd20_compute_gyro_offsets(gyroData_st *gyro_offsets)
{
    gyroData_st gyro_data;
    float gyro_roll = 0, gyro_pitch = 0, gyro_yaw = 0;

    int counter;
    for (counter = 0; counter < 4000; counter++)
    {
        l3gd20_read_gyro(&gyro_data);
        gyro_roll += gyro_data.roll;
        gyro_pitch += gyro_data.pitch;
        gyro_yaw += gyro_data.yaw;

        tim10_delay_ms(2);
    }

    gyro_roll /= 4000;
    gyro_pitch /= 4000;
    gyro_yaw /= 4000;
    gyro_offsets->roll = gyro_roll;
    gyro_offsets->pitch = gyro_pitch;
    gyro_offsets->yaw = gyro_yaw;
}

void l3gd20_sub_gyro_offsets(gyroData_st *gyro_data, gyroData_st gyro_offsets)
{
    gyro_data->pitch -= gyro_offsets.pitch;
    gyro_data->roll -= gyro_offsets.roll;
    gyro_data->yaw -= gyro_offsets.yaw;
}

/**
 * @author SurgeExperiments
 * @brief this takes abt 24 microseconds to run. DMA not needed.
 */
uint8_t l3gd20_read_gyro(gyroData_st *my_data)
{
    /* Read X axis */
    my_data->roll = l3gd20_fetch_data(L3GD20_REG_OUT_X_L);
    my_data->roll |= l3gd20_fetch_data(L3GD20_REG_OUT_X_H) << 8;

    /* Read Y axis */
    my_data->pitch = l3gd20_fetch_data(L3GD20_REG_OUT_Y_L);
    my_data->pitch |= l3gd20_fetch_data(L3GD20_REG_OUT_Y_H) << 8;

    /* Read Z axis */
    my_data->yaw = l3gd20_fetch_data(L3GD20_REG_OUT_Z_L);
    my_data->yaw |= l3gd20_fetch_data(L3GD20_REG_OUT_Z_H) << 8;

    /* Return OK */
    return 0;
}
