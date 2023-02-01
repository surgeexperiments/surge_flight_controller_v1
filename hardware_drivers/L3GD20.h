
#ifndef L3GD20_DRIVER_
#define L3GD20_DRIVER_

#include <stdint.h>
#include <math.h>

#include "../arm_drivers/spi_driver.h"
#include "../flight_controller/structs.h"

/* Identification number */
#define L3GD20_WHO_AM_I 0xD4

/* Registers addresses */
#define L3GD20_REG_WHO_AM_I 0x0F

#define SET_CS_HIGH GPIOE->BSRR = (1 << 3)
#define SET_CS_LOW GPIOE->BSRR = (1 << 19)

uint8_t l3gd20_init(void);
void l3gd20_compute_gyro_offsets(gyroData_st *gyro_offsets);
void l3gd20_sub_gyro_offsets(gyroData_st *gyro_data, gyroData_st gyro_offsets);
uint8_t l3gd20_read_gyro(gyroData_st *my_data);

/* Registers addresses */
#define L3GD20_REG_WHO_AM_I 0x0F
#define L3GD20_REG_CTRL_REG1 0x20
#define L3GD20_REG_CTRL_REG2 0x21
#define L3GD20_REG_CTRL_REG3 0x22
#define L3GD20_REG_CTRL_REG4 0x23
#define L3GD20_REG_CTRL_REG5 0x24
#define L3GD20_REG_REFERENCE 0x25
#define L3GD20_REG_OUT_TEMP 0x26
#define L3GD20_REG_STATUS_REG 0x27
#define L3GD20_REG_OUT_X_L 0x28
#define L3GD20_REG_OUT_X_H 0x29
#define L3GD20_REG_OUT_Y_L 0x2A
#define L3GD20_REG_OUT_Y_H 0x2B
#define L3GD20_REG_OUT_Z_L 0x2C
#define L3GD20_REG_OUT_Z_H 0x2D
#define L3GD20_REG_FIFO_CTRL_REG 0x2E
#define L3GD20_REG_FIFO_SRC_REG 0x2F
#define L3GD20_REG_INT1_CFG 0x30
#define L3GD20_REG_INT1_SRC 0x31
#define L3GD20_REG_INT1_TSH_XH 0x32
#define L3GD20_REG_INT1_TSH_XL 0x33
#define L3GD20_REG_INT1_TSH_YH 0x34
#define L3GD20_REG_INT1_TSH_YL 0x35
#define L3GD20_REG_INT1_TSH_ZH 0x36
#define L3GD20_REG_INT1_TSH_ZL 0x37
#define L3GD20_REG_INT1_DURATION 0x38

/* Sensitivity factors, datasheet pg. 9 */
#define L3GD20_SENSITIVITY_250 8.75 /* 8.75 mdps/digit */
#define L3GD20_SENSITIVITY_500 17.5 /* 17.5 mdps/digit */
#define L3GD20_SENSITIVITY_2000 70  /* 70 mdps/digit */

#endif /* L3GD20_DRIVER_ */
