#ifndef LSM303DLHC_H_
#define LSM303DLHC_H_

#include "../arm_drivers/i2c_driver.h"

#include "../flight_controller/structs.h"
#include "../arm_drivers/i2c_driver.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
 
uint8_t lsm303_init_normal(void);
uint8_t lsm303_init_dma_with_interrupt(void);
uint8_t lsm303_init_dma_no_int_pin(void);
uint8_t lsm303_init_dma_with_timing(void);
uint8_t lsm303_byte_read_sequence(uint8_t  internal_register, uint8_t *return_data);
uint8_t lsm303_clear_int1(void); 
uint8_t lsm303_dlhc_compute_accel_offsets(accelData_st *accel_offsets);
uint8_t lsm303_compute_accel_offsets_dma_no_int(accelData_st *accel_offsets);
uint8_t lsm303_compute_accel_offsets_dma_with_int(accelData_st *accel_offsets);
uint8_t lsm303_verify_accel_offset_integrity(accelData_st accel_offsets);
void lsm303_dma_read(void);
uint8_t lsm303_data_read(uint8_t *data_buffer, uint8_t number_of_items);
uint8_t lsm303_imu_raw_to_structs(uint8_t *data_buffer, accelData_st *accel_raw);
void lsm303_imu_raw_to_structs_dma(accelData_st *accel_raw);
void lsm303_fix_bytes(uint8_t *data_buffer, accelData_st *my_data);
uint8_t lsm303_single_byte_write_sequence(uint8_t internal_register, uint8_t command);
void lsm303_setup_tim11_int_timing(void);



#define LSM303_ADDRESS_ACCEL          (0x32 >> 1)         // 0011001x
#define LSM303_ADDRESS_ACCEL_WRITE   0x32
#define LSM303_ADDRESS_ACCEL_READ   0x33

#define LSM303_ADDRESS_MAG            (0x3C >> 1)         // 0011110x

                                                   // DEFAULT    TYPE
#define      LSM303_REGISTER_ACCEL_CTRL_REG1_A         0x20   // 00000111   rw
#define     LSM303_REGISTER_ACCEL_CTRL_REG2_A         0x21   // 00000000   rw
#define     LSM303_REGISTER_ACCEL_CTRL_REG3_A          0x22   // 00000000   rw
#define      LSM303_REGISTER_ACCEL_CTRL_REG4_A          0x23   // 00000000   rw
#define      LSM303_REGISTER_ACCEL_CTRL_REG5_A          0x24   // 00000000   rw
#define      LSM303_REGISTER_ACCEL_CTRL_REG6_A          0x25   // 00000000   rw
#define      LSM303_REGISTER_ACCEL_REFERENCE_A          0x26   // 00000000   r
#define      LSM303_REGISTER_ACCEL_STATUS_REG_A         0x27   // 00000000   r
#define      LSM303_REGISTER_ACCEL_OUT_X_L_A            0x28
#define      LSM303_REGISTER_ACCEL_OUT_X_H_A            0x29
#define      LSM303_REGISTER_ACCEL_OUT_Y_L_A            0x2A
#define      LSM303_REGISTER_ACCEL_OUT_Y_H_A            0x2B
#define      LSM303_REGISTER_ACCEL_OUT_Z_L_A            0x2C
#define     LSM303_REGISTER_ACCEL_OUT_Z_H_A            0x2D
#define     LSM303_REGISTER_ACCEL_FIFO_CTRL_REG_A      0x2E
#define      LSM303_REGISTER_ACCEL_FIFO_SRC_REG_A       0x2F
#define     LSM303_REGISTER_ACCEL_INT1_CFG_A           0x30
#define     LSM303_REGISTER_ACCEL_INT1_SOURCE_A        0x31
#define     LSM303_REGISTER_ACCEL_INT1_THS_A           0x32
#define      LSM303_REGISTER_ACCEL_INT1_DURATION_A      0x33
#define     LSM303_REGISTER_ACCEL_INT2_CFG_A           0x34
#define      LSM303_REGISTER_ACCEL_INT2_SOURCE_A        0x35
#define      LSM303_REGISTER_ACCEL_INT2_THS_A           0x36
#define      LSM303_REGISTER_ACCEL_INT2_DURATION_A      0x37
#define      LSM303_REGISTER_ACCEL_CLICK_CFG_A          0x38
#define      LSM303_REGISTER_ACCEL_CLICK_SRC_A          0x39
#define      LSM303_REGISTER_ACCEL_CLICK_THS_A          0x3A
#define      LSM303_REGISTER_ACCEL_TIME_LIMIT_A         0x3B
#define      LSM303_REGISTER_ACCEL_TIME_LATENCY_A       0x3C
#define     LSM303_REGISTER_ACCEL_TIME_WINDOW_A        0x3D

#endif /* LSM303DLHC_H_ */
