

#include "debug_info.h"
#include "enums.h"
#include "structs.h"
#include "../arm_drivers/dbg_swo_driver.h"

void print_startup_msg_flight_mode(int flightMode)
{
	switch(flightMode){
		
		case ACRO_MPU6050: {
			print_str_to_dbg_port("Mpu 6050 acro mode starting!\n");
		} break; 
		
		case ACRO_L3GD20: {
			print_str_to_dbg_port("L3gd20 acro mode starting!\n");
		} break; 
		
		case ACRO_MPU6050_DMA:{
			print_str_to_dbg_port("Mpu 6050 acro mode with dma starting!\n");
		} break; 
		
		case ACRO_L3GD_LSM303DLHC: {
			print_str_to_dbg_port("Dev board acro mode starting!\n");
		} break; 
		
		case AUTO_LEVEL_L3GD_LSM303DLHC_DMA_KALMAN: {
			print_str_to_dbg_port("Dev board with kalman starting!\n"); 
		} break;

		/*
		case AUTO_LEVEL_L3GD_NORMAL_LSM303DLHC_DMA_MADGEWICK: {
			LOOP_imuInitSelector = L3GD_NORMAL_AND_LSM303DLHC_DMA_TIM_INT;
		} break;
		*/

		case AUTO_LEVEL_L3GD_LSM303DLHC_DMA_MAHONY: {
			print_str_to_dbg_port("Dev board with mahony starting\n"); 
		} break;
		
		default: {
			return;
		} break; 
	
	} /* END SWITCH */
}