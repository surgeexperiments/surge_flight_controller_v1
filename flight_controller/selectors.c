

#include "selectors.h"
#include "error.h"

/* We need to get the right define for a flight mode when initializing the IMU */
imuSelector_et get_imu_selector_for_flight_mode(flightModeSelector_et flightModeName)
{
	switch(flightModeName){
		
		case ACRO_MPU6050: {
			return MPU6050_NORMAL;
		} break; 
		
		case ACRO_L3GD20: {
			return L3GD20_NORMAL;
		} break; 
		
		case ACRO_MPU6050_DMA:{
			return MPU6050_DMA;
		} break; 
		
		case ACRO_L3GD_LSM303DLHC: {
			return L3GD_NORMAL_AND_LSM303DLHC_NORMAL;
		} break; 
		
		case AUTO_LEVEL_L3GD_LSM303DLHC_DMA_KALMAN: {
			return L3GD_NORMAL_AND_LSM303DLHC_DMA_TIM_INT;
		} break;

		/*
		case AUTO_LEVEL_L3GD_NORMAL_LSM303DLHC_DMA_MADGEWICK: {
			LOOP_imuInitSelector = L3GD_NORMAL_AND_LSM303DLHC_DMA_TIM_INT;
		} break;
		*/

		case AUTO_LEVEL_L3GD_LSM303DLHC_DMA_MAHONY: {
			return L3GD_NORMAL_AND_LSM303DLHC_DMA_TIM_INT;
		} break;

		default: {
			infinite_error_loop();
		} break;
	
	} /* END SWITCH */
}
