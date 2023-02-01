

#include "init.h"
#include "imu.h"
#include "../arm_drivers/dbg_swo_driver.h"
#include "error.h"

/**
 * @author SurgeExperiments
 * @brief Some flight modes requires a bit of setup. This function handles this.
 * 		  On error it puts the STM into an infinite error loop with flickering 
 * 		  lights.
*/
void init_additional_fly_modes(flightModeSelector_et levelling_mode,
                               imuDataCollection_st *IMU_realtimeData,
                               kalmanVariables_st *LOOP_kalmanVariablesX,
                               kalmanVariables_st *LOOP_kalmanVariablesY,
                               mahonyVariables_st *LOOP_myMahoneyVariables) {

    /* Some of the auto-level methods require a bit of extra stuff when starting up */
	switch(levelling_mode){

		/* Fallthrough */
		case ACRO_MPU6050:
		case ACRO_L3GD20:
		case ACRO_MPU6050_DMA:
		case ACRO_L3GD_LSM303DLHC:
		case ACRO_FLYING_L3GD_NORMAL_LSM303DLHC_DMA:
		case AUTO_LEVEL_L3GD_NORMAL_LSM303DLHC_DMA_MADGEWICK: {
			break;
		}
		case AUTO_LEVEL_L3GD_LSM303DLHC_DMA_KALMAN: {
			if(imu_fetch_filtered_data(L3GD_NORMAL_AND_LSM303DLHC_DMA_TIM_INT, IMU_realtimeData) != 0) {
								print_str_to_dbg_port("MAIN PROGRAM: KALMAN : FETCHING DATA FAILED!");
								infinite_error_loop(); 
			}
				imu_init_kalman_filter(LOOP_kalmanVariablesX, LOOP_kalmanVariablesY, IMU_realtimeData->accelDataFiltered);
			} break;

		case AUTO_LEVEL_L3GD_LSM303DLHC_DMA_MAHONY: {
			imu_init_mahoney_algorithm(LOOP_myMahoneyVariables);
		} break;
		
	} /* END SWITCH */
}