/**
 * @file enums.h
 * @author SurgeExperiments
 * 
 * NOTE: Have taken out multiple structs used for debugging
 * 		 and manual testing. They were over-complicating things.
*/

#ifndef ENUMS_H_
#define ENUMS_H_


typedef enum {
	MOTORS_OFF=0,
	MOTORS_STANDBY_MODE,
	MOTORS_FLY_MODE,
	MOTORS_ACROBATIC,
	MOTORS_RX_SIGNAL_LOST
} flyMode_et;

typedef enum {
 MPU6050_NORMAL=0,
 MPU6050_DMA,
 L3GD20_NORMAL, 
 L3GD_NORMAL_AND_LSM303DLHC_NORMAL,
 /* DMA initiated by timers */
 L3GD_NORMAL_AND_LSM303DLHC_DMA_TIM_INT,
 UNKNOWN_IMU_SELECTOR
} imuSelector_et;


/**
 * @brief Flight mode selectors used by main.c
 */
typedef enum {
	/* Gyro-only modes */
	ACRO_MPU6050=0,
	ACRO_L3GD20,
	ACRO_MPU6050_DMA,

	/* Non-auto level modes */
	ACRO_L3GD_LSM303DLHC,
	ACRO_FLYING_L3GD_NORMAL_LSM303DLHC_DMA,

	/* AUTO-LEVEL MODES: DMA read timer interrupt-activated */
	AUTO_LEVEL_L3GD_LSM303DLHC_DMA_KALMAN,
	AUTO_LEVEL_L3GD_NORMAL_LSM303DLHC_DMA_MADGEWICK,
	AUTO_LEVEL_L3GD_LSM303DLHC_DMA_MAHONY
} flightModeSelector_et;


/* Used for imu setup */
typedef enum {
 AFS_2G = 0,
 AFS_4G,
 AFS_8G,
 AFS_16G
} aScale_et;

typedef enum {
 GFS_250DPS = 0,
 GFS_500DPS,
 GFS_1000DPS,
 GFS_2000DPS
} gScale_et;


#endif /* ENUMS_H_ */



