/**
 * @file main.h
 * @author SurgeExperiments
 */

#include "flight_controller/structs.h"
#include "flight_controller/enums.h"

#include "main.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "arm_drivers/timing_driver.h"

#include "flight_controller/battery.h"
#include "flight_controller/debug_tools.h"
#include "flight_controller/error.h"
#include "flight_controller/esc.h"
#include "flight_controller/flight_mode_kalman.h"
#include "flight_controller/flight_mode_mahony.h"
#include "flight_controller/imu.h"
#include "flight_controller/init.h"
#include "flight_controller/led.h"
#include "flight_controller/pid.h"
#include "flight_controller/power.h"
#include "flight_controller/running_mode.h"
#include "flight_controller/rx_control.h"
#include "flight_controller/selectors.h"


extern rxChannels_st g_rx_channel_pulses;
extern pidStorage_st g_pid_roll_rotation, g_pid_roll_angle;
extern pidStorage_st g_pid_pitch_rotation, g_pid_pitch_angle;
extern pidStorage_st g_pid_yaw_rotation;


#define LEVELLING_MODE ACRO_L3GD20
/* One to activate else 0 */
#define USE_TPA 1


/**
 * @brief main function that runs everything.
 *
 * It first sets up the hardware based on the value in
 * real_time_settings.levelling_mode, and then
 * it goes into the flight loop which runs 416.67 times per
 * second when using a 2400 us loop time.
 * Loop time is defined in main.h, LOOP_TIME_MICROSECONDS.
 */
int main(void) {

	flyMode_et flight_mode;
	int battery_voltage;
	uint16_t led_flickering_counter = 0;

	/* ESC AND PID VARIABLES */
	escOutput_st esc_output;
	pidData_st pid_rotation_output, desired_rot_speed_3_axes; 

	/* CONTROL VARIABLES */
	droneRealTimeSettings_st real_time_settings;
	real_time_settings.levelling_mode = LEVELLING_MODE;
	real_time_settings.use_tpa = USE_TPA;
	real_time_settings.esc_voltage_compensation = 0;

	imuDataCollection_st imu_realtime_data;

	/* IMU: Auto-level variables */
	imuOrientation_st angle_orientation_data;
	kalmanVariables_st kalman_vars_x, kalman_vars_y;
	mahonyVariables_st mahony_vars; 
	uint8_t mahony_counter;

	/* IMU: Madgewick variables (gyro drag reduction) */
	madgwickQuaternionVars_st madgwick_vars;
	imuOrientationEuler_st madgwick_orientation; 
	float madgwick_accel_res, madgwick_gyro_res;
	float madgwick_gyro_bias[3] = {0, 0, 0}, madgwick_accel_bias[3] = {0, 0, 0};

	led_init();
	LED_ON(1);

	/* We use timers to generate PWMs to control the ESC's and thus motor speed */
	esc_init_hardware_pwm_gen();

	esc_set_motors_to_standby_speed(&esc_output);
	esc_generate_hw_pwm_motors(TIM2, esc_output);

	LED_ON(2);
	tim10_delay_ms(500);

	/* Goes into infinite error loop if the selector is unknown */
	imuSelector_et imu_init_selector = get_imu_selector_for_flight_mode(real_time_settings.levelling_mode); 

	LED_ON(3);
	if(imu_initialize_hardware(imu_init_selector,
							   &(imu_realtime_data.gyroCalibrationData),
							   &(imu_realtime_data.accelCalibrationData)) != 0) {
		infinite_error_loop(); 
	}		

	init_additional_fly_modes(real_time_settings.levelling_mode,
                              &imu_realtime_data,
                              &kalman_vars_x,
                              &kalman_vars_y,
                              &mahony_vars);

	rx_setup_timers_for_receiver_read();

	flight_mode = MOTORS_OFF;
	LOAD_BATTERY_VOLTAGE(battery_voltage);
	LED_ON(4);
	tim10_delay_ms(1000);

	LED_TOGGLE_ALL;

	/* Timer overflows after 65ms which we want to avoid, so we start it here.
	 * In the flight loop anything above 3ms is a guaranteed crash regardless
	 * so the 65ms limit is not a problem.
	 */
	tim9_setup_for_delay_count_us();

	pid_init_controller_structs(&g_pid_roll_rotation,
								&g_pid_pitch_rotation,
								&g_pid_yaw_rotation,
								&g_pid_roll_angle,
								&g_pid_pitch_angle);

	/* FLIGHT LOOP */
	while(1){
		
		/* Flicker the led every 500 loop iterations */
		flicker_led(&led_flickering_counter, (uint8_t)500);

		imuSelector_et curr_imu_mode = get_imu_selector_for_flight_mode(real_time_settings.levelling_mode);

		/* IMU calculations, RX-data processing */
		switch(real_time_settings.levelling_mode){

			/* If fetching IMU data fails we just crash the drone. It's unflyable anyway at that point */
			case ACRO_MPU6050: 
			case ACRO_L3GD20:
				if(imu_fetch_filtered_gyro_data(curr_imu_mode, &imu_realtime_data) != 0) {
					infinite_error_loop(); 
				}
				set_running_mode(&flight_mode, g_rx_channel_pulses);
				rx_signals_to_rot_deg_seconds(&desired_rot_speed_3_axes, g_rx_channel_pulses);
			 	break; 

			case ACRO_MPU6050_DMA:
			case ACRO_L3GD_LSM303DLHC: 
				if(imu_fetch_filtered_data(curr_imu_mode, &imu_realtime_data) != 0) {infinite_error_loop(); }
				set_running_mode(&flight_mode, g_rx_channel_pulses);
				rx_signals_to_rot_deg_seconds(&desired_rot_speed_3_axes, g_rx_channel_pulses);
				break; 

			case AUTO_LEVEL_L3GD_LSM303DLHC_DMA_KALMAN:
				if(imu_fetch_filtered_data(curr_imu_mode, &imu_realtime_data) != 0) {infinite_error_loop(); }

				flight_mode_kalman(&kalman_vars_x,
								   &kalman_vars_y, 
								   &angle_orientation_data,
								   &desired_rot_speed_3_axes,
								   &flight_mode,
								   imu_realtime_data,
								   g_rx_channel_pulses,
								   LOOP_TIME_MICROSECONDS);
				break;

			case AUTO_LEVEL_L3GD_LSM303DLHC_DMA_MAHONY: 
				if(imu_fetch_filtered_data(curr_imu_mode, &imu_realtime_data) != 0) {infinite_error_loop(); }

				flight_mode_mahony(&angle_orientation_data,
								   &mahony_vars,
								   &desired_rot_speed_3_axes,
								   &flight_mode,
								   imu_realtime_data,
								   g_rx_channel_pulses);
			 	break; 

			default: {
				infinite_error_loop(); 
			} break;
		
		} /* END SWITCH */

		/* This requires 500_DPS setting for the gyro */
		run_pid_calculations(imu_realtime_data.gyroDataFilteredToDegSeconds,
						  	 desired_rot_speed_3_axes,
						  	 &pid_rotation_output,
						  	 g_rx_channel_pulses.three,
						  	 real_time_settings.use_tpa);

		battery_voltage = BATTERY_COMPENSATE(battery_voltage);

		esc_calculate_pulse_length(flight_mode,
								   real_time_settings.esc_voltage_compensation,
								   g_rx_channel_pulses.three,
								   (int)1260,
								   pid_rotation_output,
								   &esc_output);

		/* Make the loop run exactly LOOP_TIME_MICROSECONDS */
		while(tim9_read_count() < LOOP_TIME_MICROSECONDS) {
		}

		tim9_reset_count(); 
		esc_generate_hw_pwm_motors(TIM2, esc_output);

	} /* End while */
} /* End main */

