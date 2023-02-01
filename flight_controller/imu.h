/**
 *	@file imu.h
 *	@author SurgeExperiments
 *
 *	@brief This file contains the defines and prototypes for using the IMU
 */

#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>
#include "enums.h"
#include "structs.h"

/* this is for 500dps, so set all gyros to this (or create some custom stuff) */
#define GYRO_FILTER_DIVISOR 57.14286f
#define ACCEL_TO_DEG_DIVISOR 4096

/* Axis reversal defines (1 for no reversing, -1 for reversing).
 * No error testing for these defines being set to bogus values
 */

/* NO-CAM:
#define ROLL_AXIS_REVERSE	1
#define PITCH_AXIS_REVERSE -1
#define YAW_AXIS_REVERSE   -1
*/

/* CAM */
#define ROLL_AXIS_REVERSE -1
#define PITCH_AXIS_REVERSE 1
#define YAW_AXIS_REVERSE -1

/* postfix with _ to avoid redef error */
#define M_PI_ 3.14159265f

uint8_t imu_initialize_hardware(imuSelector_et imu_selector,
                                gyroData_st *gyro_offsets,
                                accelData_st *accel_offsets);

void gyro_compensation(const gyroData_st gyro_offsets, gyroData_st *gyro_output);
uint8_t imu_fetch_filtered_gyro_data(imuSelector_et gyro_selector, imuDataCollection_st *imu_data);
uint8_t imu_fetch_filtered_data(imuSelector_et imu_selector, imuDataCollection_st *imu_data);

/* MADGEWICK SENSOR FUSION */
void init_madgewick_variables(madgwickQuaternionVars_st *init_me,
                              float *a_res,
                              aScale_et a_scale,
                              float *g_res,
                              gScale_et g_scale);

void madgwick_imu_scale(accelData_st *accel_raw,
                        float accel_scale,
                        gyroData_st *gyro_raw,
                        gyroData_st *gyro_madgewick,
                        float gyro_scale);

void MadgwickQuaternionUpdate(float ax,
                              float ay,
                              float az,
                              float gx,
                              float gy,
                              float gz,
                              float deltat,
                              float beta,
                              float zeta,
                              madgwickQuaternionVars_st *madgewick_vars);

void quaternion_to_euler_angles(imuOrientationEuler_st *imu_orientation, float *q);

/* KALMAN FILTER */
void imu_init_kalman_filter(kalmanVariables_st *kalman_variables_x,
                            kalmanVariables_st *kalman_variables_y,
                            accelData_st accel_filtered);

void imu_run_kalman_filter(kalmanVariables_st *kalman_variables_x,
                           kalmanVariables_st *kalman_variables_y,
                           gyroData_st gyro_filtered,
                           accelData_st accel_filtered,
                           float dt);

void initialize_kalman_struct(kalmanVariables_st *kalman_data);

/* MAHONY FUNCTIONS */
void imu_init_mahoney_algorithm(mahonyVariables_st *instance);

void mahony_update(mahonyVariables_st *mahoney_vars,
                   float gx,
                   float gy,
                   float gz,
                   float ax,
                   float ay,
                   float az,
                   float mx,
                   float my,
                   float mz);

void imu_mahony_update(mahonyVariables_st *mahoney_vars,
                       float gx,
                       float gy,
                       float gz,
                       float ax,
                       float ay,
                       float az);

float imu_mahony_get_roll(mahonyVariables_st *mahoney_vars);
float imu_mahony_get_pitch(mahonyVariables_st *mahoney_vars);
float imu_mahony_get_yaw(mahonyVariables_st *mahoney_vars);
float imu_mahony_get_roll_radians(mahonyVariables_st *mahoney_vars);
float imu_mahony_get_pitch_radians(mahonyVariables_st *mahoney_vars);
float imu_mahony_get_yaw_radians(mahonyVariables_st *mahoney_vars);

#endif /* IMU_H_ */
