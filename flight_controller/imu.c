/**
 * @file imu.c
 * @author SurgeExperiments
 *
 * @brief File containing functions to initialize and get data from various sensors.
 *        Also contains sensor fusion algorithms and Kalman filter.
 *        These are found online and customized
 *        to work with the data structures in this program.
 *
 *
 *        The IMU drivers handles setup, calibration and getting
 *        output data into the data structures for gyro
 *        and accel data in DMA or normal mode (incl subtracting calibration data).
 *
 *        This module orchestrates different IMU-combinations and handles
 *        higher level data-manipulation such as axis reversing,
 *        auto-levelling stuff, drift-elimination and so on.
 *
 *        Set the correct IMU axis reversals in imu.h
 *        (depending on how the IMU is mounted on the drone)
 */

#include <stdlib.h>
#include <math.h>
#include "imu.h"
#include "../arm_drivers/dbg_swo_driver.h"
#include "../arm_drivers/timing_driver.h"

#include "../hardware_drivers/mpu6050.h"
#include "../hardware_drivers/L3GD20.h"
#include "../hardware_drivers/lsm303dlhc.h"

#include "structs.h"

/**
 * @brief define used for Kalman
 */
#define RAD_TO_DEG 57.2957786

/**
 * @author SurgeExperiments
 * @brief function that computes the offsets for gyro and accel (with the right IMUSelector)
 *
 * When using an IMU_DATA_CONTAINER struct, just pass the fields as pointers
 * IMPORTANT: DO NOT MOVE THE SENSOR WHEN OFFSETS ARE COMPUTED!
 *
 * @param[in] IMUSelector which IMU install you want, check the enum cases in the switch for options
 * @param[out] gyro_offsets pointer to gyro_data struct that will contain avg offset for the gyro
 * @param[out] accel_offsets ptr to acce_data struct that will contain the avg offset for the accel
 *
 */
uint8_t imu_initialize_hardware(imuSelector_et imu_selector, gyroData_st *gyro_offsets, accelData_st *accel_offsets)
{
    switch (imu_selector)
    {

    case MPU6050_NORMAL:
    {
        if (mpu_6050_init_normal() != 0)
            return 1;
        tim10_delay_ms(1000);
        mpu6050_compute_gyro_offsets(gyro_offsets);
    }
    break;

    case MPU6050_DMA:
    {
        if (mpu6050_init_dma() != 0)
            return 1;
        tim10_delay_ms(1000);
        mpu6050_compute_imu_offsets_dma(gyro_offsets, accel_offsets);
    }
    break;

    case L3GD20_NORMAL:
    {
        if (l3gd20_init() != 0)
            return 1;
        tim10_delay_ms(1000);
        l3gd20_compute_gyro_offsets(gyro_offsets);
    }
    break;

    case L3GD_NORMAL_AND_LSM303DLHC_NORMAL:
    {
        if (l3gd20_init() != 0)
            return 1;
        tim10_delay_ms(1000);
        l3gd20_compute_gyro_offsets(gyro_offsets);
        lsm303_init_dma_no_int_pin();
        lsm303_compute_accel_offsets_dma_no_int(accel_offsets);
    }
    break;

    case L3GD_NORMAL_AND_LSM303DLHC_DMA_TIM_INT:
    {
        if (l3gd20_init() != 0)
            return 1;
        tim10_delay_ms(1000);
        l3gd20_compute_gyro_offsets(gyro_offsets);
        if (lsm303_init_dma_with_timing() != 0)
            return 1;
        lsm303_compute_accel_offsets_dma_with_int(accel_offsets);
        /* If these are 0 the DMA read is not working and we need to crash the loading */
        if (lsm303_verify_accel_offset_integrity(*accel_offsets) == 1)
            return 1;
    }
    break;

    default:
        break;
    } /* END SWITCH */

    return 0;
}

/**
 * @author SurgeExperiments
 * @brief the gyro might b mounted in the opposite direction of the drone,
 *        so it gives the wrong output signals in relation to the drones position.
 *        By setting the defines R/P/Y_AXIS_REVERSE you can reverse the gyro signals to
 *        make the drone work with a gyro mounted in the opposite way
 *
 * @param[out] gyro_raw self explanatory
 */
static void apply_gyro_axis_reversal(gyroData_st *gyro_raw)
{
    gyro_raw->roll *= ROLL_AXIS_REVERSE;
    gyro_raw->pitch *= PITCH_AXIS_REVERSE;
    gyro_raw->yaw *= YAW_AXIS_REVERSE;
}

/**
 * @author SurgeExperiments
 * @brief the accel might b mounted in the opposite direction of the drone,
 *        so it gives the wrong output signals in relation to the drones position.
 *        By setting the defines R/P/Y_AXIS_REVERSE you can reverse the accel signals to
 *        make the drone work with an accel mounted in the opposite way
 *
 * @param[out] accel_raw self explanatory
 */
static void imu_apply_axis_reversal(accelData_st *accel_raw, gyroData_st *gyro_raw)
{
    gyro_raw->roll *= ROLL_AXIS_REVERSE;
    gyro_raw->pitch *= PITCH_AXIS_REVERSE;
    gyro_raw->yaw *= YAW_AXIS_REVERSE;
    accel_raw->roll *= ROLL_AXIS_REVERSE;
    accel_raw->pitch *= PITCH_AXIS_REVERSE;
    accel_raw->z *= YAW_AXIS_REVERSE;
}

/**
 * @author SurgeExperiments
 * @brief function that fetches gyro data, incl subtracting the gyro offsets
 *
 * @param[in] gyro_selector check the switch case for into, self explanatory
 */
uint8_t imu_fetch_filtered_gyro_data(imuSelector_et gyro_selector, imuDataCollection_st *imu_data)
{
    switch (gyro_selector)
    {

    case MPU6050_NORMAL:
    {
        if (mpu6050_gyro_raw_to_structs(imu_data->dataBuffer, &(imu_data->gyroDataRaw)) != 0)
            return 1;
    }
    break;

    case L3GD20_NORMAL:
    {
        if (l3gd20_read_gyro(&(imu_data->gyroDataRaw)) != 0)
            return 1;
    }
    break;

    default:
        break;
    } /* END SWITCH */

    /* Get inversions right by looking at the MPU6050 data test in /test_utils
     * If the angle-movements don't match the prescribed ones in the test-file, fix them or the
     * PID controller tries to adjust in the wrong direction == instacrash
     */

    /* Subtract calibration data (BEFORE inverting haha) */
    imu_data->gyroDataRaw.pitch -= imu_data->gyroCalibrationData.pitch;
    imu_data->gyroDataRaw.roll -= imu_data->gyroCalibrationData.roll;
    imu_data->gyroDataRaw.yaw -= imu_data->gyroCalibrationData.yaw;

    apply_gyro_axis_reversal(&(imu_data->gyroDataRaw));

    /* Run filter + set input to deg/sec (this is what the PID controller wants)
     * TODO: Turn floating point constants into defines in the .h file
     */
    imu_data->gyroDataFilteredToDegSeconds.roll = (imu_data->gyroDataFilteredToDegSeconds.roll * 0.8f) +
                                                  ((imu_data->gyroDataRaw.roll / GYRO_FILTER_DIVISOR) * 0.2f);
    imu_data->gyroDataFilteredToDegSeconds.pitch = (imu_data->gyroDataFilteredToDegSeconds.pitch * 0.8f) +
                                                   ((imu_data->gyroDataRaw.pitch / GYRO_FILTER_DIVISOR) * 0.2f);
    imu_data->gyroDataFilteredToDegSeconds.yaw = (imu_data->gyroDataFilteredToDegSeconds.yaw * 0.8f) +
                                                 ((imu_data->gyroDataRaw.yaw / GYRO_FILTER_DIVISOR) * 0.2f);

    return 0;
}

/**
 * @author SurgeExperiments
 * @brief function that fetches data based on the param IMUSelector
 *
 * @param[in] IMUSelector selects which type of data and by which type of sensor
 * @param[out] IMU_data self explanatory
 *             IMPORTANT: the data buffer must be at least 14 bytes long!
 *                        (or you'll corrupt data somewhere)
 *
 * @retval 0 on success, 1 on error. An error will be caused by a sensor read-error.
 */
uint8_t imu_fetch_filtered_data(imuSelector_et imu_selector, imuDataCollection_st *imu_data)
{
    switch (imu_selector)
    {

    case MPU6050_NORMAL:
        if (mpu6050_imu_raw_to_structs(imu_data->dataBuffer,
                                       &(imu_data->accelDataRaw),
                                       &(imu_data->tempDataRaw),
                                       &(imu_data->gyroDataRaw)) != 0)
            return 1;
        break;

    case MPU6050_DMA:
    {
        mpu6050_imu_raw_to_structs_dma(&(imu_data->accelDataRaw),
                                       &(imu_data->tempDataRaw),
                                       &(imu_data->gyroDataRaw));
    }
    break;

    case L3GD_NORMAL_AND_LSM303DLHC_NORMAL:
    {
        if (l3gd20_read_gyro(&(imu_data->gyroDataRaw)) != 0)
            return 1;
        if (lsm303_imu_raw_to_structs(imu_data->dataBuffer,
                                      &(imu_data->accelDataRaw)) != 0)
            return 1;
    }
    break;

    case L3GD_NORMAL_AND_LSM303DLHC_DMA_TIM_INT:
    {
        if (l3gd20_read_gyro(&(imu_data->gyroDataRaw)) != 0)
            return 1;
        lsm303_imu_raw_to_structs_dma(&(imu_data->accelDataRaw));
    }
    break;

    default:
        break;
    } /* END SWITCH */

    /* Get inversions right by looking at the MPU6050 data test in /test_utils
     * If the angle-movements don't match the prescribed ones in the test-file, fix them or the
     * PID controller tries to adjust in the wrong direction == instacrash
     */

    /* Subtract calibration data  (BEFORE inverting haha) */
    imu_data->gyroDataRaw.pitch -= imu_data->gyroCalibrationData.pitch;
    imu_data->gyroDataRaw.roll -= imu_data->gyroCalibrationData.roll;
    imu_data->gyroDataRaw.yaw -= imu_data->gyroCalibrationData.yaw;
    imu_data->accelDataRaw.pitch -= imu_data->accelCalibrationData.pitch;
    imu_data->accelDataRaw.roll -= imu_data->accelCalibrationData.roll;
    imu_data->accelDataRaw.z -= imu_data->accelCalibrationData.z;

    /* Axis reversal (user: set these values in imu.h) */
    imu_apply_axis_reversal(&(imu_data->accelDataRaw), &(imu_data->gyroDataRaw));

    /* Complementary filter for gyro and accel data */
    imu_data->gyroDataFilteredToDegSeconds.roll = (imu_data->gyroDataFilteredToDegSeconds.roll * 0.8f) +
                                                  ((imu_data->gyroDataRaw.roll / GYRO_FILTER_DIVISOR) * 0.2f);

    imu_data->gyroDataFilteredToDegSeconds.pitch = (imu_data->gyroDataFilteredToDegSeconds.pitch * 0.8f) +
                                                   ((imu_data->gyroDataRaw.pitch / GYRO_FILTER_DIVISOR) * 0.2f);

    imu_data->gyroDataFilteredToDegSeconds.yaw = (imu_data->gyroDataFilteredToDegSeconds.yaw * 0.8f) +
                                                 ((imu_data->gyroDataRaw.yaw / GYRO_FILTER_DIVISOR) * 0.2f);

    imu_data->accelDataFiltered.roll = (imu_data->accelDataFiltered.roll * 0.9f) +
                                       imu_data->accelDataFiltered.roll * 0.1f;

    imu_data->accelDataFiltered.pitch = (imu_data->accelDataFiltered.pitch * 0.9f) +
                                        imu_data->accelDataFiltered.pitch * 0.1f;

    imu_data->accelDataFiltered.z = (imu_data->accelDataFiltered.z * 0.9f) +
                                    imu_data->accelDataFiltered.z * 0.1f;

    return 0;
}

/* MADGEWICK FUNCTIONS */

/**
 * @author SurgeExperiments
 * @brief as the name says, init the madgewick variables
 */
void init_madgewick_variables(madgwickQuaternionVars_st *init_me,
                              float *a_res,
                              aScale_et a_scale,
                              float *g_res,
                              gScale_et g_scale)
{
    /* beta and zeta can b experimented with */
    init_me->beta = sqrt(3.0f / 4.0f) * M_PI_ * (40.0f / 180.0f);
    init_me->zeta = sqrt(3.0f / 4.0f) * M_PI_ * (2.0f / 180.0f);
    init_me->deltat = 0;
    init_me->q[0] = 1.0f;
    init_me->q[1] = 0.0f;
    init_me->q[2] = 0.0f;
    init_me->q[3] = 0.0f;

    switch (g_scale)
    {
    /* Possible gyro scales (and their register bit settings) are:
     * 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
     * Here's a bit of an algorithm to calculate DPS/(ADC tick) based on that 2-bit value:
     */
    case GFS_250DPS:
        *g_res = 250.0f / 32768.0f;
        break;
    case GFS_500DPS:
        *g_res = 500.0f / 32768.0f;
        break;
    case GFS_1000DPS:
        *g_res = 1000.0f / 32768.0f;
        break;
    case GFS_2000DPS:
        *g_res = 2000.0f / 32768.0f;
        break;
    }

    switch (a_scale)
    {
    /* Possible accelerometer scales (and their register bit settings) are:
     * 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
     * Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
     */
    case AFS_2G:
        *a_res = 2.0f / 32768.0f;
        break;
    case AFS_4G:
        *a_res = 4.0f / 32768.0f;
        break;
    case AFS_8G:
        *a_res = 8.0f / 32768.0f;
        break;
    case AFS_16G:
        *a_res = 16.0f / 32768.0f;
        break;
    }
}

/**
 * @author SurgeExperiments
 * @brief Madgewick needs to scale the raw gyro and accel data.
 */
void madgwick_imu_scale(accelData_st *accel_raw,
                        float accel_scale,
                        gyroData_st *gyro_raw,
                        gyroData_st *gyro_madgewick,
                        float gyro_scale)
{
    accel_raw->roll = accel_raw->roll * accel_scale;
    accel_raw->pitch = accel_raw->pitch * accel_scale;
    accel_raw->z = accel_raw->z * accel_scale;

    /* Can't transform gyroRaw here since the PID need the unscaled outputs */
    gyro_madgewick->roll = gyro_raw->roll * gyro_scale;
    gyro_madgewick->pitch = gyro_raw->pitch * gyro_scale;
    gyro_madgewick->yaw = gyro_raw->yaw * gyro_scale;
}

/**
 * @brief computes the quaternions that represents the current position in space
 *        NOTE: The Madgewick sensor fusion algorithm requires up to a few seconds
 *              to stabilize it's measurements. Before they have converged they are
 *              unreliable.
 *
 * @param a[xyz] accelerometer input xyz
 * @param g[xyz] gyro input xyz
 * @param deltat delta time since the last call of this func in microseconds.
 *               Set to 0 on first call (this will be while waiting for convergence of values)
 * @param[out] myMadgewickVars See structs.h for struct definitions.
 */
void MadgwickQuaternionUpdate(float ax,
                              float ay,
                              float az,
                              float gx,
                              float gy,
                              float gz,
                              float deltat,
                              float beta,
                              float zeta,
                              madgwickQuaternionVars_st *madgewick_vars)
{

    float q1 = madgewick_vars->q[0],
          q2 = madgewick_vars->q[1],
          q3 = madgewick_vars->q[2],
          q4 = madgewick_vars->q[3];

    float norm;                                               // vector norm
    float f1, f2, f3;                                         // objetive funcyion elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz, gbiasx = 0, gbiasy = 0, gbiasz = 0; // gyro bias error

    /* Auxiliary variables to avoid repeated arithmetic */
    float _halfq1 = 0.5f * q1;
    float _halfq2 = 0.5f * q2;
    float _halfq3 = 0.5f * q3;
    float _halfq4 = 0.5f * q4;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;

    /* Normalise accelerometer measurement */
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f)
        return; // handle NaN
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    /* Compute the objective function and Jacobian */
    f1 = _2q2 * q4 - _2q1 * q3 - ax;
    f2 = _2q1 * q2 + _2q3 * q4 - ay;
    f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
    J_11or24 = _2q3;
    J_12or23 = _2q4;
    J_13or22 = _2q1;
    J_14or21 = _2q2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;

    /* Compute the gradient (matrix multiplication) */
    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 * f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;

    /* Normalize the gradient */
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;

    /* Compute estimated gyroscope biases */
    gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
    gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
    gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

    /* Compute and remove gyroscope biases */
    gbiasx += gerrx * deltat * zeta;
    gbiasy += gerry * deltat * zeta;
    gbiasz += gerrz * deltat * zeta;
    gx -= gbiasx;
    gy -= gbiasy;
    gz -= gbiasz;

    /* Compute the quaternion derivative */
    qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
    qDot2 = _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
    qDot3 = _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
    qDot4 = _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

    /* Compute then integrate estimated quaternion derivative */
    q1 += (qDot1 - (beta * hatDot1)) * deltat;
    q2 += (qDot2 - (beta * hatDot2)) * deltat;
    q3 += (qDot3 - (beta * hatDot3)) * deltat;
    q4 += (qDot4 - (beta * hatDot4)) * deltat;

    /* Normalize the quaternion */
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4); // normalise quaternion
    norm = 1.0f / norm;
    madgewick_vars->q[0] = q1 * norm;
    madgewick_vars->q[1] = q2 * norm;
    madgewick_vars->q[2] = q3 * norm;
    madgewick_vars->q[3] = q4 * norm;
}

/**
 * @brief get roll/pitch/yaw from the array q from an instance
 *        of a MadgewickQuaternionVariables struct.
 *
 * NOTE: this doesn't prevent gimbal-lock! A better alternative
 *       is to run the drone positioning on the actual computed
 *       quaternions instead of using pitch/roll/yaw.
 *
 * @param[in] q quaternion struct from the MadgewickQuaternionVariables struct
 * @param[out] imu_orientation_ struct being filled with the computed angles
 */
void quaternion_to_euler_angles(imuOrientationEuler_st *imu_orientation, float *q)
{
    /* Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
     * In this coordinate system, the positive z-axis is down toward Earth.
     * Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination,
     * looking down on the sensor positive yaw is counterclockwise.
     * Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
     * Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
     * These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
     * Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
     * applied in the correct order which for this configuration is yaw, pitch, and then roll.
     * For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
     */

    /*
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / M_PI_;
    yaw   *= 180.0f / M_PI_;
    roll  *= 180.0f / M_PI_;
    */

    imu_orientation->yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    imu_orientation->pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    imu_orientation->roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    imu_orientation->pitch *= 180.0f / M_PI_;
    imu_orientation->yaw *= 180.0f / M_PI_;
    imu_orientation->roll *= 180.0f / M_PI_;
}

/* KALMAN FUNCTIONS */

/**
 * @brief kalman filter function found online on tkjelectronic
 */
static float get_angle(kalmanVariables_st *kalman_data, float new_angle, float new_rate, float dt)
{
    // Update xhat - Project the state ahead
    /* Step 1 */
    kalman_data->rate = new_rate - kalman_data->bias;
    kalman_data->angle += dt * kalman_data->rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    kalman_data->P[0][0] += dt * (dt * kalman_data->P[1][1] - kalman_data->P[0][1] - kalman_data->P[1][0] + kalman_data->Q_angle);
    kalman_data->P[0][1] -= dt * kalman_data->P[1][1];
    kalman_data->P[1][0] -= dt * kalman_data->P[1][1];
    kalman_data->P[1][1] += kalman_data->Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    kalman_data->S = kalman_data->P[0][0] + kalman_data->R_measure;
    /* Step 5 */
    kalman_data->K[0] = kalman_data->P[0][0] / kalman_data->S;
    kalman_data->K[1] = kalman_data->P[1][0] / kalman_data->S;

    // Calculate angle and bias - Update estimate with measurement zk (new_angle)
    /* Step 3 */
    kalman_data->y = new_angle - kalman_data->angle;
    /* Step 6 */
    kalman_data->angle += kalman_data->K[0] * kalman_data->y;
    kalman_data->bias += kalman_data->K[1] * kalman_data->y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    kalman_data->P[0][0] -= kalman_data->K[0] * kalman_data->P[0][0];
    kalman_data->P[0][1] -= kalman_data->K[0] * kalman_data->P[0][1];
    kalman_data->P[1][0] -= kalman_data->K[1] * kalman_data->P[0][0];
    kalman_data->P[1][1] -= kalman_data->K[1] * kalman_data->P[0][1];

    return kalman_data->angle;
};

static void set_angle(kalmanVariables_st *kalman_data, float new_angle)
{
    kalman_data->angle = new_angle;
};

/**
 * @brief set up kalman structs.
 *
 * TODO: clear this function up, replace X and Y with euler angle names
 */
void imu_init_kalman_filter(kalmanVariables_st *kalman_variables_x,
                            kalmanVariables_st *kalman_variables_y,
                            accelData_st accel_filtered)
{
    /* NOTE: run AFTER the IMU has been installed and the first data has been fetched */
    initialize_kalman_struct(kalman_variables_x);
    initialize_kalman_struct(kalman_variables_y);

    /* Wait for sensor to stabilize */
    tim10_delay_ms(100);

    kalman_variables_y->intermediateAngle = atan(-accel_filtered.roll /
                                                 sqrt(accel_filtered.pitch * accel_filtered.pitch + accel_filtered.z * accel_filtered.z)) *
                                            RAD_TO_DEG;

    kalman_variables_x->intermediateAngle = atan(accel_filtered.pitch /
                                                 sqrt(accel_filtered.roll * accel_filtered.roll + accel_filtered.z * accel_filtered.z)) *
                                            RAD_TO_DEG;

    set_angle(kalman_variables_x, kalman_variables_x->intermediateAngle);
    set_angle(kalman_variables_y, kalman_variables_y->intermediateAngle);
}

/**
 * @brief run the kalman filter and compute the pitch and roll angles of the drone.
 *
 *        http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
 *
 *        TODO: test implementation more. Flights are a bit glitchy.
 *
 * @param dt delta time since last function call in microseconds
 */
void imu_run_kalman_filter(kalmanVariables_st *kalman_variables_x,
                           kalmanVariables_st *kalman_variables_y,
                           gyroData_st gyro_filtered,
                           accelData_st accel_filtered,
                           float dt)
{
    dt = dt / 1000000;

    kalman_variables_y->intermediateAngle = atan(-accel_filtered.roll /
                                                 sqrt(accel_filtered.pitch * accel_filtered.pitch + accel_filtered.z * accel_filtered.z)) *
                                            RAD_TO_DEG;

    kalman_variables_x->intermediateAngle = atan(accel_filtered.pitch /
                                                 sqrt(accel_filtered.roll * accel_filtered.roll + accel_filtered.z * accel_filtered.z)) *
                                            RAD_TO_DEG;

    kalman_variables_x->gyroRate = gyro_filtered.roll;
    kalman_variables_y->gyroRate = gyro_filtered.pitch;

    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((kalman_variables_x->intermediateAngle < -90 && kalman_variables_x->finalAngle > 90) ||
        (kalman_variables_x->intermediateAngle > 90 && kalman_variables_x->finalAngle < -90))
    {

        set_angle(kalman_variables_x, kalman_variables_x->intermediateAngle);
        kalman_variables_x->finalAngle = kalman_variables_x->intermediateAngle;
    }
    else
    {
        kalman_variables_x->finalAngle = get_angle(kalman_variables_x,
                                                   kalman_variables_x->intermediateAngle,
                                                   kalman_variables_x->gyroRate, dt);

        // Invert rate, so it fits the restriced accelerometer reading
        if (fabs(kalman_variables_x->finalAngle) > 90)
            kalman_variables_y->gyroRate = -kalman_variables_y->gyroRate;

        kalman_variables_y->finalAngle = get_angle(kalman_variables_y,
                                                   kalman_variables_y->intermediateAngle,
                                                   kalman_variables_y->gyroRate, dt);

        /* This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees */
        if ((kalman_variables_y->intermediateAngle < -90 && kalman_variables_y->finalAngle > 90) ||
            (kalman_variables_y->intermediateAngle > 90 && kalman_variables_y->finalAngle < -90))
        {
            set_angle(kalman_variables_y, kalman_variables_y->intermediateAngle);
            kalman_variables_y->finalAngle = kalman_variables_y->intermediateAngle;
        }
        else
        {
            kalman_variables_y->finalAngle = get_angle(kalman_variables_y,
                                                       kalman_variables_y->intermediateAngle,
                                                       kalman_variables_y->gyroRate,
                                                       dt);
        }
        if (fabs(kalman_variables_y->finalAngle) > 90)
        {
            kalman_variables_x->gyroRate = -kalman_variables_x->gyroRate;
            kalman_variables_x->finalAngle = get_angle(kalman_variables_x,
                                                       kalman_variables_x->intermediateAngle,
                                                       kalman_variables_x->gyroRate,
                                                       dt);
        }
    }
}

void initialize_kalman_struct(kalmanVariables_st *kalman_data)
{
    /* We will set the variables like so, these can also be tuned by the user */
    kalman_data->Q_angle = 0.001;
    kalman_data->Q_bias = 0.003;
    kalman_data->R_measure = 0.03;

    kalman_data->angle = 0; // Reset the angle
    kalman_data->bias = 0;  // Reset bias

    /* Since we assume that the bias is 0 and we know the starting angle (use setAngle),
     * the error covariance matrix is set like so
     * see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
     */
    kalman_data->P[0][0] = 0;
    kalman_data->P[0][1] = 0;
    kalman_data->P[1][0] = 0;
    kalman_data->P[1][1] = 0;
}

/* MAHONEY FUNCTIONS */

#define DEFAULT_SAMPLE_FREQ 416.67f // sample frequency in Hz
#define twoKpDef (2.0f * 0.5f)      // 2 * proportional gain
#define twoKiDef (2.0f * 0.0f)      // 2 * integral gain

/* Reference implementatinos of AHRS algorithms */

//============================================================================================
// Functions

//-------------------------------------------------------------------------------------------
// AHRS algorithm update

float normal_inv_sqrt(float x)
{
    return 1 / sqrt(x);
}

static void begin(mahonyVariables_st *mahoney_vars, float sampleFrequency)
{
    mahoney_vars->invSampleFreq = 1.0f / sampleFrequency;
}

/**
 * @brief Old school Carmack-hack
 *
 */
static float inv_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    // Could probably do this simpler, but faster?
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void imu_init_mahoney_algorithm(mahonyVariables_st *instance)
{
    /* 2 * proportional gain (Kp) */
    instance->twoKp = twoKpDef;

    /* 2 * integral gain (Ki) */
    instance->twoKi = twoKiDef;
    instance->q0 = 1.0f;
    instance->q1 = 0.0f;
    instance->q2 = 0.0f;
    instance->q3 = 0.0f;
    instance->integralFBx = 0.0f;
    instance->integralFBy = 0.0f;
    instance->integralFBz = 0.0f;
    instance->anglesComputed = 0;
    instance->invSampleFreq = 1.0f / DEFAULT_SAMPLE_FREQ;
}

void mahony_update(mahonyVariables_st *mahoney_vars,
                   float gx,
                   float gy,
                   float gz,
                   float ax,
                   float ay,
                   float az,
                   float mx,
                   float my,
                   float mz)
{
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    /* Use IMU algorithm if magnetometer measurement invalid
     * (avoids NaN in magnetometer normalisation)
     */
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        imu_mahony_update(mahoney_vars, gx, gy, gz, ax, ay, az);
        return;
    }

    /* Convert gyroscope degrees/sec to radians/sec */
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    /* Compute feedback only if accelerometer measurement valid
     * (avoids NaN in accelerometer normalisation)
     */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        /* Normalise accelerometer measurement */
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* Normalise magnetometer measurement */
        recipNorm = inv_sqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        /* Auxiliary variables to avoid repeated arithmetic */
        q0q0 = mahoney_vars->q0 * mahoney_vars->q0;
        q0q1 = mahoney_vars->q0 * mahoney_vars->q1;
        q0q2 = mahoney_vars->q0 * mahoney_vars->q2;
        q0q3 = mahoney_vars->q0 * mahoney_vars->q3;
        q1q1 = mahoney_vars->q1 * mahoney_vars->q1;
        q1q2 = mahoney_vars->q1 * mahoney_vars->q2;
        q1q3 = mahoney_vars->q1 * mahoney_vars->q3;
        q2q2 = mahoney_vars->q2 * mahoney_vars->q2;
        q2q3 = mahoney_vars->q2 * mahoney_vars->q3;
        q3q3 = mahoney_vars->q3 * mahoney_vars->q3;

        /* Reference direction of Earth's magnetic field */
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        /* Estimated direction of gravity and magnetic field */
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        /* Error is sum of cross product between estimated direction
         * and measured direction of field vectors
         */
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        /* Compute and apply integral feedback if enabled */
        if (mahoney_vars->twoKi > 0.0f)
        {
            /* integral error scaled by Ki */
            mahoney_vars->integralFBx += mahoney_vars->twoKi * halfex * mahoney_vars->invSampleFreq;
            mahoney_vars->integralFBy += mahoney_vars->twoKi * halfey * mahoney_vars->invSampleFreq;
            mahoney_vars->integralFBz += mahoney_vars->twoKi * halfez * mahoney_vars->invSampleFreq;
            gx += mahoney_vars->integralFBx; // apply integral feedback
            gy += mahoney_vars->integralFBy;
            gz += mahoney_vars->integralFBz;
        }
        else
        {
            mahoney_vars->integralFBx = 0.0f; // prevent integral windup
            mahoney_vars->integralFBy = 0.0f;
            mahoney_vars->integralFBz = 0.0f;
        }

        /* Apply proportional feedback */
        gx += mahoney_vars->twoKp * halfex;
        gy += mahoney_vars->twoKp * halfey;
        gz += mahoney_vars->twoKp * halfez;
    }

    /* Integrate rate of change of quaternion */
    gx *= (0.5f * mahoney_vars->invSampleFreq); // pre-multiply common factors
    gy *= (0.5f * mahoney_vars->invSampleFreq);
    gz *= (0.5f * mahoney_vars->invSampleFreq);
    qa = mahoney_vars->q0;
    qb = mahoney_vars->q1;
    qc = mahoney_vars->q2;
    mahoney_vars->q0 += (-qb * gx - qc * gy - mahoney_vars->q3 * gz);
    mahoney_vars->q1 += (qa * gx + qc * gz - mahoney_vars->q3 * gy);
    mahoney_vars->q2 += (qa * gy - qb * gz + mahoney_vars->q3 * gx);
    mahoney_vars->q3 += (qa * gz + qb * gy - qc * gx);

    /* Normalise quaternion */
    recipNorm = inv_sqrt(mahoney_vars->q0 * mahoney_vars->q0 +
                         mahoney_vars->q1 * mahoney_vars->q1 +
                         mahoney_vars->q2 * mahoney_vars->q2 +
                         mahoney_vars->q3 * mahoney_vars->q3);

    mahoney_vars->q0 *= recipNorm;
    mahoney_vars->q1 *= recipNorm;
    mahoney_vars->q2 *= recipNorm;
    mahoney_vars->q3 *= recipNorm;
    mahoney_vars->anglesComputed = 0;
}

//-------------------------------------------------------------------------------------------

/** @brief NOTE: requires deg/sec for gyro
 *               This algorithm needs a few seconds to converge to stable measurements before being reliable.
 *               (just like Madgewick)
 */
void imu_mahony_update(mahonyVariables_st *mahoney_vars, float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    /* Convert gyroscope degrees/sec to radians/sec */
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    /* Compute feedback only if accelerometer measurement valid
     * (avoids NaN in accelerometer normalisation)
     */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        /* Normalise accelerometer measurement */
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* Estimated direction of gravity */
        halfvx = mahoney_vars->q1 * mahoney_vars->q3 - mahoney_vars->q0 * mahoney_vars->q2;
        halfvy = mahoney_vars->q0 * mahoney_vars->q1 + mahoney_vars->q2 * mahoney_vars->q3;
        halfvz = mahoney_vars->q0 * mahoney_vars->q0 - 0.5f + mahoney_vars->q3 * mahoney_vars->q3;

        /* Error is sum of cross product between estimated
         * and measured direction of gravity
         */
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        /* Compute and apply integral feedback if enabled */
        if (mahoney_vars->twoKi > 0.0f)
        {
            // integral error scaled by Ki
            mahoney_vars->integralFBx += mahoney_vars->twoKi * halfex * mahoney_vars->invSampleFreq;
            mahoney_vars->integralFBy += mahoney_vars->twoKi * halfey * mahoney_vars->invSampleFreq;
            mahoney_vars->integralFBz += mahoney_vars->twoKi * halfez * mahoney_vars->invSampleFreq;
            gx += mahoney_vars->integralFBx; // apply integral feedback
            gy += mahoney_vars->integralFBy;
            gz += mahoney_vars->integralFBz;
        }
        else
        {
            mahoney_vars->integralFBx = 0.0f; // prevent integral windup
            mahoney_vars->integralFBy = 0.0f;
            mahoney_vars->integralFBz = 0.0f;
        }

        /* Apply proportional feedback */
        gx += mahoney_vars->twoKp * halfex;
        gy += mahoney_vars->twoKp * halfey;
        gz += mahoney_vars->twoKp * halfez;
    }

    /* Integrate rate of change of quaternion */
    gx *= (0.5f * mahoney_vars->invSampleFreq); // pre-multiply common factors
    gy *= (0.5f * mahoney_vars->invSampleFreq);
    gz *= (0.5f * mahoney_vars->invSampleFreq);
    qa = mahoney_vars->q0;
    qb = mahoney_vars->q1;
    qc = mahoney_vars->q2;
    mahoney_vars->q0 += (-qb * gx - qc * gy - mahoney_vars->q3 * gz);
    mahoney_vars->q1 += (qa * gx + qc * gz - mahoney_vars->q3 * gy);
    mahoney_vars->q2 += (qa * gy - qb * gz + mahoney_vars->q3 * gx);
    mahoney_vars->q3 += (qa * gz + qb * gy - qc * gx);

    /* Normalise quaternion */
    recipNorm = inv_sqrt(mahoney_vars->q0 * mahoney_vars->q0 +
                         mahoney_vars->q1 * mahoney_vars->q1 +
                         mahoney_vars->q2 * mahoney_vars->q2 +
                         mahoney_vars->q3 * mahoney_vars->q3);

    mahoney_vars->q0 *= recipNorm;
    mahoney_vars->q1 *= recipNorm;
    mahoney_vars->q2 *= recipNorm;
    mahoney_vars->q3 *= recipNorm;
    mahoney_vars->anglesComputed = 0;
}

//-------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

//-------------------------------------------------------------------------------------------

static void compute_angles(mahonyVariables_st *mahoney_vars)
{
    mahoney_vars->roll = atan2f(mahoney_vars->q0 * mahoney_vars->q1 +
                                    mahoney_vars->q2 * mahoney_vars->q3,
                                0.5f - mahoney_vars->q1 * mahoney_vars->q1 -
                                    mahoney_vars->q2 * mahoney_vars->q2);

    mahoney_vars->pitch = asinf(-2.0f * (mahoney_vars->q1 * mahoney_vars->q3 -
                                         mahoney_vars->q0 * mahoney_vars->q2));

    mahoney_vars->yaw = atan2f(mahoney_vars->q1 * mahoney_vars->q2 +
                                   mahoney_vars->q0 * mahoney_vars->q3,
                               0.5f - mahoney_vars->q2 * mahoney_vars->q2 -
                                   mahoney_vars->q3 * mahoney_vars->q3);

    mahoney_vars->anglesComputed = 1;
}

float imu_mahony_get_roll(mahonyVariables_st *mahoney_vars)
{
    if (!mahoney_vars->anglesComputed)
        compute_angles(mahoney_vars);
    return mahoney_vars->roll * 57.29578f;
}

float imu_mahony_get_pitch(mahonyVariables_st *mahoney_vars)
{
    if (!mahoney_vars->anglesComputed)
        compute_angles(mahoney_vars);
    return mahoney_vars->pitch * 57.29578f;
}

float imu_mahony_get_yaw(mahonyVariables_st *mahoney_vars)
{
    if (!mahoney_vars->anglesComputed)
        compute_angles(mahoney_vars);
    return mahoney_vars->yaw * 57.29578f + 180.0f;
}

float imu_mahony_get_roll_radians(mahonyVariables_st *mahoney_vars)
{
    if (!mahoney_vars->anglesComputed)
        compute_angles(mahoney_vars);
    return mahoney_vars->roll;
}

float imu_mahony_get_pitch_radians(mahonyVariables_st *mahoney_vars)
{
    if (!mahoney_vars->anglesComputed)
        compute_angles(mahoney_vars);
    return mahoney_vars->pitch;
}

float imu_mahony_get_yaw_radians(mahonyVariables_st *mahoney_vars)
{
    if (!mahoney_vars->anglesComputed)
        compute_angles(mahoney_vars);
    return mahoney_vars->yaw;
}
