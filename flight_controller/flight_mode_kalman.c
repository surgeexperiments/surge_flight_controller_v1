
#include "flight_mode_kalman.h"
#include "imu.h"
#include "running_mode.h"
#include "pid.h"

/**
 * @author SurgeExperiments
 * @brief This is an earlier version that needs updating.
 *        TODO: There are errors in here that causes flight instability.
 *              Add existing updates.
 */
void flight_mode_kalman(kalmanVariables_st *kalman_var_x,
                        kalmanVariables_st *kalman_var_y,
                        imuOrientation_st *angle_orientation_data,
                        pidData_st *desired_rot_speed_3_axes,
                        flyMode_et *flight_mode,
                        imuDataCollection_st imu_realtime_data,
                        rxChannels_st rx_channel_pulses,
                        float loop_time_ms)
{
    /* NOTE: we use the loop_time_ms here, it's a good estimate of the sample rate
     * (since it's being called once per loop_time_ms)
     */
    imu_run_kalman_filter(kalman_var_x,
                          kalman_var_y,
                          imu_realtime_data.gyroDataFilteredToDegSeconds,
                          imu_realtime_data.accelDataFiltered,
                          loop_time_ms);

    /*
     * KEY: If there is a lag between actual drone angle and the reported angle, the drone will oscillate between the desired angle
     * 		instead of hitting it exactly: ex actual: 0 deg, kalman think it's 2 deg, give signal for rotation.
     *       So need high responsiveness from the IMU angle estimation algorithms.
     */
    angle_orientation_data->rollAngleAccel =
        angle_orientation_data->rollAngleAccel * 0.99 + kalman_var_x->finalAngle * 0.01;

    angle_orientation_data->pitchAngleAccel =
        angle_orientation_data->pitchAngleAccel * 0.99 + kalman_var_y->finalAngle * 0.01;

    set_running_mode(flight_mode, rx_channel_pulses);
    rx_signals_to_angles(desired_rot_speed_3_axes, rx_channel_pulses);

    /* NOTE: the multiplication with -1 is just to get the axises right (since they can b reversed easily
     * DO NOT multiply with more than -1 because the quad will then correct with a scaled angle (and thus over correct)
     * unless the angle is exactly 0 which is extremely rare
     */
    pid_calculations_angle((-1.0) * angle_orientation_data->pitchAngleAccel,
                           (-1.0) * angle_orientation_data->rollAngleAccel,
                           desired_rot_speed_3_axes);
}