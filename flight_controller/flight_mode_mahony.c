
#include "flight_mode_mahony.h"
#include "imu.h"
#include "pid.h"
#include "running_mode.h"

/**
 * @author SurgeExperiments
 * @brief This is an earlier version that needs updating.
 *        TODO: There are errors in here that causes flight instability.
 *              Add existing updates.
 */
void flight_mode_mahony(imuOrientation_st *angle_orientation_data,
                        mahonyVariables_st *mahoney_vars,
                        pidData_st *desired_rot_speed_3_axes,
                        flyMode_et *flight_mode,
                        imuDataCollection_st imu_realtime_data,
                        rxChannels_st rx_channel_pulses)
{
    int i;
    for (i = 0; i < 15; i++)
    {
        imu_mahony_update(mahoney_vars,
                          imu_realtime_data.gyroDataFilteredToDegSeconds.roll,
                          imu_realtime_data.gyroDataFilteredToDegSeconds.pitch,
                          imu_realtime_data.gyroDataFilteredToDegSeconds.yaw,
                          imu_realtime_data.accelDataFiltered.roll,
                          imu_realtime_data.accelDataFiltered.pitch,
                          imu_realtime_data.accelDataFiltered.z);
    }

    angle_orientation_data->rollAngleAccel = imu_mahony_get_roll(mahoney_vars);
    angle_orientation_data->pitchAngleAccel = imu_mahony_get_pitch(mahoney_vars);

    if (angle_orientation_data->rollAngleAccel < 0)
        angle_orientation_data->rollAngleAccel += 180;
    else
        angle_orientation_data->rollAngleAccel -= 180;

    angle_orientation_data->rollAngleAccel *= -1;

    set_running_mode(flight_mode, rx_channel_pulses);

    /* This loads desired quad angle into roll and pitch and desired rotation deg/s into yaw
     * (since we do not yet use auto-levelling for yaw)
     */
    rx_signals_to_angles(desired_rot_speed_3_axes, rx_channel_pulses);
    pid_calculations_angle((-1) * angle_orientation_data->pitchAngleAccel,
                           (-1) * angle_orientation_data->rollAngleAccel,
                           desired_rot_speed_3_axes);
}