


#ifndef FLIGHT_MODE_KALMAN_H_
#define FLIGHT_MODE_KALMAN_H_

#include "enums.h"
#include "structs.h"


void flight_mode_kalman(kalmanVariables_st *kalman_var_x,
                       kalmanVariables_st *kalman_var_y,
                       imuOrientation_st *angle_orientation_data,
                       pidData_st *desired_rot_speed_3_axes,
                       flyMode_et *flight_mode,
                       imuDataCollection_st imu_realtime_data,
                       rxChannels_st rx_channel_pulses,
                       float loop_time_ms);

#endif /* FLIGHT_MODE_KALMAN_H_ */