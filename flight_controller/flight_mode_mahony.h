


#ifndef FLIGHT_MODE_MAHONY_H_
#define FLIGHT_MODE_MAHONY_H_

#include "enums.h"
#include "structs.h"

void flight_mode_mahony(imuOrientation_st *angle_orientation_data,
                        mahonyVariables_st *mahoney_vars,
                        pidData_st *desired_rot_speed_3_axes,
                        flyMode_et *flight_mode,
                        imuDataCollection_st imu_realtime_data,
                        rxChannels_st rx_channel_pulses);

#endif /* AUTOPILOT_H_ */