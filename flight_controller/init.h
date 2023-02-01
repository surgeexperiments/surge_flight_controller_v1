

#ifndef INIT_H_
#define INIT_H_

#include "enums.h"
#include "structs.h"


void init_additional_fly_modes(flightModeSelector_et levelling_mode,
                               imuDataCollection_st *IMU_realtimeData,
                               kalmanVariables_st *LOOP_kalmanVariablesX,
                               kalmanVariables_st *LOOP_kalmanVariablesY,
                               mahonyVariables_st *LOOP_myMahoneyVariables);


#endif /* INIT_H_ */
