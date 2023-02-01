/**
 *	@file pid.h
 *	@author Surge
 *	@date 1/1/2019
 *
 *	@brief This file contains prototypes for the PID functions and defines for setting values
 *		   for PID gains.
 *
 *		   PID settings are adjusted below the function prototypes.
 *
 */

#ifndef PID_H_
#define PID_H_

#include <stdint.h>
#include "structs.h"

void pid_init_controller_structs(pidStorage_st *roll,
                                 pidStorage_st *pitch,
                                 pidStorage_st *yaw,
                                 pidStorage_st *roll_angle,
                                 pidStorage_st *pitch_angle);

void rx_signals_to_rot_deg_seconds(pidData_st *pid_setpoints, rxChannels_st rx_pulse);
void rx_signals_to_angles(pidData_st *pid_setpoints, rxChannels_st rx_pulse);

void pid_calculations_rotation_tpa(const gyroData_st gyro_deg_second,
                                   pidData_st pid_setpoints,
                                   pidData_st *pid_output_data,
                                   uint32_t throttlePWMvalue);

/* Convenience */
void run_pid_calculations(const gyroData_st gyro_deg_second,
                          pidData_st pid_setpoints,
                          pidData_st *pid_output_data,
                          uint32_t throttle_pwm_value,
                          uint8_t use_tpa);

void pid_calculations_angle(float pitch_angle, float roll_angle, pidData_st *pid_setpoints);
void pid_calculations_rotation(const gyroData_st gyro_deg_second,
                               pidData_st pid_setpoints,
                               pidData_st *pid_output_data);

void pid_reset(void);

/* You set which drone you're using here, so you can have different PID values
 * If this val isn't DRONE1, DRONE2 or DRONE3 this the flight controller won't compile
 */
#define DRONE2_AUTOLEVEL

#ifdef DRONE1
#define ROLL_PID_P_GAIN 1.3   // ok w/1.2
#define ROLL_PID_I_GAIN 0.01  // ok w/0.01
#define ROLL_PID_D_GAIN 10.00 // ok/w 3
#define ROLL_PID_MAX_OUTPUT 400

#define PITCH_PID_P_GAIN 1.3
#define PITCH_PID_I_GAIN 0.01 // Put this at 3 and the quad oscillates like crazy
#define PITCH_PID_D_GAIN 10.00
#define PITCH_PID_MAX_OUTPUT 400

#define YAW_PID_P_GAIN 3.0
#define YAW_PID_I_GAIN 0.03
#define YAW_PID_D_GAIN 0.00
#define YAW_PID_MAX_OUTPUT 400

/* The higher these numbers, the less responsive it is
 * Pos>min and a little space == deadband for the controller
 * Current vars give a speed of max 158.5 deg/s
 * Play with these to maximize control
 */

// used to be 3
#define RX_PULSE_DIVIDER (float)4.5
#define RX_PULSE_DEADBAND_POS (float)1520
#define RX_PULSE_DEADBAND_MIN (float)1480
// Too high?
#define RX_PULSE_YAWSTOP (float)1150

// This limits the max +- degree to about 80 degrees
#define RX_PULSE_TO_DEGPOS (float)6
#define PID_ANGLEERROR_TO_ROTATION_DEGS (float)3

/* when the throttle is above TPA_THROTTLE_MIN_CUTOFF_PWM all PID gains are multiplied by TPA_PID_COEFFICIENT */
#define TPA_PID_COEFFICIENT 0.85
#define TPA_THROTTLE_MIN_CUTOFF_PWM 1850
#endif

#ifdef DRONE2_ANGLE
#define ROLL_PID_P_GAIN 3.1
#define ROLL_PID_I_GAIN 0.014
#define ROLL_PID_D_GAIN 30.00
#define ROLL_PID_MAX_OUTPUT 400

#define PITCH_PID_P_GAIN 3.1
#define PITCH_PID_I_GAIN 0.014
#define PITCH_PID_D_GAIN 30.00
#define PITCH_PID_MAX_OUTPUT 400

#define YAW_PID_P_GAIN 6.0
#define YAW_PID_I_GAIN 0.01
#define YAW_PID_D_GAIN 0.00
#define YAW_PID_MAX_OUTPUT 400

// used to be 3
#define RX_PULSE_DIVIDER (float)4.5
#define RX_PULSE_DEADBAND_POS (float)1520
#define RX_PULSE_DEADBAND_MIN (float)1480
// Too high?
#define RX_PULSE_YAWSTOP (float)1150

// This limits the max +- degree to about 80 degrees
#define RX_PULSE_TO_DEGPOS (float)6
#define PID_ANGLEERROR_TO_ROTATION_DEGS (float)3

/* when the throttle is above TPA_THROTTLE_MIN_CUTOFF_PWM all PID gains are multiplied by TPA_PID_COEFFICIENT */
#define TPA_PID_COEFFICIENT 1
#define TPA_THROTTLE_MIN_CUTOFF_PWM 2000
#endif

#ifdef DRONE2_AUTOLEVEL
#define ROLL_PID_P_GAIN 1.5
#define ROLL_PID_I_GAIN 0.01
#define ROLL_PID_D_GAIN 20.00
#define ROLL_PID_MAX_OUTPUT 400

#define PITCH_PID_P_GAIN 1.5
#define PITCH_PID_I_GAIN 0.01
#define PITCH_PID_D_GAIN 20.00
#define PITCH_PID_MAX_OUTPUT 400

#define YAW_PID_P_GAIN 6.0
#define YAW_PID_I_GAIN 0.01
#define YAW_PID_D_GAIN 0.00
#define YAW_PID_MAX_OUTPUT 400

// used to be 3
#define RX_PULSE_DIVIDER (float)4.5
#define RX_PULSE_DEADBAND_POS (float)1520
#define RX_PULSE_DEADBAND_MIN (float)1480
// Too high?
#define RX_PULSE_YAWSTOP (float)1150

// This limits the max +- degree to about 80 degrees
#define RX_PULSE_TO_DEGPOS (float)6
#define PID_ANGLEERROR_TO_ROTATION_DEGS (float)3

/* when the throttle is above TPA_THROTTLE_MIN_CUTOFF_PWM all PID gains are multiplied by TPA_PID_COEFFICIENT */
#define TPA_PID_COEFFICIENT 1
#define TPA_THROTTLE_MIN_CUTOFF_PWM 2000
#endif

/* GOPRO drone */
#ifdef DRONE3
#define ROLL_PID_P_GAIN 1.3
#define ROLL_PID_I_GAIN 0.01
#define ROLL_PID_D_GAIN 10.00
#define ROLL_PID_MAX_OUTPUT 400

#define PITCH_PID_P_GAIN 1.3
#define PITCH_PID_I_GAIN 0.01
#define PITCH_PID_D_GAIN 10.00
#define PITCH_PID_MAX_OUTPUT 400

#define YAW_PID_P_GAIN 3.0
#define YAW_PID_I_GAIN 0.03
#define YAW_PID_D_GAIN 0.00
#define YAW_PID_MAX_OUTPUT 400

// used to be 3
#define RX_PULSE_DIVIDER (float)4.5
#define RX_PULSE_DEADBAND_POS (float)1520
#define RX_PULSE_DEADBAND_MIN (float)1480
// Too high?
#define RX_PULSE_YAWSTOP (float)1150

// This limits the max +- degree to about 80 degrees
#define RX_PULSE_TO_DEGPOS (float)6
#define PID_ANGLEERROR_TO_ROTATION_DEGS (float)3

/* when the throttle is above TPA_THROTTLE_MIN_CUTOFF_PWM all PID gains are multiplied by TPA_PID_COEFFICIENT */
#define TPA_PID_COEFFICIENT 0.85
#define TPA_THROTTLE_MIN_CUTOFF_PWM 1850
#endif

#endif /* PID_H_ */
