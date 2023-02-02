/**
 *	@file pid.C
 *	@author SurgeExperiments
 *
 *  @brief This file contains functions for using the PID controller.
 *
 *  TPA mode:
 *  - A mode that multiplies the P, I and D gains over a certain throttle value by a multiplier.
 *  - This way you can reduce PID gains when doing high throttle, allowing U to use things like a higher
 *    P gain and not get oscillations on high throttle levels
 *
 * TODO: pid_init_controller_structs() has some hard coded angle values.
 *       Move to defines.
 */

#include "pid.h"

/**
 * @brief globals used as a "working space" for the macros in this module.
 */
pidStorage_st g_pid_roll_rotation, g_pid_pitch_rotation, g_pid_yaw_rotation;
pidStorage_st g_pid_roll_angle, g_pid_pitch_angle;

static float current_error;

/**
 * @author SurgeExperiments
 * @brief macro that does PID calculations for angle-mode.
 *
 * @param[out] PID struct of type PID_storage containing PID values. Stores output result.
 *             This param must be a global in this file.
 * @param[in] ACTUAL_ANGLE the angle the drone has for a for an axis
 * @param[in] DESIRED_ANGLE the desired angle the user wants for an axis
 */
#define CALCULATE_PID_STABILIZATION(PID, ACTUAL_ANGLE, DESIRED_ANGLE)             \
    current_error = ACTUAL_ANGLE - DESIRED_ANGLE;                                 \
    PID.proportional = PID.proportionalGain * current_error;                      \
    PID.integral += current_error * PID.integralGain;                             \
    PID.derivative = PID.derivativeGain * (current_error - PID.previousPidError); \
    if (PID.integral > PID.pidMaxOutput)                                          \
        PID.integral = PID.pidMaxOutput;                                          \
    else if (PID.integral < -PID.pidMaxOutput)                                    \
        PID.integral = -PID.pidMaxOutput;                                         \
    PID.output = PID.proportional + PID.integral + PID.derivative;                \
    if (PID.output > PID.pidMaxOutput)                                            \
        PID.output = PID.pidMaxOutput;                                            \
    else if (PID.output < -PID.pidMaxOutput)                                      \
        PID.output = -PID.pidMaxOutput;                                           \
    PID.previousPidError = current_error;

/**
 * @author SurgeExperiments
 * @brief macro that does PID calculations for gyro-mode
 *
 * @param[out] PID struct of type PID_storage containing PID values. Stores output result.
 *             This param must be a global in this file.
 * @param[in] GYRO_DEG_SECOND the actual angular speed the drone has in one axis
 * @param[in] PID_SETPOINT the desired angular speed the user want the drone to have
 */
#define CALCULATE_PID_AXIS(PID, GYRO_DEG_SECOND, PID_SETPOINT)                    \
    current_error = GYRO_DEG_SECOND - PID_SETPOINT;                               \
    PID.proportional = PID.proportionalGain * current_error;                      \
    PID.integral += current_error * PID.integralGain;                             \
    PID.derivative = PID.derivativeGain * (current_error - PID.previousPidError); \
    if (PID.integral > PID.pidMaxOutput)                                          \
        PID.integral = PID.pidMaxOutput;                                          \
    else if (PID.integral < -PID.pidMaxOutput)                                    \
        PID.integral = -PID.pidMaxOutput;                                         \
    PID.output = PID.proportional + PID.integral + PID.derivative;                \
    if (PID.output > PID.pidMaxOutput)                                            \
        PID.output = PID.pidMaxOutput;                                            \
    else if (PID.output < -PID.pidMaxOutput)                                      \
        PID.output = -PID.pidMaxOutput;                                           \
    PID.previousPidError = current_error;

/**
 * @author SurgeExperiments
 * @brief macro that does PID calculations for gyro-mode with TPA
 *
 * NOTE: this might b combined with the CALCULATE_PID_AXIS() later
 *
 * @param[out] PID struct of type PID_storage containing PID values. Stores output result.
 *             This param must be a global in this file.
 * @param[in] GYRO_DEG_SECOND the actual angular speed the drone has in one axis
 * @param[in] PID_SETPOINT the desired angular speed the user want the drone to have
 * @param[in] TPA_MULTIPLIER the multiplier to multiply the gains with, set by the user, and
 *            usually for high throttle values
 */
#define CALCULATE_PID_AXIS_TPA(PID, GYRO_DEG_SECOND, PID_SETPOINT, TPA_MULTIPLIER)                   \
    current_error = GYRO_DEG_SECOND - PID_SETPOINT;                                                  \
    PID.proportional = (PID.proportionalGain * TPA_MULTIPLIER) * current_error;                      \
    PID.integral += current_error * (PID.integralGain * TPA_MULTIPLIER);                             \
    PID.derivative = (PID.derivativeGain * TPA_MULTIPLIER) * (current_error - PID.previousPidError); \
    if (PID.integral > PID.pidMaxOutput)                                                             \
        PID.integral = PID.pidMaxOutput;                                                             \
    else if (PID.integral < -PID.pidMaxOutput)                                                       \
        PID.integral = -PID.pidMaxOutput;                                                            \
    PID.output = PID.proportional + PID.integral + PID.derivative;                                   \
    if (PID.output > PID.pidMaxOutput)                                                               \
        PID.output = PID.pidMaxOutput;                                                               \
    else if (PID.output < -PID.pidMaxOutput)                                                         \
        PID.output = -PID.pidMaxOutput;                                                              \
    PID.previousPidError = current_error;

/**
 * @author SurgeExperiments
 * @brief function to init the structs.
 *
 * This method is a bit clumsy, but good enough for prototyping.
 *
 * @param[out] roll PID-struct for roll gyro
 * @param[out] pitch PID-struct for pitch gyro
 * @param[out] yaw PID-struct for yaw gyro
 * @param[out] roll_angle PID-struct for roll angle
 * @param[out] pitch_angle PID-struct for pitch angle
 */
void pid_init_controller_structs(pidStorage_st *roll,
                                 pidStorage_st *pitch,
                                 pidStorage_st *yaw,
                                 pidStorage_st *roll_angle,
                                 pidStorage_st *pitch_angle)
{
    roll->proportional = 0;
    roll->integral = 0;
    roll->derivative = 0;
    roll->previousPidError = 0;
    roll->output = 0;
    roll->proportionalGain = ROLL_PID_P_GAIN;
    roll->integralGain = ROLL_PID_I_GAIN;
    roll->derivativeGain = ROLL_PID_D_GAIN;
    roll->pidMaxOutput = ROLL_PID_MAX_OUTPUT;

    pitch->proportional = 0;
    pitch->integral = 0;
    pitch->derivative = 0;
    pitch->previousPidError = 0;
    pitch->output = 0;
    pitch->proportionalGain = PITCH_PID_P_GAIN;
    pitch->integralGain = PITCH_PID_I_GAIN;
    pitch->derivativeGain = PITCH_PID_D_GAIN;
    pitch->pidMaxOutput = PITCH_PID_MAX_OUTPUT;

    yaw->proportional = 0;
    yaw->integral = 0;
    yaw->derivative = 0;
    yaw->previousPidError = 0;
    yaw->output = 0;
    yaw->proportionalGain = YAW_PID_P_GAIN;
    yaw->integralGain = YAW_PID_I_GAIN;
    yaw->derivativeGain = YAW_PID_D_GAIN;
    yaw->pidMaxOutput = YAW_PID_MAX_OUTPUT;

    roll_angle->proportional = 0;
    roll_angle->integral = 0;
    roll_angle->derivative = 0;
    roll_angle->previousPidError = 0;
    roll_angle->output = 0;
    roll_angle->proportionalGain = 2.5; // Worked well with kalman -1.5, P gain 1.5, d gain 2
    roll_angle->integralGain = 0.0;
    roll_angle->derivativeGain = 0.0;
    roll_angle->pidMaxOutput = 100;

    pitch_angle->proportional = 0;
    pitch_angle->integral = 0;
    pitch_angle->derivative = 0;
    pitch_angle->previousPidError = 0;
    pitch_angle->output = 0;
    pitch_angle->proportionalGain = 2.5;
    pitch_angle->integralGain = 0.0;
    pitch_angle->derivativeGain = 0.0;
    pitch_angle->pidMaxOutput = 100;
}

/**
 * @author SurgeExperiments
 * @brief simple function that resets the global PID controller structs.
 *
 * Very useful if the quad has landed on the ground and you don't want old PID
 * values to mess up another takeoff.
 *
 * TODO: Set this to auto-trigger upon any landing, even in acro mode
 *       (this does trigger on a controller-reset, but that can be forgotten.
 *       Use a GPIO triggered interrupt to force this)
 */
void pid_reset(void)
{
    g_pid_roll_rotation.integral = 0;
    g_pid_roll_rotation.previousPidError = 0;
    g_pid_pitch_rotation.integral = 0;
    g_pid_pitch_rotation.previousPidError = 0;
    g_pid_yaw_rotation.integral = 0;
    g_pid_yaw_rotation.previousPidError = 0;

    g_pid_roll_angle.integral = 0;
    g_pid_roll_angle.previousPidError = 0;

    g_pid_pitch_angle.integral = 0;
    g_pid_pitch_angle.previousPidError = 0;
}

/***********************
 *   PID CALCULATIONS:  *
 *					   *
 ***********************/

/**
 * @author SurgeExperiments
 * @brief function that transforms PWM from the RX receiver [1000-2000] to
 *        desired angular speed per second
 *
 * @param[in] RX_pulse a struct of type RX_channels that contains the current pulse length from the 4 RX channels
 * @param[out] PID_data struct that contains the desired angular speed/s for pitch, roll and yaw
 */
void rx_signals_to_rot_deg_seconds(pidData_st *pid_setpoints, rxChannels_st rx_pulse)
{
    /* set to 0 just in case  RX_PULSE_DEADBAND_MIN < RX_pulse.n < RX_PULSE_DEADBAND_POS  */
    pid_setpoints->roll = 0;
    if (rx_pulse.one > RX_PULSE_DEADBAND_POS)
    {
        pid_setpoints->roll = (rx_pulse.one - RX_PULSE_DEADBAND_POS) / RX_PULSE_DIVIDER;
    }
    else if (rx_pulse.one < RX_PULSE_DEADBAND_MIN)
    {
        pid_setpoints->roll = (rx_pulse.one - RX_PULSE_DEADBAND_MIN) / RX_PULSE_DIVIDER;
    }

    pid_setpoints->pitch = 0;
    if (rx_pulse.two > RX_PULSE_DEADBAND_POS)
    {
        pid_setpoints->pitch = (rx_pulse.two - RX_PULSE_DEADBAND_POS) / RX_PULSE_DIVIDER;
    }
    else if (rx_pulse.two < RX_PULSE_DEADBAND_MIN)
    {
        pid_setpoints->pitch = (rx_pulse.two - RX_PULSE_DEADBAND_MIN) / RX_PULSE_DIVIDER;
    }

    pid_setpoints->yaw = 0;
    if (rx_pulse.three > RX_PULSE_YAWSTOP)
    {
        if (rx_pulse.four > RX_PULSE_DEADBAND_POS)
        {
            pid_setpoints->yaw = (rx_pulse.four - RX_PULSE_DEADBAND_POS) / RX_PULSE_DIVIDER;
        }
        else if (rx_pulse.four < RX_PULSE_DEADBAND_MIN)
        {
            pid_setpoints->yaw = (rx_pulse.four - RX_PULSE_DEADBAND_MIN) / RX_PULSE_DIVIDER;
        }
    }
}

/**
 * @author SurgeExperiments
 * @brief function that takes RX controller pulses for pitch and yaw and converts them to desired angle
 *        in degrees for these two axes.
 *
 * @param[out] pid_setpoints ptr to struct that will have its roll and pitch fields set with
 *             the desired angle
 * @param[in] RX_pulse incoming rx pulses, which we transform into desired angles
 */
void rx_signals_to_angles(pidData_st *pid_setpoints, rxChannels_st rx_pulse)
{
    pid_setpoints->roll = 0;
    if (rx_pulse.one > RX_PULSE_DEADBAND_POS)
    {
        pid_setpoints->roll = (rx_pulse.one - RX_PULSE_DEADBAND_POS);
    }
    else if (rx_pulse.one < RX_PULSE_DEADBAND_MIN)
    {
        pid_setpoints->roll = (rx_pulse.one - RX_PULSE_DEADBAND_MIN);
    }
    pid_setpoints->roll = pid_setpoints->roll / RX_PULSE_TO_DEGPOS;

    pid_setpoints->pitch = 0;
    if (rx_pulse.two > RX_PULSE_DEADBAND_POS)
    {
        pid_setpoints->pitch = (rx_pulse.two - RX_PULSE_DEADBAND_POS);
    }
    else if (rx_pulse.two < RX_PULSE_DEADBAND_MIN)
    {
        pid_setpoints->pitch = (rx_pulse.two - RX_PULSE_DEADBAND_MIN);
    }
    pid_setpoints->pitch = pid_setpoints->pitch / RX_PULSE_TO_DEGPOS;

    pid_setpoints->yaw = 0;
    if (rx_pulse.three > RX_PULSE_YAWSTOP)
    {
        if (rx_pulse.four > RX_PULSE_DEADBAND_POS)
        {
            pid_setpoints->yaw = (rx_pulse.four - RX_PULSE_DEADBAND_POS) / RX_PULSE_DIVIDER;
        }
        else if (rx_pulse.four < RX_PULSE_DEADBAND_MIN)
        {
            pid_setpoints->yaw = (rx_pulse.four - RX_PULSE_DEADBAND_MIN) / RX_PULSE_DIVIDER;
        }
    }
}

/**
 * @author SurgeExperiments
 * @brief function that outputs the TPA multiplier based on the throttle PWM value from
 *        the RX receiver.
 *
 * @param current_throttle_pwm_value uint32_t in range [1000, 2000]
 */
static float get_tpa_multiplier(uint32_t current_throttle_pwm_value)
{
    if (current_throttle_pwm_value >= TPA_THROTTLE_MIN_CUTOFF_PWM)
    {
        return TPA_PID_COEFFICIENT;
    }
    else
    {
        return 1;
    }
}

/**
 * @author SurgeExperiments
 * @brief function that calculates the PID output values for the angle mode
 *
 * @param[in] actual_pitch_angle the actual pitch angle of the drone relative to the ground.
 * @param[in] actual_roll_angle the actual roll angle of the drone relative to the ground
 * @param[out] pid_setpoints struct that gets filled with the output from the PID controller
 *             These values will b translated into desired rotation and applied to the gyro-based
 *             PID controller later.
 *
 */
void pid_calculations_angle(float actual_pitch_angle, float actual_roll_angle, pidData_st *pid_setpoints)
{
    CALCULATE_PID_AXIS(g_pid_roll_angle, actual_roll_angle, pid_setpoints->roll);
    CALCULATE_PID_AXIS(g_pid_pitch_angle, actual_pitch_angle, pid_setpoints->pitch);

    pid_setpoints->roll = g_pid_roll_angle.output;   //*PID_ANGLEERROR_TO_ROTATION_DEGS;
    pid_setpoints->pitch = g_pid_pitch_angle.output; //*PID_ANGLEERROR_TO_ROTATION_DEGS;
}

/**
 * This requires 500_DPS setting for the gyro
 * The LOOP_QuadDesiredRotationalSpeed3axes sets the desired rotational movement and
 * LOOP_gyroDataFilteredToDegSeconds is the actual movement in deg/s.
 */
void run_pid_calculations(const gyroData_st gyro_deg_second,
                          pidData_st pid_setpoints,
                          pidData_st *pid_output_data,
                          uint32_t throttle_pwm_value,
                          uint8_t use_tpa)
{
    if (use_tpa == 1)
    {
        pid_calculations_rotation_tpa(gyro_deg_second,
                                      pid_setpoints,
                                      pid_output_data,
                                      throttle_pwm_value);
    }
    else
    {
        pid_calculations_rotation(gyro_deg_second, pid_setpoints, pid_output_data);
    }
}

/**
 * @author SurgeExperiments
 * @brief Low level PID controller that operates on desired
 *        rotation and actual rotation (from gyro). Angle mode is eventually
 *        translated down to this PID controller.
 *
 * @param[in] gyro_deg_second angular speed in deg/s in pitch, roll and yaw
 * @param[in] pid_setpoints desired angluar speed in deg/s in put, roll and yaw
 * @param[out] pid_output_data output from the PID controller, will be used to compute speed per engine
 */
void pid_calculations_rotation(const gyroData_st gyro_deg_second, pidData_st pid_setpoints, pidData_st *pid_output_data)
{
    CALCULATE_PID_AXIS(g_pid_roll_rotation, gyro_deg_second.roll, pid_setpoints.roll);
    CALCULATE_PID_AXIS(g_pid_pitch_rotation, gyro_deg_second.pitch, pid_setpoints.pitch);
    CALCULATE_PID_AXIS(g_pid_yaw_rotation, gyro_deg_second.yaw, pid_setpoints.yaw);
    pid_output_data->roll = g_pid_roll_rotation.output;
    pid_output_data->pitch = g_pid_pitch_rotation.output;
    pid_output_data->yaw = g_pid_yaw_rotation.output;
}

/**
 * @author SurgeExperiments
 * @brief Low level PID controller that operates on desired rotation and actual rotation (from gyro). Similar to
 *        PID_calculationsRotation except it employs TPA for high throttle values. This tapers off the
 *        PID gains when the throttle is high, improving performance for acro flights (with correct PID tunings).
 *
 * Remember to tune the constants used in get_tpa_multiplier() for any new drone frame.
 *
 * @param[in] gyro_deg_second angular speed in deg/s in pitch, roll and yaw
 * @param[in] pid_setpoints desired angluar speed in deg/s in put, roll and yaw
 * @param[out] pid_output_data output from the PID controller, will be used to compute speed per engine
 */
void pid_calculations_rotation_tpa(const gyroData_st gyro_deg_second,
                                   pidData_st pid_setpoints,
                                   pidData_st *pid_output_data,
                                   uint32_t throttle_pwm_value)
{
    /* PID_roll, PID_pitch and PID_yaw are globals (for now) */
    float tpa_multiplier = get_tpa_multiplier(throttle_pwm_value);
    CALCULATE_PID_AXIS_TPA(g_pid_roll_rotation, gyro_deg_second.roll, pid_setpoints.roll, tpa_multiplier);
    CALCULATE_PID_AXIS_TPA(g_pid_pitch_rotation, gyro_deg_second.pitch, pid_setpoints.pitch, tpa_multiplier);
    /* We do not use the TPA constant for yaw, only pitch and roll*/
    CALCULATE_PID_AXIS_TPA(g_pid_yaw_rotation, gyro_deg_second.yaw, pid_setpoints.yaw, 1);
    pid_output_data->roll = g_pid_roll_rotation.output;
    pid_output_data->pitch = g_pid_pitch_rotation.output;
    pid_output_data->yaw = g_pid_yaw_rotation.output;
}
