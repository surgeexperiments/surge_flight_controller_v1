/**
 *  @file esc.c
 *  @author SurgeExperiments
 *
 *  @brief This file contains the functions for handling the ESC's.
 */

#include "../arm_drivers/pwm_gen.h"
#include "esc.h"

/**
 * @brief This function sets up PWM generation for ESC's on Timer2.
 *        Timer2 will b set to the right GPIO pins, for info check PWM_gen.c
 */
void esc_init_hardware_pwm_gen(void)
{
    enable_pwm_timer_2(49, 2400);
}

/**
 * @author SurgeExperiments
 * @brief Function that adds the extra "push" to the engines based on the current battery voltage.
 *
 * @param[out] esc_motor_output ptr to the esc output struct we want to modify
 * @param[in] current_battery_voltage LOL
 */
static void esc_pwm_voltage_comp(escOutput_st *esc_motor_output, int current_battery_voltage)
{
    float voltage_compensation = (1240 - current_battery_voltage) / (float)3500;
    esc_motor_output->one += esc_motor_output->one * voltage_compensation;
    esc_motor_output->two += esc_motor_output->two * voltage_compensation;
    esc_motor_output->three += esc_motor_output->three * voltage_compensation;
    esc_motor_output->four += esc_motor_output->four * voltage_compensation;
}

/**
 * @author SurgeExperiments
 * @brief Computes the length of the PWM pulses that will be sent to the ESC's.
 *
 * @param[in] mode one of the fly-modes described in structs.h
 * @param[in] use_voltage_compensation set to 1 if voltage compensation is to be used,
 *                                     which means the PWM is boosted a bit when the
 *                                     voltage is lower to obtain the same motor
 *                                     output as with higher voltages.
 *
 *
 * @param[in] throttle the value of the throttle (usually from the RX control): 1000-2000
 * @param[in] battery_voltage self explanatory
 * @param[in] pid_output_data data from the PID controller, the low level one
 *                            that operates on gyro data
 *
 * @param[out] esc_output pointer to the struct that will contain the
 *                        computed pulse-lengths to be sent to the ESC's
 */
void esc_calculate_pulse_length(const flyMode_et mode,
                                uint8_t use_voltage_compensation,
                                int throttle,
                                const int battery_voltage,
                                const pidData_st pid_output_data,
                                escOutput_st *esc_output)
{
    if (mode == MOTORS_FLY_MODE)
    {
        /*
         * Trick: some room to get control @ full throttle:
         * standard method used everywhere (or the quad behaves weird)
         */
        if (throttle > 1850)
            throttle = 1850;
        esc_output->one = throttle - pid_output_data.pitch + pid_output_data.roll - pid_output_data.yaw;
        esc_output->two = throttle + pid_output_data.pitch + pid_output_data.roll + pid_output_data.yaw;
        esc_output->three = throttle + pid_output_data.pitch - pid_output_data.roll - pid_output_data.yaw;
        esc_output->four = throttle - pid_output_data.pitch - pid_output_data.roll + pid_output_data.yaw;

        if (use_voltage_compensation == 1)
        {
            /* Add a little extra to the engine output when voltage drops.  */
            if ((battery_voltage < ESC_VOLTAGE_COMPENSATION_MINIMUM) && (battery_voltage > 800))
            {
                esc_pwm_voltage_comp(esc_output, battery_voltage);
            }
        }

        /* Avoid pulse going outside cutoff ranges */
        if (esc_output->one < ESC_RUNNING_MIN_PULSE)
            esc_output->one = ESC_RUNNING_MIN_PULSE;
        else if (esc_output->one > ESC_RUNNING_MAX_PULSE)
            esc_output->one = ESC_RUNNING_MAX_PULSE;

        if (esc_output->two < ESC_RUNNING_MIN_PULSE)
            esc_output->two = ESC_RUNNING_MIN_PULSE;
        else if (esc_output->two > ESC_RUNNING_MAX_PULSE)
            esc_output->two = ESC_RUNNING_MAX_PULSE;

        if (esc_output->three < ESC_RUNNING_MIN_PULSE)
            esc_output->three = ESC_RUNNING_MIN_PULSE;
        else if (esc_output->three > ESC_RUNNING_MAX_PULSE)
            esc_output->three = ESC_RUNNING_MAX_PULSE;

        if (esc_output->four < ESC_RUNNING_MIN_PULSE)
            esc_output->four = ESC_RUNNING_MIN_PULSE;
        else if (esc_output->four > ESC_RUNNING_MAX_PULSE)
            esc_output->four = ESC_RUNNING_MAX_PULSE;
    }

    /* Mode: MOTORS_STANDBY_MODE or MOTORS_STOP_MODE */
    else
    {
        esc_output->one = ESC_KEEPALIVE_PULSE;
        esc_output->two = ESC_KEEPALIVE_PULSE;
        esc_output->three = ESC_KEEPALIVE_PULSE;
        esc_output->four = ESC_KEEPALIVE_PULSE;
    }
}

/**
 * @author SurgeExperiments
 * @brief The function that sends the computed ESC pulse lengths to the timer generating them
 *
 * IMPORTANT: the timer needs to be set to 1 microsecond per count to get the right pulse lengths!
 *
 * @param TIMx the timer we use to generate the PWM output
 * @param esc_output the struct containing the computed ESC pulses
 */
void esc_generate_hw_pwm_motors(TIM_TypeDef *TIMx, const escOutput_st esc_output)
{
    TIMx->CCR1 = esc_output.one;
    TIMx->CCR2 = esc_output.two;
    TIMx->CCR3 = esc_output.three;
    TIMx->CCR4 = esc_output.four;
}

void esc_set_motors_to_standby_speed(escOutput_st *outputData)
{
    outputData->one = 1000;
    outputData->two = 1000;
    outputData->three = 1000;
    outputData->four = 1000;
}
