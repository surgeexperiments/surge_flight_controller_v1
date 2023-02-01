/**
 * This file contains convenience functions you can
 * use to do debug prints on demand.
 */

#include <stdio.h>
#include "debug_tools.h"
#include "../arm_drivers/dbg_swo_driver.h"
#include "led.h"

/* MACROS for debugging */
#define NUMBER2ASCIIBUFFER_SIZE 100
static char num_to_ascii_buffer[NUMBER2ASCIIBUFFER_SIZE];

/**
 * @author SurgeExperiments
 * @brief flicker led every flicker_mod loop iterations
 */
void flicker_led(uint16_t *loop_iteration_count, uint8_t flicker_mod)
{
    *loop_iteration_count += 1;
    if (*loop_iteration_count % flicker_mod == 0)
    {
        LED_TOGGLE_ALL;
        *loop_iteration_count = 1;
    }
}

void print_imu_data(gyroData_st gyro_data, accelData_st accel_data)
{
    sprintf(num_to_ascii_buffer,
            "%d\t%d\t%d\t%d\t%d\t%d\n",
            (int)gyro_data.roll,
            (int)gyro_data.pitch,
            (int)gyro_data.yaw,
            (int)accel_data.pitch,
            (int)accel_data.roll,
            (int)accel_data.z);
    print_str_to_dbg_port(num_to_ascii_buffer);
}

void print_esc_output(escOutput_st esc_output)
{
    sprintf(num_to_ascii_buffer, "%u\t", esc_output.one);
    print_str_to_dbg_port(num_to_ascii_buffer);
    sprintf(num_to_ascii_buffer, "%u\t", esc_output.two);
    print_str_to_dbg_port(num_to_ascii_buffer);
    sprintf(num_to_ascii_buffer, "%u\t", esc_output.three);
    print_str_to_dbg_port(num_to_ascii_buffer);
    sprintf(num_to_ascii_buffer, "%u\n", esc_output.four);
    print_str_to_dbg_port(num_to_ascii_buffer);
}

void print_rx_signals(rxChannels_st rx_channel_pulses)
{
    sprintf(num_to_ascii_buffer, "%u\t%u\t%u\t%u\n",
            (unsigned int)rx_channel_pulses.one,
            (unsigned int)rx_channel_pulses.two,
            (unsigned int)rx_channel_pulses.three,
            (unsigned int)rx_channel_pulses.four);
    print_str_to_dbg_port(num_to_ascii_buffer);
}
