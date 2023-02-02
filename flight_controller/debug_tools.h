/**
 * @brief This is a very basic debug method. All vars are
 *        titled the same as in main.h/c and you can
 *        import this file to activate debug macros.
 *
 *        (since this relies on var naming it's quite
 *        fragile, but for its intended transient
 *        purpose it's ok)
 *
 *        NOTE: remember to activate the right defines to use macros.
 */

#ifndef DEBUG_MACROS_FLIGHT_LOOP_H_
#define DEBUG_MACROS_FLIGHT_LOOP_H_

#include "enums.h"
#include "structs.h"

void flicker_led(uint16_t *loop_iteration_count, uint8_t flicker_mod);
void print_imu_data(gyroData_st gyro_data, accelData_st accel_data);
void print_esc_output(escOutput_st esc_output);
void print_rx_signals(rxChannels_st rx_channel_pulses);

#endif /* DEBUG_MACROS_FLIGHT_LOOP_H_ */
