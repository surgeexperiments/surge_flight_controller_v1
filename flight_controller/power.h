/**
 *	@file power.h
 *	@author SurgeExperiments
 *
 *	@brief This file contains the macro that returns the battery voltage
 */

/**
 * @brief return the battery voltage to the caller
 *
 * @param battery_voltage a float variable you want to store the battery voltage inside.
 */
#define LOAD_BATTERY_VOLTAGE(battery_voltage) //(battery_voltage = (analog_read(0) + 65) * 1.2317);

