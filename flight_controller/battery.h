
#ifndef BATTERY_H_
#define BATTERY_H_

#endif /* BATTERY_H_ */

#include "../arm_drivers/adc_driver.h"

//The battery voltage is needed for compensation.
// TODO: Adjust filter values due to a smaller loop_time
#define BATTERY_COMPENSATE(battery_voltage) (battery_voltage * 0.92 + (analog_read(0) + 65) * 0.09853)
#define BATTERY_ALERT(battery_voltage)		if(battery_voltage < 1050 && battery_voltage > 600) LED_ON

#include "led.h"


