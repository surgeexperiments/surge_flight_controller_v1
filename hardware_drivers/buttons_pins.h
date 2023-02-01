
#ifndef BUTTONSPINS_H_
#define BUTTONSPINS_H_

#include "../arm_drivers/gpio_driver.h"
#include "../flight_controller/structs.h"

void button_pins_setup(void);
void button_handle_presses(buttonHandling_st *button_values);

#endif /* BUTTONSPINS_H_ */

