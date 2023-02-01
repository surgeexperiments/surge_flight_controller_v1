/**
 *	@file lcd_screen_menu.c
 *	@author SurgeExperiments
 *
 *	@brief This file contains the functions for using the lcd screen menu
 *		   This menu is meant to be hard coded, it will not change much.
 *   	   NOTE: This menu is not finished.
 *		   TODO: Add enums or defines for symbolic names. Make more generic?
 */

#define NUMBEROFLEVELMODES 4

#include <string.h>
#include <stdio.h>

#include "../arm_drivers/dbg_swo_driver.h"
#include "../arm_drivers/timing_driver.h"
#include "../hardware_drivers/buttons_pins.h"
#include "../hardware_drivers/lcdHitachi.h"

#include "lcd_screen_menu.h"

static char print_buffer[30];

void lcd_menu_intro(lcdVariables_st *lcd_vars, char **intro_string)
{

    lcd_clear(lcd_vars);
    lcd_set_cursor(lcd_vars, (uint8_t)0, (uint8_t)0);

    /* Set the LCD cursor to position to position 0,0, flicker text */
    for (int n = 0; n < 5; n++)
    {
        lcd_clear(lcd_vars);
        tim9_delay_ms(100);
        lcd_print_string(lcd_vars, intro_string[n]);
        tim9_delay_ms(500);
    }
    for (int n = 0; n < 10; n++)
    {
        lcd_clear(lcd_vars);
        tim9_delay_ms(50);
        lcd_print_string(lcd_vars, intro_string[4]);
        tim9_delay_ms(50);
    }
    tim9_delay_ms(1000);
}

void lcd_menu_print_main(lcdVariables_st *lcd_vars, char **menu_strings, int menu_index)
{
    lcd_clear(lcd_vars);
    lcd_set_cursor(lcd_vars, (uint8_t)0, (uint8_t)0);
    lcd_print_string(lcd_vars, menu_strings[menu_index]);
}

void lcd_menu_print_pid(lcdVariables_st *lcd_vars, char **pid_menu, int menu_index)
{
    print_str_to_dbg_port(print_buffer);
    lcd_clear(lcd_vars);
    lcd_set_cursor(lcd_vars, (uint8_t)0, (uint8_t)0);
    lcd_print_string(lcd_vars, pid_menu[menu_index]);

    /* Only print number when we are not on menu-item "exit" */
    if (menu_index != 9)
    {
        lcd_set_cursor(lcd_vars, (uint8_t)0, (uint8_t)1);
        lcd_print_string(lcd_vars, print_buffer);
    }
}

void lcd_menu_print_flight_modes(lcdVariables_st *lcd_vars,
                                 char **level_modes,
                                 char **levelling_menu,
                                 int menu_index)
{
    lcd_clear(lcd_vars);
    lcd_set_cursor(lcd_vars, (uint8_t)0, (uint8_t)0);
    lcd_print_string(lcd_vars, levelling_menu[menu_index]);
    lcd_set_cursor(lcd_vars, (uint8_t)0, (uint8_t)1);
    lcd_print_string(lcd_vars, level_modes[menu_index]);
}

void lcd_menu_run_main(lcdVariables_st *lcd_vars,
                       float pid_menu_values[][4],
                       menuStrings_st *menu_data,
                       uint8_t *level_mode_index,
                       buttonHandling_st *button_values)
{

    int menu_index = 0;
    lcd_menu_print_main(lcd_vars, menu_data->mainMenuNames, menu_index);

    while (1)
    {
        button_handle_presses(button_values);

        // Go backwards in the menu
        if (button_values->buttonPressEvent[0] == 1)
        {
            button_values->buttonPressEvent[0] = 0;
            // Decrease menu index and load data
            --menu_index;
            if (menu_index < 0)
            {
                menu_index = 2;
            }
            lcd_menu_print_main(lcd_vars, menu_data->mainMenuNames, menu_index);
        }

        /* Go forwards in the menu */
        if (button_values->buttonPressEvent[1] == 1)
        {
            button_values->buttonPressEvent[1] = 0;
            /* Increase menu index and load data */
            ++menu_index;
            if (menu_index > 2)
            {
                menu_index = 0;
            }
            lcd_menu_print_main(lcd_vars, menu_data->mainMenuNames, menu_index);
        }

        /* Minus and plus means "activate sub-menu" */
        if ((button_values->buttonPressEvent[2] == 1) || (button_values->buttonPressEvent[3] == 1))
        {
            button_values->buttonPressEvent[2] = 0;
            button_values->buttonPressEvent[3] = 0;

            if (menu_index == 0)
            {

                lcd_menu_run_pid_settings(lcd_vars, pid_menu_values, menu_data, button_values);
            }
            else if (menu_index == 1)
            {
                /* This function runs until the user exits the menu */
                lcd_menu_run_flight_modes(lcd_vars, menu_data, level_mode_index, button_values);
            }
            else if (menu_index == 2)
            {
                /* Menu is done, exit this function */
                return;
            }
            lcd_menu_print_main(lcd_vars, menu_data->mainMenuNames, menu_index);
        }
    }
}

void lcd_menu_run_pid_settings(lcdVariables_st *lcd_vars,
                               float pid_menu_values[][4],
                               menuStrings_st *menu_data,
                               buttonHandling_st *button_values)
{
    int menu_index = 0;
    float current_value = pid_menu_values[menu_index][0];
    print_str_to_dbg_port(print_buffer);
    lcd_menu_print_pid(lcd_vars, menu_data->PID_menu, menu_index);

    while (1)
    {
        button_handle_presses(button_values);
        /* Go backwards in the menu */
        if (button_values->buttonPressEvent[0] == 1)
        {
            button_values->buttonPressEvent[0] = 0;
            if ((menu_index != 9) && (current_value != pid_menu_values[menu_index][0]))
            {
                pid_menu_values[menu_index][0] = current_value;
            }
            /* Decrease menu index and load data */
            --menu_index;
            if (menu_index < 0)
            {
                menu_index = 9;
            }
            /* only set current_value when not on the exit menu */
            else
            {
                current_value = pid_menu_values[menu_index][0];
            }
            lcd_menu_print_pid(lcd_vars, menu_data->PID_menu, menu_index);
        }

        /* Go forwards in the menu */
        if (button_values->buttonPressEvent[1] == 1)
        {
            button_values->buttonPressEvent[1] = 0;
            if ((menu_index != 9) && (current_value != pid_menu_values[menu_index][0]))
            {
                pid_menu_values[menu_index][0] = current_value;
            }

            ++menu_index;
            if (menu_index > 9)
            {
                menu_index = 0;
            }
            /* only set current_value when not on the exit menu */
            if (menu_index != 9)
            {
                current_value = pid_menu_values[menu_index][0];
            }
            lcd_menu_print_pid(lcd_vars, menu_data->PID_menu, menu_index);
        }

        /* Minus on variable or exit */
        if (button_values->buttonPressEvent[2] == 1)
        {
            button_values->buttonPressEvent[2] = 0;

            /* If in exit menu, exit and ev write nes */
            if (menu_index == 9)
            {
                return;
            }
            /* Else: Update variable */
            current_value -= pid_menu_values[menu_index][1];
            if (current_value < pid_menu_values[menu_index][2])
            {
                current_value = pid_menu_values[menu_index][2];
            }
            lcd_menu_print_pid(lcd_vars, menu_data->PID_menu, menu_index);
        }

        /* Plus on variable or exit */
        if (button_values->buttonPressEvent[3] == 1)
        {
            button_values->buttonPressEvent[3] = 0;

            /* If in exit menu, exit */
            if (menu_index == 9)
            {
                return;
            }
            /* Else: Update variable */
            current_value += pid_menu_values[menu_index][1];
            if (current_value > pid_menu_values[menu_index][3])
            {
                current_value = pid_menu_values[menu_index][3];
            }

            lcd_menu_print_pid(lcd_vars, menu_data->PID_menu, menu_index);
        }
    }
}

void lcd_menu_run_flight_modes(lcdVariables_st *lcd_vars,
                               menuStrings_st *menu_data,
                               uint8_t *level_mode_index,
                               buttonHandling_st *button_values)
{
    int menu_index = 0;
    uint8_t current_value = *level_mode_index;

    lcd_menu_print_flight_modes(lcd_vars, menu_data->level_modes, menu_data->levelling_menu, menu_index);

    while (1)
    {
        button_handle_presses(button_values);
        /* Go backwards in the menu */
        if (button_values->buttonPressEvent[0] == 1)
        {
            button_values->buttonPressEvent[0] = 0;
            /* If values has changed, store to EEPROM */
            if (menu_index == 0)
            {
                if (current_value != *level_mode_index)
                {
                    // eeprom_write_byte ((uint8_t *)1 , current_value);
                    *level_mode_index = current_value;
                }
            }
            /* Decrease menu index and load data */
            --menu_index;
            if (menu_index < 0)
            {
                menu_index = 1;
            }
            lcd_menu_print_flight_modes(lcd_vars, menu_data->level_modes, menu_data->levelling_menu, menu_index);
        }

        /* Go forwards in the menu */
        if (button_values->buttonPressEvent[1] == 1)
        {
            button_values->buttonPressEvent[1] = 0;
            /* If values has changed, store to EEPROM */
            if (menu_index == 0)
            {
                if (current_value != *level_mode_index)
                {
                    // eeprom_write_byte ((uint8_t *)1 , current_value);
                    *level_mode_index = current_value;
                }
            }
            ++menu_index;
            if (menu_index > 1)
            {
                menu_index = 0;
            }
            lcd_menu_print_flight_modes(lcd_vars, menu_data->level_modes, menu_data->levelling_menu, menu_index);
        }

        if (button_values->buttonPressEvent[2] == 1)
        {
            button_values->buttonPressEvent[2] = 0;
            /* If in exit menu, exit */
            if (menu_index == 1)
            {
                button_values->buttonPressEvent[3] = 0;
                return;
            }
            if (current_value > 0)
            {
                --current_value;
            }
            lcd_menu_print_flight_modes(lcd_vars, menu_data->level_modes, menu_data->levelling_menu, menu_index);
        }

        if (button_values->buttonPressEvent[3] == 1)
        {
            button_values->buttonPressEvent[3] = 0;
            // If in exit menu, exit
            if (menu_index == 1)
            {
                return;
            }
            if (current_value < NUMBEROFLEVELMODES - 1)
            {
                ++current_value;
            }
            // void printFlightModesMenuUSART(char **level_modes, char **levelling_menu, int menuIndex, uint8_t valueIndex)
            lcd_menu_print_flight_modes(lcd_vars, (char **)menu_data->level_modes, menu_data->levelling_menu, menu_index);
        }
    }
}
