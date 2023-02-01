/**
 *	@file lcd_screen_menu.h
 *	@author SurgeExperiments
 *
 *	@brief This file contains the prototypes for using the lcd screen meny
 */

#ifndef SCREENMENU_H_
#define SCREENMENU_H_

#include "structs.h"

/* LCD display functions */
void lcd_menu_intro(lcdVariables_st *lcd_vars, char **intro_string);
void lcd_menu_print_main(lcdVariables_st *lcd_vars, char **menu_strings, int menu_index);
void lcd_menu_print_pid(lcdVariables_st *lcd_vars, char **pid_menu, int menu_index);

void lcd_menu_print_flight_modes(lcdVariables_st *lcd_vars,
                                 char **level_modes,
                                 char **levelling_menu,
                                 int menu_index);

/* Menu-logic */
void lcd_menu_run_main(lcdVariables_st *lcd_vars,
                       float pid_menu_values[][4],
                       menuStrings_st *menu_data,
                       uint8_t *level_mode_index,
                       buttonHandling_st *button_values);

void lcd_menu_run_pid_settings(lcdVariables_st *lcd_vars,
                               float pid_menu_values[][4],
                               menuStrings_st *menu_data,
                               buttonHandling_st *button_values);

void lcd_menu_run_flight_modes(lcdVariables_st *lcd_vars,
                               menuStrings_st *menu_data,
                               uint8_t *level_mode_index,
                               buttonHandling_st *button_values);

#endif /* SCREENMENU_H_ */
