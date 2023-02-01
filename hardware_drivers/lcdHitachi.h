/**
 * @author SurgeExperiments
 */ 


#ifndef LCDHITACHI_H_
#define LCDHITACHI_H_

#include <inttypes.h>
#include <stdlib.h>
#include "../flight_controller/structs.h"

// Addr
#define WRITE_ADDR 0x4E

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAY_CONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

void lcd_begin(lcdVariables_st *lcd_vars,
               uint8_t i2c_write_addr,
               uint8_t initialize_i2c,
               uint8_t cols,
               uint8_t lines);

void lcd_print_string(lcdVariables_st *lcd_vars, char *my_string);
void lcd_clear(lcdVariables_st *lcd_vars);
void lcd_home(lcdVariables_st *lcd_vars);

void lcd_no_display(lcdVariables_st *lcd_vars);
void lcd_display(lcdVariables_st *lcd_vars);
void lcd_no_blink(lcdVariables_st *lcd_vars);
void lcd_blink(lcdVariables_st *lcd_vars);
void lcd_no_cursor(lcdVariables_st *lcd_vars);
void lcd_cursor(lcdVariables_st *lcd_vars);
void lcd_scroll_display_left(lcdVariables_st *lcd_vars);
void lcd_scroll_display_right(lcdVariables_st *lcd_vars);
void lcd_left_to_right(lcdVariables_st *lcd_vars);
void lcd_right_to_left(lcdVariables_st *lcd_vars);
void lcd_autoscroll(lcdVariables_st *lcd_vars);
void lcd_no_autoscroll(lcdVariables_st *lcd_vars);
void lcd_set_backlight(lcdVariables_st *lcd_vars, uint8_t brightness);
void lcd_set_cursor(lcdVariables_st *lcd_vars, uint8_t col, uint8_t row);

#endif /* LCDHITACHI_H_ */

