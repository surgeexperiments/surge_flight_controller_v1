/**
 * @author SurgeExperiments
 *
 * @brief Ported from Arduino CPP library and then modified.
 */

#include "lcdHitachi.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "../arm_drivers/i2c_driver.h"
#include "../arm_drivers/timing_driver.h"
#include "../arm_drivers/dbg_swo_driver.h"

#define PCF_RS 0x01
#define PCF_RW 0x02
#define PCF_EN 0x04
#define PCF_BACK_LIGHT 0x08

/* Definitions on how the PCF8574 is connected to the LCD */
#define RSMODE_CMD 0
#define RSMODE_DATA 1

/* ----- low level functions ----- */

/* change the PCF8674 pins to the given value */
static void lcd_write_to_wire(lcdVariables_st *lcd_vars, uint8_t half_byte, uint8_t mode, uint8_t enable)
{
    uint8_t i2cData = half_byte << 4;
    if (mode > 0)
        i2cData |= PCF_RS;
    /* PCF_RW is never used. */
    if (enable > 0)
        i2cData |= PCF_EN;
    if (lcd_vars->backLight > 0)
        i2cData |= PCF_BACK_LIGHT;

    if (i2c_start(I2C2, lcd_vars->addr, 0) != 0)
    {
        print_str_to_dbg_port("i2cStartFail\n!");
        return;
    }
    if (i2c_write_byte(I2C2, i2cData) != 0)
    {
        print_str_to_dbg_port("i2cWriteByteFail\n!");
        return;
    }
    if (i2c_stop(I2C2) != 0)
    {
        print_str_to_dbg_port("i2cStopFail\n!");
        return;
    }
}

/* write a nibble / half_byte with handshake */
static void lcd_send_nibble(lcdVariables_st *lcd_vars, uint8_t half_byte, uint8_t mode)
{
    lcd_write_to_wire(lcd_vars, half_byte, mode, 1);
    /* enable pulse must be >450ns */
    tim9_delay_us(1);
    lcd_write_to_wire(lcd_vars, half_byte, mode, 0);
    /* commands need > 37us to settle */
    tim9_delay_us(37);
}

static void lcd_send(lcdVariables_st *lcd_vars, uint8_t value, uint8_t mode)
{
    /* separate the 4 value-nibbles */
    uint8_t valueLo = value & 0x0F;
    uint8_t valueHi = value >> 4 & 0x0F;

    lcd_send_nibble(lcd_vars, valueHi, mode);
    lcd_send_nibble(lcd_vars, valueLo, mode);
}

static inline void lcd_command(lcdVariables_st *lcd_vars, uint8_t value)
{
    lcd_send(lcd_vars, value, RSMODE_CMD);
}

/* The write function is needed for derivation from the Print class. */
static size_t lcd_write(lcdVariables_st *lcd_vars, uint8_t value)
{
    lcd_send(lcd_vars, value, RSMODE_DATA);
    return 1;
}

/**
 * Allows us to fill the first 8 CGRAM locations
 * with custom characters
 */
static void lcd_create_char(lcdVariables_st *lcd_vars, uint8_t location, uint8_t charmap[])
{
    location &= 0x7; // we only have 8 locations 0-7
    lcd_command(lcd_vars, LCD_SETCGRAMADDR | (location << 3));
    for (int i = 0; i < 8; i++)
    {
        lcd_write(lcd_vars, charmap[i]);
    }
}

void lcd_begin(lcdVariables_st *lcd_vars,
               uint8_t i2c_write_addr,
               uint8_t initialize_i2c,
               uint8_t cols,
               uint8_t lines)
{

    lcd_vars->addr = i2c_write_addr;
    lcd_vars->backLight = 0;
    lcd_vars->numLines = lines;
    lcd_vars->displayFunction = 0;

    if (lines > 1)
    {
        lcd_vars->displayFunction |= LCD_2LINE;
    }

    /*
     * SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
     * according to datasheet, we need at least 40ms after power rises above 2.7V
     * before sending commands. Arduino can turn on way befor 4.5V so we'll wait 50
     */
    print_str_to_dbg_port("+");
    lcd_write_to_wire(lcd_vars, 0x00, 0, 0);
    tim9_delay_ms(50);

    /* put the LCD into 4 bit mode according to the hitachi HD44780 datasheet figure 26, pg 47 */
    lcd_send_nibble(lcd_vars, 0x03, RSMODE_CMD);
    tim9_delay_us(4500);
    lcd_send_nibble(lcd_vars, 0x03, RSMODE_CMD);
    tim9_delay_us(4500);
    lcd_send_nibble(lcd_vars, 0x03, RSMODE_CMD);
    tim9_delay_us(150);

    /* finally, set to 4-bit interface */
    lcd_send_nibble(lcd_vars, 0x02, RSMODE_CMD);

    /* finally, set # lines, font size, etc. */
    lcd_command(lcd_vars, LCD_FUNCTIONSET | lcd_vars->displayFunction);

    /* turn the display on with no cursor or blinking default */
    lcd_vars->displayControl = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    lcd_display(lcd_vars);
    lcd_clear(lcd_vars);

    /* Initialize to default text direction (for romance languages) */
    lcd_vars->displayMode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    lcd_command(lcd_vars, LCD_ENTRYMODESET | lcd_vars->displayMode);
}

/********** high level commands, for the user! */

void lcd_print_string(lcdVariables_st *lcd_vars, char *my_string)
{
    uint8_t lol;
    char *string_ptr = my_string;
    while (*string_ptr != '\0')
    {
        lol = (uint8_t)*string_ptr;
        lcd_write(lcd_vars, lol);
        ++string_ptr;
    }
}

void lcd_clear(lcdVariables_st *lcd_vars)
{
    lcd_command(lcd_vars, LCD_CLEARDISPLAY);
    tim9_delay_us(2000);
}

void lcd_home(lcdVariables_st *lcd_vars)
{
    lcd_command(lcd_vars, LCD_RETURNHOME);
    tim9_delay_us(2000);
}

/**
 * Set the cursor to a new position: Numlines is found in the lcd var struct
 */
void lcd_set_cursor(lcdVariables_st *lcd_vars, uint8_t col, uint8_t row)
{
    int row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    if (row >= lcd_vars->numLines)
    {
        /* we count rows starting w/0 */
        row = lcd_vars->numLines - 1;
    }
    lcd_command(lcd_vars, LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

/**
 * Turn the display on/off (quickly)
 */
void lcd_no_display(lcdVariables_st *lcd_vars)
{
    lcd_vars->displayControl &= ~LCD_DISPLAYON;
    lcd_command(lcd_vars, LCD_DISPLAY_CONTROL | lcd_vars->displayControl);
}

void lcd_display(lcdVariables_st *lcd_vars)
{
    lcd_vars->displayControl |= LCD_DISPLAYON;
    lcd_command(lcd_vars, LCD_DISPLAY_CONTROL | lcd_vars->displayControl);
}

// Turns the underline cursor on/off
void lcd_no_cursor(lcdVariables_st *lcd_vars)
{
    lcd_vars->displayControl &= ~LCD_CURSORON;
    lcd_command(lcd_vars, LCD_DISPLAY_CONTROL | lcd_vars->displayControl);
}
void lcd_cursor(lcdVariables_st *lcd_vars)
{
    lcd_vars->displayControl |= LCD_CURSORON;
    lcd_command(lcd_vars, LCD_DISPLAY_CONTROL | lcd_vars->displayControl);
}

/**
 * Turn on and off the blinking cursor
 */
void lcd_no_blink(lcdVariables_st *lcd_vars)
{
    lcd_vars->displayControl &= ~LCD_BLINKON;
    lcd_command(lcd_vars, LCD_DISPLAY_CONTROL | lcd_vars->displayControl);
}
void lcd_blink(lcdVariables_st *lcd_vars)
{
    lcd_vars->displayControl |= LCD_BLINKON;
    lcd_command(lcd_vars, LCD_DISPLAY_CONTROL | lcd_vars->displayControl);
}

/**
 * These commands scroll the display without changing the RAM
 */
void lcd_scroll_display_left(lcdVariables_st *lcd_vars)
{
    lcd_command(lcd_vars, LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void lcd_scroll_display_right(lcdVariables_st *lcd_vars)
{
    lcd_command(lcd_vars, LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

/* This is for text that flows Left to Right */
void lcd_left_to_right(lcdVariables_st *lcd_vars)
{
    lcd_vars->displayMode |= LCD_ENTRYLEFT;
    lcd_command(lcd_vars, LCD_ENTRYMODESET | lcd_vars->displayMode);
}

void lcd_right_to_left(lcdVariables_st *lcd_vars)
{
    lcd_vars->displayMode &= ~LCD_ENTRYLEFT;
    lcd_command(lcd_vars, LCD_ENTRYMODESET | lcd_vars->displayMode);
}

/* This will 'right justify' text from the cursor */
void lcd_autoscroll(lcdVariables_st *lcd_vars)
{
    lcd_vars->displayMode |= LCD_ENTRYSHIFTINCREMENT;
    lcd_command(lcd_vars, LCD_ENTRYMODESET | lcd_vars->displayMode);
}

/* This will 'left justify' text from the cursor */
void lcd_no_autoscroll(lcdVariables_st *lcd_vars)
{
    lcd_vars->displayMode &= ~LCD_ENTRYSHIFTINCREMENT;
    lcd_command(lcd_vars, LCD_ENTRYMODESET | lcd_vars->displayMode);
}

/**
 * Setting the brightness of the background display light.
 * The backlight can be switched on and off.
 */
void lcd_set_backlight(lcdVariables_st *lcd_vars, uint8_t brightness)
{
    lcd_vars->backLight = brightness;
    lcd_write_to_wire(lcd_vars, 0x00, RSMODE_DATA, 0);
}
