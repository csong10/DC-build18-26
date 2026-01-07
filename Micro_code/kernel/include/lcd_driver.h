/** @file lcd_driver.h
 *
 *  @brief  Wrappers for interfacing with LCD screen
 *
 *  @date   January 7, 2026
 *
 *  @author Caleb Song
 */
#ifndef _LCD_DRIVER_H_
#define _LCD_DRIVER_H_

#include <unistd.h>

void lcd_driver_init();
void sys_lcd_print(char *input);
void sys_lcd_set_cursor(uint8_t row, uint8_t col);
void sys_lcd_clear();

/** @brief returns the row of the cursor */
int lcd_get_cursor_row();
/** @brief returns the column of the cursor */
int lcd_get_cursor_col();
/** @brief sets the cursor to user defined row */
int lcd_set_cursor_row(int row);

void lcd_send_cmd(uint8_t c);
void lcd_send_data(uint8_t d);
void lcd_send_instr(uint8_t s, int init, int mode);
void sys_lcd_swap_row();

#endif /* _LCD_DRIVER_H_ */
