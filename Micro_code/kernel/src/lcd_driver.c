/**
 * @file lcd_driver.c
 *
 * @brief implementing the lcd of the lab 2
 *
 * @date 9/28/27
 *
 * @author Caleb Song calebson Andrew Liu andrewl5
 */
#include <unistd.h>

#include <i2c.h>
#include <lcd_driver.h>
#include <systick.h>

/** @brief PCF8574T constants */
#define LCD_ADDR 0x27

#define P_NULL 0x00
#define P_INSTR_LEN 4
#define SAFETY_WAIT 5000

/** @brief HD44780U constants */
#define INIT 0x38
#define INIT_EN 1
#define FOUR_BIT_MODE 0x28 //on, cursor on, blink off
#define SET_DDRAM 0x80 //set cursor pos, add addr to this

/**
 *  NOTE: pin mapping:
 *  P0 -> RS (0 = send command, 1 = send data)
 *  P1 -> RW (0)
 *  P2 -> E (pulsed per 4 bit transfer)
 *  P3 -> Backlight (1)
 *  P4 -> D4
 *  P5 -> D5
 *  P6 -> D6
 *  P7 -> D7
 *  4 bit packet sent once for high, once for low
 */


/** @brief Commands */
#define CLEAR_DISP 0x01
#define DISP_ON 0x0F //display on, cursor on, blink on
#define SET_DDRAM 0x80 //set cursor pos, add addr to this

/** @brief LCD logic constants */
#define ROW0_ADDR 0x00
#define ROW1_ADDR 0x40
#define R_MODE 0x01
#define W_MODE 0x00
#define MAX_CHARS 16
#define MAX_ROWS 2

/** @brief global vars */
static uint8_t cursor_row;
static uint8_t cursor_col0;
static uint8_t cursor_col1;

/**
 * @brief initializes the lcd driver and also clears it
 */
void lcd_driver_init() {

    lcd_send_instr(INIT, INIT_EN, W_MODE);

    sys_lcd_clear();
    
    cursor_row = 0;
    cursor_col0 = 0;
    cursor_col1 = 0;
    
    lcd_send_cmd(DISP_ON);
	return;
}

/**
 * @brief prints the given string to the LCD
 * ASSUME: input is a null terminated string
 * @param[in] input is the string to be printed
 * 
 * for every character, it will increment the column position of the respective
 * row every time it writes and because it is stored in a static variable
 * the last column written to should be remembered and when it switches
 * it should switch to the last indexed place.
 */
void sys_lcd_print(char *input){

    for (int i = 0; input[i] != '\0'; i++) {

        switch (cursor_row) {

            case 0:
                if (cursor_col0 == MAX_CHARS) {
                    cursor_row = 1;
                    cursor_col0 = 0;
                    sys_lcd_set_cursor(cursor_row, cursor_col0);
                }
                cursor_col0++;
                break;

            case 1:
                if (cursor_col1 == MAX_CHARS) {
                    cursor_row = 0;
                    cursor_col1 = 0;
                    sys_lcd_set_cursor(cursor_row, cursor_col1);
                }
                cursor_col1++;
                break;
        }

        lcd_send_data(input[i]);
    }
}

/**
 * @brief helper function to set the cursor to specified coordinate
 * @param[in] row is the row to be set to
 * @param[in] col is the col to set the cursor to
 */
void sys_lcd_set_cursor(uint8_t row, uint8_t col){
    switch (row) {
            case 0:
                lcd_send_cmd(SET_DDRAM | (ROW0_ADDR + col));
                break;
            case 1:
                //add 0x40 because that is the index of the first element in row1
                lcd_send_cmd(SET_DDRAM | (ROW1_ADDR + col));
                break;
        }
}

/**
 * @brief API function that will swap the cursor to the last known col in other row
 */
void sys_lcd_swap_row() {
    cursor_row = (cursor_row + 1) % 2;
    sys_lcd_set_cursor(cursor_row, ((cursor_row) ? cursor_col0 : cursor_col1));
}

/**
 * @brief API function that will clear the LCD screen
 */
void sys_lcd_clear() {
    lcd_send_cmd(CLEAR_DISP);
    cursor_row = 0;
    cursor_col0 = 0;
    cursor_col1 = 0;
    for(int i = 0; i < SAFETY_WAIT; i++);
    return;
}

/**
 * @brief helper function that sends a command to the LCD
 * @param[in] c the 8 bit command to send
 */
void lcd_send_cmd(uint8_t c) {
    lcd_send_instr(c, !INIT_EN, W_MODE);
    return;
}

/**
 * @brief helper function that puts data to the LCD
 * @param[in] d 8 bit data to read
 */
void lcd_send_data(uint8_t d) {
    lcd_send_instr(d, !INIT_EN, R_MODE);
    return;
}

/**
 * @brief helper function that does the sending logic to the LCD
 * @param[in] s 8 bit data to send
 * @param[in] init if 1 it will send the init sequence or do a regular send otherwise
 * @param[in] mode if R_MODE it will put to the LCD, if W_MODE it will send command
 */
void lcd_send_instr(uint8_t s, int init, int mode) {
    if (init) {
        //send init sequence
        uint8_t first[] = {(INIT | 0x4), INIT, (P_NULL | 0xC), (P_NULL | 0x8)};
        uint8_t second[] = {(FOUR_BIT_MODE | 0x4), FOUR_BIT_MODE, (P_NULL | 0xC), (P_NULL | 0x8)};

        i2c_master_write(first, P_INSTR_LEN, LCD_ADDR);
        
        i2c_master_write(first, P_INSTR_LEN, LCD_ADDR);

        i2c_master_write(first, P_INSTR_LEN, LCD_ADDR);

        i2c_master_write(second, P_INSTR_LEN, LCD_ADDR);
        return;
    }

    uint8_t first_hi, first_lo, sec_hi, sec_lo;

    first_hi = (s & 0xF0) | 0x0C | mode;  
    first_lo = (s & 0xF0) | 0x08 | mode;

    sec_hi = ((s << P_INSTR_LEN) & 0xF0) | 0x0C | mode;
    sec_lo = ((s << P_INSTR_LEN) & 0xF0) | 0x08 | mode;

    uint8_t seq[P_INSTR_LEN] = {first_hi, first_lo, sec_hi, sec_lo}; 
    i2c_master_write(seq, P_INSTR_LEN, LCD_ADDR);
}