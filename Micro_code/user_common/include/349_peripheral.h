/**
 * @file 349_peripheral.h
 * @brief Syscall APIs for peripheral/on-board devices
 * @author Tom Schmitz
 * @version 0.1 Lab5 Fall 2024
 * @date 2024-11-19
 */
#ifndef _349_PERIPHERAL_H
#define _349_PERIPHERAL_H

typedef enum encoder { LEFT_ENCODER, RIGHT_ENCODER } encoder_t;

typedef enum motor { LEFT_MOTOR, RIGHT_MOTOR } motor_t;

/**
 * @brief Register a callback for encoder interrupts
 *
 * @param encoder The encoder to register the callback for
 * @param callback Point to the callback function
 */
void register_encoder_callback(encoder_t encoder,
                               void (*callback)(uint32_t, uint32_t));

/* Defines for motor direction input to motor_set */
typedef enum motor_direction {
    MOTOR_FREE = 0,
    MOTOR_FORWARD = 1,
    MOTOR_BACKWARD = 2,
    MOTOR_STOP = 3
} direction_t;

/**
 * @brief Set motor speed and direction
 *
 * @param motor The motor to set the speed and direction for
 * @param duty_cycle This is the speed value, which is an integer
 *                   between 0 and 100
 * @param direction Direction of rotation desired
 *                   0 - Free rotation
 *                   1 - Forward
 *                   2 - Backward
 *                   3 - Stop
 *
 */
void motor_set(motor_t motor, uint32_t duty_cycle, direction_t direction);

//LCD STUFF

void lcd_clear(void);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_print(char *str);

void register_spd_callback(void (*callback)(uint32_t));

#endif /* _349_PERIPHERAL_H */
