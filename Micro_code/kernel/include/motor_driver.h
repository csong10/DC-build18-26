#ifndef _MOTOR_DRIVER_H_
#define _MOTOR_DRIVER_H_

#include <encoder.h>
#include <gpio.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdint.h>

typedef enum motor_direction
{
    FREE = 0,
    FORWARD = 1,
    BACKWARD = 2,
    STOP = 3
} direction_t;

enum motor_mapping
{
    LEFT_MOTOR,
    RIGHT_MOTOR
};

struct motor_timer
{
    uint32_t timer;    ///< The timer number for the PWM Pin
    uint32_t channel;  ///< The timer channel for the PWM Pin
    uint32_t gpio_alt; ///< The alternate function number for the timer used on
                       ///< the PWM pin
    bool is_comp;      ///< True if the PWM pin you are using has an N at the end of
                       ///< it. Check the Arduino pin layout
};

struct motor_attr
{
    struct pin motor_in1;     ///< GPIO Pin for MOTOR_IN1
    struct pin motor_in2;     ///< GPIO Pin for MOTOR_IN2
    struct pin motor_en;      ///< GPIO Pin for MOTOR_EN (This should be your PWM
                              ///< capable pin)
    struct motor_timer timer; ///< The timer attributes of the motor
};

/**
 * Motor Driver initialization function
 * Initializes the motor driver.
 * This driver only supports two motors
 *
 * @param motor The motor that the pin configurations are linked to
 * @param attr The pin cnd timer configurations for the motor
 * @param enc_attr The encoder pin configurations
 */
void motor_init(enum motor_mapping motor, struct motor_attr *attr,
                struct encoder_pin_attr *enc_attr);

/**
 * Sets the direction and speed of the motor
 *
 * @param duty_cycle - Sets the duty_cycle (and thus the speed) of the PWM
 * output. Value must be 0 - 100
 * @param direction  - must be one of FREE, FORWARD, BACKWARD, STOP
 */
int sys_motor_set(enum motor_mapping motor, uint32_t duty_cycle,
                  direction_t direction);

/**
 * Registers a callback function to edit variables of user proj
 */
void sys_register_spd_callback(void (*callback) (uint32_t));

void btn1_irq_handler(void);

void btn2_irq_handler(void);

#endif /* _MOTOR_DRIVER_H_ */
