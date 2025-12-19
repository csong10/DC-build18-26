#ifndef _PWM_H_
#define _PWM_H_

#include <gpio.h>
#include <unistd.h>
#include <stdint.h>

extern uint8_t IS_COMP;

/*
 * Starts the timer based pwm
 *
 * @param period     - The period of the PWM
 * @param duty_cycle - The starting duty cycle of the PWM signal
 * @param timer      - The timer controlling the PWM
 * @param channel    - The timer channel controlling the PWM
 */
void timer_start_pwm(uint32_t period, uint32_t duty_cycle, uint32_t timer,
                     uint32_t channel);

/*
 * Stops the timer based pwm
 *
 * @param timer      - The timer controlling the PWM
 * @param channel    - The timer channel controlling the PWM
 */
void timer_disable_pwm(uint32_t timer, uint32_t channel);

/*
 * Sets the duty cycle of the PWM signal
 *
 * @param duty_cycle - The new duty cycle of the PWM signal
 * @param timer      - The timer controlling the PWM
 * @param channel    - The timer channel controlling the PWM
 */
void timer_set_duty_cycle(uint32_t timer, uint32_t channel, uint32_t duty_cycle);

#endif /* _PWM_H_ */
