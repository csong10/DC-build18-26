/** @file exti.h
 *
 *  @brief  wrappers for external interrupts and respective handlers.
 *
 *  @date   January 7, 2026
 *
 *  @author Caleb Song
 */

#ifndef _EXTI_H_
#define _EXTI_H_

#include <gpio.h>
#include <unistd.h>

////////////////////////////  CONST  /////////////////////////////////

#define RISING_EDGE         1
#define FALLING_EDGE        2
#define RISING_FALLING_EDGE 3

////////////////////////////  END CONST  /////////////////////////////////

////////////////////////////  FN HEADERS  /////////////////////////////////

/**
 * @brief Enable an external interrupt
 *
 * @param sel     - The GPIO port for the external interrupt
 * @param channel - The GPIO channel for the external interrupt
 * @param edge    - The ege triggering the interrupt. Must be one of
 * RISING_EDGE, FALLING_EDGE, RISING_FALLING_EDGE
 */
void enable_exti(gpio_port port, uint32_t channel, uint32_t edge);

/**
 * @brief Disable the exti
 *
 * @param channel - The channel to disable
 */
void disable_exti(uint32_t channel);

/**
 * @brief Clear the pending bit for the exti
 *
 * @param channel - The channel of interest
 */
void exti_clear_pending_bit(uint32_t channel);

uint8_t exti_channel_to_irq(uint32_t channel);

////////////////////////////  END FN HEADERS  /////////////////////////////////


#endif /* _EXTI_H_ */
