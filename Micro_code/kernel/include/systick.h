/**
* @file   systick.h
*
* @brief  Prototypes for SysTick control functions
*
* @date   20 Oct 2019
*
* @author Benjamin Huang <zemingbh@andrew.cmu.edu>
*/

#ifndef _SYSTICK_H_
#define _SYSTICK_H_

#include <unistd.h>

/** @brief  The processor frequency (16MHz) */
#define BASE_FREQ 16000000

/**
 * @brief Initialize systick to frequency of <frequency>
 * @param frequency the frequency the user wants systick to run at
 */
void systick_init(uint32_t frequency);

/**
 * @brief delays the system for tick amount of clock cycles
 * @param ticks number of clock cycles to delay systick interrupt for
 */
void systick_delay(uint32_t ticks);

/**
 * @brief returns the number of ticks since the systick started
 * @returns the number of ticks since systick started
 */
uint32_t systick_get_ticks();

/**
 * @brief increments global ticks and updates status of threads currently init in kernel
 */
void systick_c_handler();

void systick_off();

#endif /* _SYSTICK_H_ */