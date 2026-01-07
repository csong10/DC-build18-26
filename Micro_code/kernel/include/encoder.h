/** @file encoder.h
 *
 *  @brief  wrappers for encoder functionality and wheel tracking.
 *
 *  @date   January 7, 2026
 *
 *  @author Caleb Song
 */

#ifndef _ENCODER_H_
#define _ENCODER_H_

#include <gpio.h>
#include <unistd.h>

////////////////////////////  STRUCTS  /////////////////////////////////

enum encoder_mapping
{
    ENCODER_LEFT = 0,
    ENCODER_RIGHT = 1
};

/**
 * @brief common gpio config for encoders
 */
struct pin
{
    gpio_port port;
    uint32_t num;
    uint32_t irq_num;
};

/**
 * @brief Each side has both an A and B terminal to their encoder.
 */
struct encoder_pin_attr
{
    struct pin encoder_pin_a;
    struct pin encoder_pin_b;
};

////////////////////////////  END STRUCTS  /////////////////////////////////

////////////////////////////  CONST  /////////////////////////////////
/** @brief empirical amount of tick changes per one revolution of a wheel */
#define TICKS_PER_REV 1200

/** @brief position of one full rotation */
#define MAX_POS 255

////////////////////////////  END CONST  /////////////////////////////////

////////////////////////////  FN HEADERS  /////////////////////////////////
/*
 * Initialize the encoder
 * This only supports one encoder at a time
 */
void encoder_init(enum encoder_mapping encoder, struct encoder_pin_attr *attr);

/*
 * Stop the encoder
 * This only supports one encoder at a time
 */
void encoder_stop(enum encoder_mapping encoder);

/*
 * Handle the IRQ for the left encoder
 * Calculate the position.
 */
void encoder_irq_handler_left();

/*
 * Handle the IRQ for the right encoder
 * Calculate the position.
 */
void encoder_irq_handler_right();

/**
 * @brief Register a callback function for the encoder ISR
 *
 * @param callback Pointer to the callback function
 *
 * @return 0 upon success
 * @return -1 upon failure
 */
int sys_register_encoder_callback(uint32_t encoder,
                                  void (*callback)(uint32_t, uint32_t));

////////////////////////////  END FN HEADERS  /////////////////////////////////

#endif /* _ENCODER_H_ */
