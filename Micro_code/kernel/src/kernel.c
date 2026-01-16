/**
 * @file     kernel.c
 *
 * @brief    Kernel entry point
 *
 */

#include "kernel.h"
#include "arm.h"
#include "pwm.h"
#include "motor_driver.h"
#include "encoder.h"
#include "exti.h"
#include "uart.h"
#include "lcd_driver.h"
#include "i2c.h"
#include "systick.h"

#define BAUD_RATE_115200 0x8B

#define LED_OUT \
    MODE_GP_OUTPUT, OUTPUT_PUSH_PULL, OUTPUT_SPEED_HIGH, PUPD_NONE, ALT0

#define TICKS_PER_REV 1200
#define MAX_SPEED 10
#define MIN_SPEED 0

#define MAX_POS (1 << 29)

#define MAX_DUTY_CYCLE 100
#define MIN_DUTY_CYCLE 15

#define MAX(x,y) (((x) > (y)) ? (x) : (y))
#define MIN(x,y) (((x) < (y)) ? (x) : (y)) 

#define USR_STACK_WORDS 256
#define NUM_THREADS 3  
#define NUM_MUTEXES 0
#define CLOCK_FREQUENCY 1000

int kernel_main(void) {
    init_349(); // DO NOT REMOVE THIS LINE
    uart_init(BAUD_RATE_115200);

// ENCA1 : PA4 (exti4)
struct pin enca1 = {.port = GPIO_A, .num = 4, .irq_num = 10},
// // SERVO : PA1 (PWM2/2)
// srvo = {.port = GPIO_A, .num = 1},
// MOTOR_IN2 : PB0 (PWM1/2N)
motor_in2 = {.port = GPIO_B, .num = 0},
// ENCA2 : PC1 (exti1)
enca2 = {.port = GPIO_C, .num = 1, .irq_num = 7},
// ENCB1 : PC0 (exti0)
encb1 = {.port = GPIO_C, .num = 0, .irq_num = 6},
// MOTOR_IN1 : PA10 
motor_in1 = {.port = GPIO_A, .num = 10},
// MOTOR_ENA : PB3 (PWM2/2)
motor_ena = {.port = GPIO_B, .num = 3},
// MOTOR_ENB : PB4 (PWM3/1)
motor_enb = {.port = GPIO_B, .num = 4},
// ENCB2 : PB10 (exti15_10)
encb2 = {.port = GPIO_B, .num = 10, .irq_num = 40},
// MOTOR_IN3 : PA8
motor_in3 = {.port = GPIO_A, .num = 8},
// MOTOR_IN4 : PA7
motor_in4 = {.port = GPIO_A, .num = 7};
// // SDA : PB9
// sda = {.port = GPIO_B, .num = 9},
// // SCL : PB8
// scl = {.port = GPIO_B, .num = 8};

// motor3,4 : Right
struct encoder_pin_attr enc_r = 
  {.encoder_pin_a = enca2, .encoder_pin_b = encb2};

struct motor_timer time_r = 
  {.channel = 1, .gpio_alt = 2, .is_comp = 0, .timer = 3};

struct motor_attr attr_r = 
  {.motor_in1 = motor_in3, .motor_in2 = motor_in4, .motor_en = motor_enb,
   .timer = time_r};

// motor1,2 : Left
struct encoder_pin_attr enc_l = 
    {.encoder_pin_a = enca1, .encoder_pin_b = encb1};

struct motor_timer time_l = 
  {.channel = 2, .gpio_alt = 1, .is_comp = 0, .timer = 2};

struct motor_attr attr_l = 
  {.motor_in1 = motor_in2, .motor_in2 = motor_in1, .motor_en = motor_ena,
   .timer = time_l};

   //enc are not used
    motor_init(RIGHT_MOTOR, &attr_r, &enc_r);
    motor_init(LEFT_MOTOR, &attr_l, &enc_l);

    sys_motor_set(LEFT_MOTOR, MIN_DUTY_CYCLE, STOP);
    sys_motor_set(RIGHT_MOTOR, MIN_DUTY_CYCLE, STOP);
    while (1) {
    }
    return 0;
}
