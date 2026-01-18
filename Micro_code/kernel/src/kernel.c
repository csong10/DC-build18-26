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

#define MAX_DUTY_CYCLE 100
#define MIN_DUTY_CYCLE 45

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
// MOTOR_IN2 : PB0 (PWM1/2N) A3
motor_in2 = {.port = GPIO_B, .num = 0},
// ENCA2 : PC1 (exti1)
enca2 = {.port = GPIO_C, .num = 1, .irq_num = 7},
// ENCB1 : PC0 (exti0)
encb1 = {.port = GPIO_C, .num = 0, .irq_num = 6},
// MOTOR_IN1 : PA10 D2
motor_in1 = {.port = GPIO_A, .num = 10},
// ENCB2 : PB10 (exti15_10) 
encb2 = {.port = GPIO_B, .num = 10, .irq_num = 40},
// MOTOR_IN3 : PA8 D7
motor_in3 = {.port = GPIO_A, .num = 8},
// MOTOR_IN4 : PA7 D11
motor_in4 = {.port = GPIO_A, .num = 7},
// MOTOR_IN5 : PA6 D12
motor_in5 = {.port = GPIO_A, .num = 6},
// MOTOR_IN6 : PA1 A1
motor_in6 = {.port = GPIO_A, .num = 1},
// MOTOR_IN7 : PC0 A5
motor_in7 = {.port = GPIO_C, .num = 0},
// MOTOR_IN8 : PC1 A4
motor_in8 = {.port = GPIO_C, .num = 1},
// MOTOR_ENC : PB6 (PWM4/1) D10
motor_enc = {.port = GPIO_B, .num = 6},
// MOTOR_END : PB5 (PWM3/2) D4
motor_end = {.port = GPIO_B, .num = 5},
// MOTOR_ENA : PB3 (PWM2/2) D3
motor_ena = {.port = GPIO_B, .num = 3},
// MOTOR_ENB : PB4 (PWM3/1) D5
motor_enb = {.port = GPIO_B, .num = 4};

// motor3,4 : left front B
struct encoder_pin_attr enc_rb = 
  {.encoder_pin_a = enca2, .encoder_pin_b = encb2};

struct motor_timer time_lf = 
  {.channel = 1, .gpio_alt = 2, .is_comp = 0, .timer = 3};

struct motor_attr attr_lf = 
  {.motor_in1 = motor_in3, .motor_in2 = motor_in4, .motor_en = motor_enb,
   .timer = time_lf};

// motor1,2 : left back A
struct encoder_pin_attr enc_lb = 
    {.encoder_pin_a = enca1, .encoder_pin_b = encb1};

struct motor_timer time_lb = 
  {.channel = 2, .gpio_alt = 1, .is_comp = 0, .timer = 2};

struct motor_attr attr_lb = 
  {.motor_in1 = motor_in2, .motor_in2 = motor_in1, .motor_en = motor_ena,
   .timer = time_lb};

//motor5,6 : right front C
struct motor_timer time_rf = 
  {.channel = 1, .gpio_alt = 2, .is_comp = 0, .timer = 4};

struct motor_attr attr_rf =
  {.motor_in1 = motor_in6, .motor_in2 = motor_in5, .motor_en = motor_enc,
   .timer = time_rf};

//motor7,8 : right back D
struct motor_timer time_rb =
  {.channel = 2, .gpio_alt = 2, .is_comp = 0, .timer = 3};

struct motor_attr attr_rb =
  {.motor_in1 = motor_in7, .motor_in2 = motor_in8, .motor_en = motor_end,
   .timer = time_rb};

   //enc are not used
    motor_init(RB_MOTOR, &attr_rb, &enc_rb);
    motor_init(LB_MOTOR, &attr_lb, &enc_lb);
    motor_init(RF_MOTOR, &attr_rf, &enc_rb);
    motor_init(LF_MOTOR, &attr_lf, &enc_lb);

    sys_motor_set(LB_MOTOR, MIN_DUTY_CYCLE, FORWARD);
    sys_motor_set(RB_MOTOR, MIN_DUTY_CYCLE, STOP);
    sys_motor_set(LF_MOTOR, MIN_DUTY_CYCLE, FORWARD);
    sys_motor_set(RF_MOTOR, MIN_DUTY_CYCLE, STOP);
    while (1) {
    }
    return 0;
}
