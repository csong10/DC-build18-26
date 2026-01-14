#include <encoder.h>
#include <gpio.h>
#include <motor_driver.h>
#include <pwm.h>
#include <stdint.h>
#include <unistd.h>

#define PWM_PERIOD 4000

#define MAX_DUTY_CYCLE 100

#define NUM_MOTORS UINT32_C(2)

#define GPIO_MOTOR_IN_ATTR \
    MODE_GP_OUTPUT, OUTPUT_PUSH_PULL, OUTPUT_SPEED_HIGH, PUPD_NONE, ALT0

static struct motor_attr motors[NUM_MOTORS];

uint8_t IS_COMP;

void (*spd_callback_fn)(uint32_t up) = NULL;

void motor_init(enum motor_mapping motor, struct motor_attr *attr,
                struct encoder_pin_attr *enc_attr)
{
    motors[motor] = *attr;

    gpio_init(attr->motor_in1.port, attr->motor_in1.num, GPIO_MOTOR_IN_ATTR);
    gpio_init(attr->motor_in2.port, attr->motor_in2.num, GPIO_MOTOR_IN_ATTR);
    gpio_init(attr->motor_en.port, attr->motor_en.num, MODE_ALT,
              OUTPUT_PUSH_PULL, OUTPUT_SPEED_VERY_HIGH, PUPD_PULL_DOWN,
              attr->timer.gpio_alt);

    encoder_init((enum encoder_mapping)motor, enc_attr);

    IS_COMP = (attr->timer.is_comp) ? 1 : 0;
    timer_start_pwm(PWM_PERIOD, 0, attr->timer.timer, attr->timer.channel);
}

//EDIT: going to be controlling 4 motors, to turn stop some motors, enable the rest
int sys_motor_set(enum motor_mapping motor, uint32_t duty_cycle,
                  direction_t direction)
{
    if (motor >= NUM_MOTORS || duty_cycle > MAX_DUTY_CYCLE) {
        return -1;
    }

    uint32_t real_duty = (duty_cycle * PWM_PERIOD) / 100;

    //might have to change the hi low for back and forward
    switch (direction) {
        case FREE:
          gpio_clr(motors[motor].motor_in1.port, motors[motor].motor_in1.num);
          gpio_clr(motors[motor].motor_in2.port, motors[motor].motor_in2.num);

          timer_set_duty_cycle(motors[motor].timer.timer, motors[motor].timer.channel, 0);
          break;
        case STOP:
          gpio_set(motors[motor].motor_in1.port, motors[motor].motor_in1.num);
          gpio_set(motors[motor].motor_in2.port, motors[motor].motor_in2.num);

          timer_set_duty_cycle(motors[motor].timer.timer, motors[motor].timer.channel, 0);
          break;
        case FORWARD:
          gpio_set(motors[motor].motor_in1.port, motors[motor].motor_in1.num);
          gpio_clr(motors[motor].motor_in2.port, motors[motor].motor_in2.num);

          timer_set_duty_cycle(motors[motor].timer.timer, motors[motor].timer.channel, real_duty);
          break;
        case BACKWARD:
          gpio_clr(motors[motor].motor_in1.port, motors[motor].motor_in1.num);
          gpio_set(motors[motor].motor_in2.port, motors[motor].motor_in2.num);

          timer_set_duty_cycle(motors[motor].timer.timer, motors[motor].timer.channel, real_duty);
          break;
        default:
          return -1;
    }
    return 0;
}

void sys_register_spd_callback(void (*callback) (uint32_t)) {
  spd_callback_fn = callback;
}

//S3
void btn1_irq_handler() {
  if (spd_callback_fn != NULL) spd_callback_fn(1);
}

//S2
void btn2_irq_handler() {
  if (spd_callback_fn != NULL) spd_callback_fn(0);
}