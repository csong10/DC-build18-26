/** @file main.c
 *
 *  @brief  motor driver
 */
#include <349_lib.h>
#include <349_peripheral.h>
#include <349_threads.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <stdlib.h>

#define UNUSED __attribute__((unused))

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

/** @brief Computation time of the task */
static const int THREAD_C_MS[] = {100, 100, 200};
/** @brief Period of the thread */
static const int THREAD_T_MS[] = {500, 500, 1000};

typedef struct {
  double kp;
  double kp_r;
  double ki;
  double ki_r;
  double kd;
  double kd_r;
  double integral_sum_l;
  double integral_sum_r;
  double integral_bnd;
} pid_ctrl_t;

typedef struct {
  uint32_t prev_a;
  uint32_t prev_b;
  uint32_t prev_time;
  double prev_speed;
  double new_speed;
  direction_t dir;
  int prev_pos;
  int pos;
  uint32_t enabled;
  double prev_err;
} motor_state_t;

typedef enum {
  TOP = 0,
  RIGHT = 1,
  BOT = 3,
  LEFT = 2
} encoder_state_t;

volatile motor_state_t left_motor, right_motor;

volatile double target_speed = 0;

volatile pid_ctrl_t pid_const = {0};

//set T to be 20 ticks or something
void pid_fn() {
//   mutex_t *mux = (mutex_t *)vargp; //look at test 6 0

  while (1) {
    // mutex_lock(mux);

    double p_r, p_l, i_r, i_l, d_r, d_l;
    uint32_t curr_time = get_time();
    double dt_l = (double) (curr_time - left_motor.prev_time);
    double dt_r = (double) (curr_time - right_motor.prev_time);

    dt_l = (!dt_l) ? 1.0 : dt_l;
    dt_r = (!dt_r) ? 1.0 : dt_r;

    left_motor.new_speed =  ((double)(left_motor.pos - left_motor.prev_pos)) / dt_l; 
    right_motor.new_speed = ((double)(right_motor.pos - right_motor.prev_pos)) / dt_r;
      
    double error_l = target_speed - left_motor.new_speed;
    double error_r = target_speed - right_motor.new_speed;

    //P
    p_r = pid_const.kp * error_r;
    p_l = pid_const.kp * error_l;

    //I
    pid_const.integral_sum_r += error_r;
    pid_const.integral_sum_r = MIN(pid_const.integral_sum_r, pid_const.integral_bnd);
    pid_const.integral_sum_r = MAX(pid_const.integral_sum_r, -pid_const.integral_bnd);
    i_r = pid_const.ki * pid_const.integral_sum_r;

    pid_const.integral_sum_l += error_l;
    pid_const.integral_sum_l = MIN(pid_const.integral_sum_l, pid_const.integral_bnd);
    pid_const.integral_sum_l = MAX(pid_const.integral_sum_l, -pid_const.integral_bnd);
    i_l = pid_const.ki * pid_const.integral_sum_l;

    //D
    d_r = pid_const.kd * (error_r - right_motor.prev_err);
    d_l = pid_const.kd * (error_l - left_motor.prev_err);

    //new speed to update to
    double pid_out_r = p_r + i_r + d_r;
    if (pid_out_r < 0) {
        right_motor.dir = (right_motor.dir == MOTOR_FORWARD) ? MOTOR_BACKWARD : MOTOR_FORWARD;
        pid_out_r = -pid_out_r;
    }
    uint32_t duty_r = (uint32_t)((pid_out_r / MAX_SPEED) * MAX_DUTY_CYCLE);
    duty_r = MIN(duty_r, MAX_DUTY_CYCLE);
    duty_r = MAX(duty_r, MIN_DUTY_CYCLE);

    double pid_out_l = p_l + i_l + d_l;
    if (pid_out_l < 0) {
        left_motor.dir = (left_motor.dir == MOTOR_FORWARD) ? MOTOR_BACKWARD : MOTOR_FORWARD;
        pid_out_l = -pid_out_l;
    }
    uint32_t duty_l = (uint32_t)((pid_out_l / MAX_SPEED) * MAX_DUTY_CYCLE);
    duty_l = MIN(duty_l, MAX_DUTY_CYCLE);
    duty_l = MAX(duty_l, MIN_DUTY_CYCLE);

    motor_set(LEFT_MOTOR, duty_l, left_motor.dir);
    motor_set(RIGHT_MOTOR, duty_r, right_motor.dir);

    //update the struct
    left_motor.prev_speed = left_motor.new_speed;
    left_motor.prev_err = error_l;
    left_motor.prev_pos = left_motor.pos;
    curr_time = get_time();
    left_motor.prev_time = curr_time;

    right_motor.prev_speed = right_motor.new_speed;
    right_motor.prev_err = error_r;
    right_motor.prev_pos = right_motor.pos;
    curr_time = get_time();
    right_motor.prev_time = curr_time;

    // mutex_unlock(mux);
    //wait until next period
    wait_until_next_period();
  }
}

void enc_l_callback(uint32_t a, uint32_t b) {
  encoder_state_t state, nstate;
  state =  (encoder_state_t) ((left_motor.prev_a << 1) | left_motor.prev_b);
  nstate = (encoder_state_t) ((a << 1) | b);

  if (!left_motor.enabled) {
    left_motor.prev_a = a;
    left_motor.prev_b = b;
    left_motor.enabled = 1;
    left_motor.prev_pos = 0;
    left_motor.pos = 0;
    left_motor.prev_speed = 0;
    return;
  }

  switch (state) {
    case TOP:
      if (nstate == RIGHT) {
        left_motor.pos++;
      } else if (nstate == LEFT) {
        left_motor.pos = (!left_motor.pos) ? 0 : (left_motor.pos - 1);
      }
      break;
    case RIGHT:
      if (nstate == BOT) {
        left_motor.pos++;
      } else if (nstate == TOP) {
        left_motor.pos = (!left_motor.pos) ? 0 : (left_motor.pos - 1);
      }
      break;
    case BOT:
      if (nstate == LEFT) {
        left_motor.pos++;
      } else if (nstate == RIGHT) {
        left_motor.pos = (!left_motor.pos) ? 0 : (left_motor.pos - 1);
      }
      break;
    case LEFT:
      if (nstate == TOP) {
        left_motor.pos++;
      } else if (nstate == BOT) {
        left_motor.pos = (!left_motor.pos) ? 0 : (left_motor.pos - 1);
      }
      break;
  }

  left_motor.prev_a = a;
  left_motor.prev_b = b;
}

void enc_r_callback(uint32_t a, uint32_t b) {
  encoder_state_t state, nstate;
  state =  (encoder_state_t) ((right_motor.prev_a << 1) | right_motor.prev_b);
  nstate = (encoder_state_t) ((a << 1) | b);

  if (!right_motor.enabled) {
    right_motor.prev_a = a;
    right_motor.prev_b = b;
    right_motor.enabled = 1;
    right_motor.prev_pos = 0;
    right_motor.pos = 0;
    right_motor.prev_speed = 0;
    return;
  }

  switch (state) {
    case TOP:
      if (nstate == RIGHT) {
        right_motor.pos = (!right_motor.pos) ? 0 : (right_motor.pos - 1);
      } else if (nstate == LEFT) {
        right_motor.pos++;
      }
      break;
    case RIGHT:
      if (nstate == BOT) {
        right_motor.pos = (!right_motor.pos) ? 0 : (right_motor.pos - 1);
      } else if (nstate == TOP) {
        right_motor.pos++;
      }
      break;
    case BOT:
      if (nstate == LEFT) {
        right_motor.pos = (!right_motor.pos) ? 0 : (right_motor.pos - 1);
      } else if (nstate == RIGHT) {
        right_motor.pos++;
      }
      break;
    case LEFT:
      if (nstate == TOP) {
        right_motor.pos = (!right_motor.pos) ? 0 : (right_motor.pos - 1);
      } else if (nstate == BOT) {
        right_motor.pos++;
      }
      break;
  }

  right_motor.prev_a = a;
  right_motor.prev_b = b;
}

void uart_thread_fn() {
//   mutex_t *mux = (mutex_t *) vargp;
  while (1) {

    // mutex_lock(mux);
    int t = (int)(target_speed * 100);

    int ml = (int)(left_motor.new_speed * 100);

    int mr = (int)(right_motor.new_speed * 100);

    printf("T:%d,ML:%d,MR:%d\n", t, ml, mr);
    // printf("pos_r: %d, pos_l: %d\n", right_motor.pos, left_motor.pos);
    // mutex_unlock(mux);

    wait_until_next_period(); 
  }
}

void lcd_thread_fn() {
//   mutex_t *mux = (mutex_t *) vargp;
  static int state = 0;   // 0 = ERR view, 1 = PID view

  while (1) {
    //   mutex_lock(mux);
      double error_l = target_speed - left_motor.new_speed;
      double error_r = target_speed - right_motor.new_speed;

      double p_l = pid_const.kp * error_l;
      double p_r = pid_const.kp * error_r;

      double i_l = pid_const.ki * pid_const.integral_sum_l;
      double i_r = pid_const.ki * pid_const.integral_sum_r;

      double d_l = pid_const.kd * (left_motor.new_speed - left_motor.prev_speed);
      double d_r = pid_const.kd * (right_motor.new_speed - right_motor.prev_speed);

      lcd_clear();

      char buf[32];

      if (state == 0) {
          //display error info
          lcd_set_cursor(0,0);
          snprintf(buf, sizeof(buf), "L e:%d  R e:%d", (int)error_l, (int)error_r);
          lcd_print(buf);

          lcd_set_cursor(1,0);
          snprintf(buf, sizeof(buf), "Tgt:%d", (int)target_speed);
          lcd_print(buf);
      }
      else {
          //display pid info
          lcd_set_cursor(0,0);
          snprintf(buf, sizeof(buf), "L:%d,%d,%d", (int)p_l, (int)i_l, (int)d_l);
          lcd_print(buf);

          lcd_set_cursor(1,0);
          snprintf(buf, sizeof(buf), "R:%d,%d,%d", (int)p_r, (int)i_r, (int)d_r);
          lcd_print(buf);
      }

      // Flip view next time
      state = ~state;

    //   mutex_unlock(mux);
      wait_until_next_period();
  }
}


void update_speed(uint32_t up) {
  target_speed = (up) ? (target_speed+0.1) : (target_speed - 0.1); 
}

int main(UNUSED int argc, UNUSED char const *argv[])
{
    printf("hello world!\n");

    ABORT_ON_ERROR(thread_init(NUM_THREADS, USR_STACK_WORDS, NULL, NUM_MUTEXES));

    // mutex_t *s1 = mutex_init(0);

    pid_const.kp = 0.75;
    pid_const.ki = 0.35;
    pid_const.kd = 0.15;

    // pid_const.kp_r = 1.0;
    // pid_const.ki_r = 0.0;
    // pid_const.kd_r = 0.0;

    target_speed = 2.5;
    
    pid_const.integral_bnd = (!pid_const.ki) ? MAX_SPEED : (MAX_SPEED / pid_const.ki);

    left_motor.prev_speed = 0.0;
    right_motor.prev_speed = 0.0;

    for ( int i = 0; i < NUM_THREADS; i++ ) {
    switch (i) {
        case 0:
          thread_create( &pid_fn, 
          i, THREAD_C_MS[i], THREAD_T_MS[i], ( void * ) i);
          break;
        case 1:
          ABORT_ON_ERROR(thread_create( &uart_thread_fn, 
          i, THREAD_C_MS[i], THREAD_T_MS[i], ( void * )i ));
          break;
        case 2:
          thread_create( &lcd_thread_fn, 
          i, THREAD_C_MS[i], THREAD_T_MS[i], ( void * )i );
          break;
    }
  }

  register_spd_callback(&update_speed);

  register_encoder_callback(LEFT_ENCODER, &enc_l_callback);
  register_encoder_callback(RIGHT_ENCODER, &enc_r_callback);

  motor_set(LEFT_MOTOR, MIN_DUTY_CYCLE, MOTOR_FORWARD);
  motor_set(RIGHT_MOTOR, MIN_DUTY_CYCLE, MOTOR_FORWARD);

  left_motor.dir = MOTOR_FORWARD;
  right_motor.dir = MOTOR_FORWARD;

  scheduler_start( CLOCK_FREQUENCY );

    return RET_0349;
}