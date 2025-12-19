/**
 * @file 
 *
 * @brief      
 *
 * @date       
 *
 * @author Caleb Song and Andrew Liu
 */

#include <stdint.h>
#include <debug.h>
#include <unistd.h>

#include <syscall.h>
#include <svc_num.h>
#include <syscall_thread.h>
#include <syscall_mutex.h>
#include <motor_driver.h>
#include <encoder.h>
#include <lcd_driver.h>

#define UNUSED __attribute__((unused))

struct stack_frame_reg_map {
  volatile uint32_t R0;
  volatile uint32_t R1;
  volatile uint32_t R2;
  volatile uint32_t R3;
  volatile uint32_t R12;
  volatile uint32_t LR;
  volatile uint32_t PC;
  volatile uint32_t xPSR;
};

/**
 * @brief handles all svc calls made by the user 
 * 
 * @param psp [in] the process stack pointer that was loaded into r0 by the assembly helper function
 * @param r4 [in] is the 5th argument that is needed for max_mutexes and it was found by added 32 bytes to psp
 *            this would give us the first register after the interrupt stack frame
 */
void svc_c_handler(struct stack_frame_reg_map *psp, uint32_t r4) {
  uint8_t svc_number = *(uint32_t*)(psp->PC - 2) & 0xFF;

  switch ( svc_number ) {
    case SVC_SBRK: psp->R0 =  (uint32_t)sys_sbrk ((int)psp->R0); break;
    case SVC_WRITE: psp->R0 = (uint32_t)sys_write(STDOUT_FILENO, (char *)psp->R1, (int)psp->R2); break;
    case SVC_READ: psp->R0 =  (uint32_t)sys_read (STDIN_FILENO,  (char *)psp->R1, (int)psp->R2); break;
    case SVC_EXIT: sys_exit((int)psp->R0); break;
    // case SVC_SERVO_ENABLE: (uint32_t)sys_servo_enable((uint8_t)psp->R0, (uint8_t)psp->R1); break;
    // case SVC_SERVO_SET: (uint32_t)sys_servo_set((uint8_t)psp->R0, (uint8_t)psp->R1); break;
    case SVC_CLOSE: break;
    case SVC_FSTAT: break;
    case SVC_ISATTY: break;
    case SVC_LSEEK: break;

    ///////////////// THREADS ///////////////
    case SVC_THR_INIT: 
      psp->R0 = (uint32_t)sys_thread_init(psp->R0, psp->R1, (void * )psp->R2, psp->R3); 
      break;
    case SVC_THR_CREATE: 
      psp->R0 = 
        (uint32_t)sys_thread_create((void *)psp->R0, psp->R1, psp->R2, psp->R3, (void *)r4);
      break;
    case SVC_SCHD_START: 
      psp->R0 = (uint32_t)sys_scheduler_start(psp->R0);
      break;
    case SVC_PRIORITY: psp->R0 = sys_get_priority(); break;
    case SVC_TIME: psp->R0 = sys_get_time(); break;
    case SVC_THR_TIME: psp->R0 = sys_thread_time(); break;
    case SVC_THR_KILL: sys_thread_kill(); break;
    case SVC_WAIT: sys_wait_until_next_period(); break;

    /////////////////// MUTEXES ////////////////////////
    case SVC_MUT_INIT: psp->R0 = (uint32_t)sys_mutex_init(psp->R0); break;
    case SVC_MUT_LOK: sys_mutex_lock((kmutex_t *)psp->R0); break;
    case SVC_MUT_ULK: sys_mutex_unlock((kmutex_t *)psp->R0); break;
    // MOTOR/ENCODER
    case SVC_SET_MOTOR: psp->R0 = (uint32_t)sys_motor_set((enum motor_mapping) psp->R0,
      psp->R1, (direction_t) psp->R2); break;
    case SVC_REG_ENC_CALLBACK: psp->R0 = (uint32_t)sys_register_encoder_callback(
      psp->R0, (void (*) (uint32_t, uint32_t)) psp->R1
    ); break;

    ////////////////// LCD //////////////
    case SVC_LCD_CLEAR: sys_lcd_clear(); break;

    case SVC_LCD_SET_CURSOR: sys_lcd_set_cursor((uint8_t)psp->R0, (uint8_t)psp->R1); break;

    case SVC_LCD_PRINT: sys_lcd_print((char *)psp->R0); break;

    ////////////// BTN FN /////////////
    case SVC_REG_SPD_CALLBACK: sys_register_spd_callback((void (*) (uint32_t)) psp->R0) ;break;

    default:  DEBUG_PRINT( "Not implemented, svc num %d\n", svc_number );
              ASSERT( 0 );
  }
}