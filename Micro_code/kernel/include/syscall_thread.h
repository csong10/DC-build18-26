/** @file syscall_thread.h
 *
 *  @brief  Custom syscalls to support thread library.
 *
 *  @date   March 27, 2019
 *
 *  @author Ronit Banerjee <ronitb@andrew.cmu.edu>
 */

#ifndef _SYSCALL_THREAD_H_
#define _SYSCALL_THREAD_H_

#include <unistd.h>

/**
 * @brief      Heap high and low pointers.
 */
//@{
extern char
  __thread_u_stacks_low,
  __thread_u_stacks_top,
  __thread_k_stacks_low,
  __thread_k_stacks_top;
//@}

/**
 * @brief      default idle function asm
 */
//@{
extern char
  __def_idle_fn_spin;
//@}

/** @brief sys thread kill assembly stub */
extern char thread_kill;

/** @brief      Initial XPSR value, all 0s except thumb bit. */
#define XPSR_INIT 0x1000000

/** @brief Interrupt return code to user mode using PSP.*/
#define LR_RETURN_TO_USER_PSP 0xFFFFFFFD
/** @brief Interrupt return code to kernel mode using MSP.*/
#define LR_RETURN_TO_KERNEL_MSP 0xFFFFFFF1

/** @brief Max number of threads allocatable.*/
#define MAX_THR 16

/** @brief thread function constants */
#define IDLE_FN_IND 14
#define DEF_IND 15
#define WORD_SZ 4

#define STACK_MIN_SZ 10
#define STACK_MAX_SZ 15

/**
 * @brief      Precalculated values for UB test.
 */
extern float ub_table[];

/**
 * @struct user_stack_frame
 *
 * @brief  Stack frame upon exception.
 */
typedef struct {
  volatile uint32_t r0;   /** @brief Register value for r0 */
  volatile uint32_t r1;   /** @brief Register value for r1 */
  volatile uint32_t r2;   /** @brief Register value for r2 */
  volatile uint32_t r3;   /** @brief Register value for r3 */
  volatile uint32_t r12;  /** @brief Register value for r12 */
  volatile uint32_t lr;   /** @brief Register value for lr*/
  volatile uint32_t pc;   /** @brief Register value for pc */
  volatile uint32_t xPSR; /** @brief Register value for xPSR */
} interrupt_stack_frame;

/**
 * @struct kernel_stack_frame
 * 
 * @brief Kernel stack frame upon entering
 */
typedef struct {
  interrupt_stack_frame *psp;
  volatile uint32_t r4;
  volatile uint32_t r5;
  volatile uint32_t r6;
  volatile uint32_t r7;
  volatile uint32_t r8;
  volatile uint32_t r9;
  volatile uint32_t r10;
  volatile uint32_t r11;
  volatile uint32_t lr;
} kernel_stack_frame;

/**
 * @enum thr_state
 * 
 * @brief thread states: 
 *  WAITING: waiting until period is done
 *  RUNNABLE: can be scheduled to run but not running
 *  RUNNING: thread that is running
 *  DEAD: never should be using this thread
 *  INIT: thread has not been initialized yet even though max_threads isn't fulfilled
 */
typedef enum {
  WAITING, 
  RUNNABLE,
  RUNNING,
  DEAD,
  INIT
} thr_state;

/**
 * @struct TCB_info
 * 
 * @brief holds info for each thread
 */
typedef struct tcb {
  kernel_stack_frame *msp;
  volatile uint32_t priority;  //static priority
  volatile uint32_t period;    //T
  volatile uint32_t comp_time; //C
  volatile uint32_t exec_time;
  volatile uint32_t ttl_wrk; 
  volatile thr_state status;
  volatile int in_svc;
  volatile uint32_t acq_mux; // bit vector for mutex hold
  volatile uint32_t dynamic_prio;
} tcb_t;

/** @brief global vars */
//array containing all thread info
extern tcb_t thread_info[MAX_THR];
//index of the running thread
extern uint8_t running_ind;
//num registered threads
extern uint8_t num_reg_thr;
//user supplied max threads
extern uint32_t user_max_thr;

/**
 * @brief      The PendSV interrupt handler.
 */
void *pendsv_c_handler( void * );

/**
 * @brief      Initialize the thread library
 *
 *             A user program must call this initializer before attempting to
 *             create any threads or starting the scheduler.
 *
 * @param[in]  max_threads        Maximum number of threads that will be
 *                                created.
 * @param[in]  stack_size         Declares the size in words of all user and
 *                                kernel stacks created.
 * @param[in]  idle_fn            Pointer to a thread function to run when no
 *                                other threads are runnable. If NULL is
 *                                is supplied, the kernel will provide its
 *                                own idle function that will sleep.
 * @param[in]  max_mutexes        Maximum number of mutexes that will be
 *                                created.
 *
 * @return     0 on success or -1 on failure
 */
int sys_thread_init(
  uint32_t        max_threads,
  uint32_t        stack_size,
  void           *idle_fn,
  uint32_t        max_mutexes
);

/**
 * @brief      Create a new thread running the given function. The thread will
 *             not be created if the UB test fails, and in that case this function
 *             will return an error.
 *
 * @param[in]  fn     Pointer to the function to run in the new thread.
 * @param[in]  prio   Priority of this thread. Lower number are higher
 *                    priority.
 * @param[in]  C      Real time execution time (scheduler ticks).
 * @param[in]  T      Real time task period (scheduler ticks).
 * @param[in]  vargp  Argument for thread function (usually a pointer).
 *
 * @return     0 on success or -1 on failure
 */
int sys_thread_create( void *fn, uint32_t prio, uint32_t C, uint32_t T, void *vargp );

/**
 * @brief      Allow the kernel to start running the thread set.
 *
 *             This function should enable SysTick and thus enable your
 *             scheduler. It will not return immediately unless there is an error.
 *			   It may eventually return successfully if all thread functions are
 *   		   completed or killed.
 *
 * @param[in]  frequency  Frequency (Hz) of context swaps.
 *
 * @return     0 on success or -1 on failure
 */
int sys_scheduler_start( uint32_t frequency );

/**
 * @brief      Get the current time.
 *
 * @return     The time in ticks.
 */
uint32_t sys_get_time( void );

/**
 * @brief      Get the effective priority of the current running thread
 *
 * @return     The thread's effective priority
 */
uint32_t sys_get_priority( void );

/**
 * @brief      Gets the total elapsed time for the thread (since its first
 *             ever period).
 *
 * @return     The time in ticks.
 */
uint32_t sys_thread_time( void );

/**
 * @brief      Waits efficiently by descheduling thread.
 */
void sys_wait_until_next_period( void );

/**
* @brief      Kills current running thread. Aborts program if current thread is
*             main thread or the idle thread or if current thread exited
*             while holding a mutex.
*
* @return     Does not return.
*/
void sys_thread_kill( void );

/**
 * @brief      Get the current time.
 *
 * @return     The time in ticks.
 */
uint32_t sys_get_time( void );

/**
 * @brief      Get the effective priority of the current running thread
 *
 * @return     The thread's effective priority
 */
uint32_t sys_get_priority( void );

/**
 * @brief      Gets the total elapsed time for the thread (since its first
 *             ever period).
 *
 * @return     The time in ticks.
 */
uint32_t sys_thread_time( void );

/**
 * @brief      Waits efficiently by descheduling thread.
 */
void sys_wait_until_next_period( void );

/**
* @brief      Kills current running thread. Aborts program if current thread is
*             main thread or the idle thread or if current thread exited
*             while holding a mutex.
*
* @return     Does not return.
*/
void sys_thread_kill( void );


//HELPERS
/**
 * @brief     Implements RMS and HLP, returns the next index to run taking into
 *            consideration, (1) if it is runnable and if it has the highest 
 *            priority (lowest index) OR (2) their dynamic priority while 
 *            holding a mutex is greater than or equal to the next highest
 *            priority thread not holding a mutex.
 * 
 * @return    the index of the next thread to run
 */
int find_next_thread_to_run(void);

/**
 * @brief     When unlocking a mutex you need to set the dynamic priority to
 *            the next highest priority of the mutexes it holds if any so that
 *            HLP is upheld.
 * 
 * @return    the priority of the next highest mutex held
 */
void update_dynamic_prio(void);

#endif /* _SYSCALL_THREAD_H_ */