/** @file   syscall_thread.c
 *
 *  @brief  
 *
 *  @date  11/19/25
 *
 *  @author Caleb Song and Andrew Liu
 */

#include <stdint.h>
#include "syscall_thread.h"
#include "syscall_mutex.h"
#include <systick.h>
#include <mpu.h>
#include <arm.h>
#include <syscall.h>
#include <printk.h>

float ub_table[] = {
  0.000, 1.000, .8284, .7798, .7568,
  .7435, .7348, .7286, .7241, .7205,
  .7177, .7155, .7136, .7119, .7106,
  .7094, .7083, .7075, .7066, .7059,
  .7052, .7047, .7042, .7037, .7033,
  .7028, .7025, .7021, .7018, .7015,
  .7012, .7009
};

//tcb array
tcb_t thread_info[MAX_THR];
//index of the running thread
uint8_t running_ind;
//num registered threads
uint8_t num_reg_thr;
//user supplied max threads
uint32_t user_max_thr;

//mutex array def
kmutex_t mux_arr[MAX_MUX];
//number registered mutexes
uint32_t num_reg_mux;
//user supplied max mutexes
uint32_t user_max_mux;

//local variable
//holding the rounded stack size
uint32_t round_stk_sz;

void *pendsv_c_handler(void *context_ptr){
  //if there are no more threads to run, switch to the default thread
  //shutting off the systick & setting svc and stuff
  if (num_reg_thr == 0) {
    systick_off();
    running_ind = DEF_IND;
    thread_info[DEF_IND].status = RUNNING;
    (thread_info[DEF_IND].in_svc) ? set_svc_status(1) : set_svc_status(0);

    return (void *)thread_info[DEF_IND].msp;
  }

  //SAVE current context including whether it's in priveleged or unprivileged mode
  thread_info[running_ind].in_svc = get_svc_status();
  thread_info[running_ind].msp = (kernel_stack_frame *) context_ptr;

  //find next thread to run
  int next_ind = find_next_thread_to_run();

  //if same thread, no switch needed
  if (next_ind == running_ind) {
      running_ind = next_ind;
      thread_info[running_ind].status = RUNNING;
      return context_ptr; // return same stack
  }

  //load next threads kernel stack pointer
  kernel_stack_frame *next_context_ptr = thread_info[next_ind].msp;

  //update current running thread
  running_ind = next_ind;
  thread_info[running_ind].status = RUNNING;
  (thread_info[running_ind].in_svc) ? set_svc_status(1) : set_svc_status(0);

  return (void *)next_context_ptr;
}

int sys_thread_init(
  uint32_t max_threads,
  uint32_t stack_size,
  void *idle_fn,
  uint32_t max_mutexes
){
  //if max user supplied threads is greater than 14 not possible
  if (max_threads > MAX_THR - 2) return -1;

  uint32_t stk_sz_byte = stack_size * WORD_SZ;
  uint32_t stack_exp = mm_log2ceil_size(stk_sz_byte);
  round_stk_sz = (1 << stack_exp);

  //stack size is not between 1kb and 32kb
  if (!((stack_exp >= STACK_MIN_SZ) && (stack_exp <= STACK_MAX_SZ))) return -1;

  //checks if stacks fit inside the allocatable space
  if ((&__thread_u_stacks_low + (round_stk_sz*(max_threads+2)) > &__thread_u_stacks_top) ||
      (&__thread_k_stacks_low + (round_stk_sz*(max_threads+2)) > &__thread_k_stacks_top)) 
      return -1;
  
  //initialize global vars
  running_ind = DEF_IND; 
  //should this be initialized to something else and then do logic on it?
  num_reg_thr = 0;
  user_max_thr = max_threads;
  num_reg_mux = 0;
  user_max_mux = max_mutexes;
  
  //initialize the thread blocks
  for (uint32_t i = 0; i < max_threads + 2; i++) {
    uint32_t user_addr = (uint32_t)(&__thread_u_stacks_top) - (i * round_stk_sz) - sizeof(interrupt_stack_frame);
    uint32_t kernel_addr = (uint32_t)(&__thread_k_stacks_top) - (i * round_stk_sz) - sizeof(kernel_stack_frame);

    if (i == max_threads) {
      thread_info[IDLE_FN_IND].msp = (kernel_stack_frame *)kernel_addr;
      thread_info[IDLE_FN_IND].msp->psp = (interrupt_stack_frame *)user_addr;

      if (!idle_fn) {
        thread_info[IDLE_FN_IND].status = RUNNABLE;
        thread_info[IDLE_FN_IND].msp->psp->pc = (uint32_t) &__def_idle_fn_spin;
      } else {
        thread_info[IDLE_FN_IND].status = RUNNABLE;
        thread_info[IDLE_FN_IND].msp->psp->pc = (uint32_t) idle_fn;
      }

      thread_info[IDLE_FN_IND].msp->psp->xPSR = XPSR_INIT;
      thread_info[IDLE_FN_IND].msp->psp->r0 = 0;
      thread_info[IDLE_FN_IND].msp->psp->r1 = 0;
      thread_info[IDLE_FN_IND].msp->psp->r2 = 0;
      thread_info[IDLE_FN_IND].msp->psp->r3 = 0;
      thread_info[IDLE_FN_IND].msp->psp->r12 = 0;
      thread_info[IDLE_FN_IND].msp->psp->lr = (uint32_t) &thread_kill;

      thread_info[IDLE_FN_IND].msp->lr = LR_RETURN_TO_USER_PSP;   
      thread_info[IDLE_FN_IND].msp->r4 = 0;
      thread_info[IDLE_FN_IND].msp->r5 = 0;
      thread_info[IDLE_FN_IND].msp->r6 = 0;
      thread_info[IDLE_FN_IND].msp->r7 = 0;
      thread_info[IDLE_FN_IND].msp->r8 = 0;
      thread_info[IDLE_FN_IND].msp->r9 = 0;
      thread_info[IDLE_FN_IND].msp->r10 = 0;
      thread_info[IDLE_FN_IND].msp->r11 = 0;
      thread_info[IDLE_FN_IND].dynamic_prio = IDLE_FN_IND;

    } else if (i == max_threads+1) {
      thread_info[DEF_IND].msp = (kernel_stack_frame *)kernel_addr;
      thread_info[DEF_IND].msp->psp = (interrupt_stack_frame *)user_addr;
      thread_info[DEF_IND].status = RUNNABLE;
      thread_info[DEF_IND].msp->psp->lr = (uint32_t) &thread_kill;
      thread_info[DEF_IND].dynamic_prio = DEF_IND;

    } else {
      thread_info[i].msp = (kernel_stack_frame *)kernel_addr;
      thread_info[i].msp->psp = (interrupt_stack_frame *) user_addr;
      thread_info[i].status = INIT;
    }

    thread_info[i].in_svc = 0;
    thread_info[i].acq_mux = 0;
  }

  //mark rest as dead
  for (uint32_t i = max_threads; i < MAX_THR-2; i++) {
    thread_info[i].status = DEAD;
    thread_info[i].in_svc = 0;
  }

  //initialize mutex array
  for (uint32_t i = 0; i < MAX_MUX; i++) {
    if (i < max_mutexes)
      mux_arr[i].locked_by = NOT_LOCKED;
    else 
      mux_arr[i].locked_by = NO_INIT;
  }
  
  return 0;
}

int sys_thread_create(
  void *fn,
  uint32_t prio,
  uint32_t C,
  uint32_t T,
  void *vargp
){
  //the thread was created already
  if ((thread_info[prio].status != INIT) && (thread_info[prio].status != DEAD)) return -1;
  //max user threads reached
  if (num_reg_thr == user_max_thr) return -1; 

  if (num_reg_thr != 0) {
    uint8_t tmp_cnt = num_reg_thr;
    float ub_sum = 0;

    for (uint32_t i = 0; i < MAX_THR - 2; i++) {
      if (tmp_cnt == 0) break;
      if ((thread_info[i].status != INIT) && (thread_info[i].status != DEAD)) {
        ub_sum += (float) thread_info[i].comp_time / (float) thread_info[i].period;
        tmp_cnt--;
      }
    }

    ub_sum += (float) C / (float) T;
    //cannot be scheduled
    if (ub_sum > ub_table[num_reg_thr+1]) return -1;

  }
  //filling in the TB
  //have to reinitialize this because sometimes when systhreadkill is called
  //its called from msp or when its priveleged and it loses the psp
  thread_info[prio].msp = (kernel_stack_frame *) 
    ((uint32_t)(&__thread_k_stacks_top) - (prio * round_stk_sz) - sizeof(kernel_stack_frame));
  thread_info[prio].msp->psp = (interrupt_stack_frame *)
    ((uint32_t)(&__thread_u_stacks_top) - (prio * round_stk_sz) - sizeof(interrupt_stack_frame));

  thread_info[prio].comp_time = C;
  thread_info[prio].exec_time = 0;
  thread_info[prio].ttl_wrk = 0;
  thread_info[prio].period = T;
  thread_info[prio].priority = prio;
  thread_info[prio].dynamic_prio = prio;
  thread_info[prio].msp->psp->pc = (uint32_t) fn;
  thread_info[prio].msp->psp->r0 = (uint32_t) vargp;
  thread_info[prio].msp->psp->lr = (uint32_t) &thread_kill;
  thread_info[prio].status = RUNNABLE;

  thread_info[prio].msp->psp->xPSR = XPSR_INIT;
  thread_info[prio].msp->psp->r1 = 0;
  thread_info[prio].msp->psp->r2 = 0;
  thread_info[prio].msp->psp->r3 = 0;
  thread_info[prio].msp->psp->r12 = 0;

  thread_info[prio].msp->lr = LR_RETURN_TO_USER_PSP;   
  thread_info[prio].msp->r4 = 0;
  thread_info[prio].msp->r5 = 0;
  thread_info[prio].msp->r6 = 0;
  thread_info[prio].msp->r7 = 0;
  thread_info[prio].msp->r8 = 0;
  thread_info[prio].msp->r9 = 0;
  thread_info[prio].msp->r10 = 0;
  thread_info[prio].msp->r11 = 0;

  thread_info[prio].in_svc = 0;
  num_reg_thr++;
  return 0;
}

int sys_scheduler_start( uint32_t frequency ){
  systick_init(frequency);

  pend_pendsv();
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// RMS/HLP scheduler
int find_next_thread_to_run() {

    uint32_t lowestP = UINT32_MAX;
    //highest dynamic priority
    uint32_t highestDP = UINT32_MAX;
    int nextind = IDLE_FN_IND;
    int nextmuxind = DEF_IND;
    int returnind = IDLE_FN_IND;

    //find next runnable thread
    for (uint32_t i = 0; i < MAX_THR; i++) {
      if ((thread_info[i].status == RUNNABLE) && (i != IDLE_FN_IND) && (i != DEF_IND)) {
        //\/ this will go all the way to the lowest index that has a high dynamic priority.
        //and has a mutex
        if (thread_info[i].dynamic_prio <= highestDP && thread_info[i].acq_mux != 0) {
          highestDP = thread_info[i].dynamic_prio;
          nextmuxind = i;
        }
        if (thread_info[i].period < lowestP) {
          lowestP = thread_info[i].period;
          nextind = i;
        }
      }
    }

    //if the nextind (lowest period) has higher prio, context switch to that
    //doesn't matter because they don't ever hold that mutex
    if (thread_info[nextind].dynamic_prio < thread_info[nextmuxind].dynamic_prio) {
      returnind = nextind;

    //if the nextmuxind has the same prio as the next index (static prio == dynamic prio)
    //make the return index the nextmuxind because the nextmuxind was set because
    //the nextmuxind holds the mutex and now their dynamic priority has boosted up to the ceiling
    } else if (thread_info[nextind].dynamic_prio == thread_info[nextmuxind].dynamic_prio) {
      returnind = nextmuxind;
    } else {
      returnind = nextmuxind;
    }

    //if after checking all, you have to run the idle thread but the idle thread
    //was killed, you run default thread.
    if (returnind == IDLE_FN_IND) {
      returnind = (thread_info[IDLE_FN_IND].status == INIT) ? DEF_IND : IDLE_FN_IND;
    }
    //if no runnable threads, run idle thread
    return returnind;
}
////////////////////////////////////////////////////////////////////////////////

void update_dynamic_prio() {
  uint32_t tmp_mutex_stat = thread_info[running_ind].acq_mux;
  uint32_t curr_mux_ind = 0;
  uint32_t lowestDP = DEF_IND;
  //goes through the bit vector and updates lowestDP to the next highest
  //priority of the next mutex
  while (tmp_mutex_stat != 0) {
    if ((tmp_mutex_stat & 1) != 0) {
      if (mux_arr[curr_mux_ind].prio_ceil < lowestDP)
        lowestDP = mux_arr[curr_mux_ind].prio_ceil;
    }
    curr_mux_ind++;
    tmp_mutex_stat = tmp_mutex_stat >> 1;
  }

  //updates the current dynamic priority to the next highest ceiling
  //otherwise set it to its normal priority
  //lowestDP will NEVER be greater than the priority of the acquisitioner
  //this is because of the check in mutex init
  if (lowestDP < thread_info[running_ind].priority) {
    thread_info[running_ind].dynamic_prio = lowestDP;
  } else {
    thread_info[running_ind].dynamic_prio = thread_info[running_ind].priority;
  }
}

uint32_t sys_get_priority(){
  if (thread_info[running_ind].priority == thread_info[running_ind].dynamic_prio)
    return thread_info[running_ind].priority;
  //the dynamic priority is initialized to be equal to the priority at the beg
  //if its changed its during mutex locking and unlocking
  else
    return thread_info[running_ind].dynamic_prio;
}

uint32_t sys_get_time(){
  return systick_get_ticks();
}

uint32_t sys_thread_time(){
  return thread_info[running_ind].ttl_wrk;
}

void sys_thread_kill(){
  //if thread is idle
  if (running_ind == IDLE_FN_IND) {
    thread_info[IDLE_FN_IND].msp->psp->pc = (uint32_t) &__def_idle_fn_spin;

  //default thread calls it then abort and exit system
  } else if (running_ind == DEF_IND) {
    sys_exit(thread_info[DEF_IND].status);

  //non default or idle thread called it
  } else {
    thread_info[running_ind].status = INIT;
    num_reg_thr--;
  }
  pend_pendsv();
}

void sys_wait_until_next_period(){
  if (thread_info[running_ind].acq_mux) {
    printk("Warning: sleeping thread still holds a mutex...\n");
  }
  thread_info[running_ind].status = WAITING;
  thread_info[running_ind].exec_time = 0;
  pend_pendsv();
}

kmutex_t *sys_mutex_init( uint32_t max_prio ) {
  if (mux_arr[num_reg_mux].locked_by == NOT_LOCKED) {
    uint32_t tmp = num_reg_mux;
    mux_arr[num_reg_mux].index = num_reg_mux;
    mux_arr[num_reg_mux].prio_ceil = max_prio;
    num_reg_mux++;
    return &mux_arr[tmp];
    
  } else //number registered muxes is greater than max muxes
    return NULL;
}

void sys_mutex_lock( kmutex_t *mutex ) {
  if (thread_info[running_ind].priority < mutex->prio_ceil) {
    printk("Error: Higher prio calling mutex with lower prio, killing thread...\n");
    sys_thread_kill();

  } else if (mutex->locked_by != NOT_LOCKED) {
    printk("Warning: trying to lock a locked mutex...\n");
    thread_info[running_ind].status = RUNNABLE;
    pend_pendsv();

  } else if (running_ind != IDLE_FN_IND) {  //edit mutex info
    thread_info[running_ind].acq_mux |= (1 << (mutex->index));

    if (thread_info[running_ind].dynamic_prio > mutex->prio_ceil)
      thread_info[running_ind].dynamic_prio = mutex->prio_ceil;

    mutex->locked_by = running_ind;
  }
}

void sys_mutex_unlock( kmutex_t *mutex ) {
  if (mutex->locked_by == NOT_LOCKED) {
    printk("Warning: trying to unlock an unlocked mutex...\n");
    return;
  } else {
    //clears the mutex acquired
    thread_info[running_ind].acq_mux &= ~(1 << (mutex->index));
    update_dynamic_prio();
    mutex->locked_by = NOT_LOCKED;

    thread_info[running_ind].status = RUNNABLE;
    pend_pendsv();
  }
}
