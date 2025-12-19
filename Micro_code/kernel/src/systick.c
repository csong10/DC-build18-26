/**
* @file
*
* @brief
*
* @date 10/15/2025
*
* @author Caleb Song calebson Andrew Liu andrewl5
*/

#include <unistd.h>
#include <systick.h>
#include <syscall_thread.h>
#include <arm.h>
#include <printk.h>

struct systick_reg_map {
    volatile uint32_t CTRL;
    volatile uint32_t LOAD;
    volatile uint32_t VAL;
    volatile uint32_t CALIB;
};

//CONSTANTS
#define SYSTICK_BASE (struct systick_reg_map *) 0xE000E010
#define CLK_SRC 0x4
#define TICKINT_EN 0x2
#define STK_EN 0x1

//global vars
volatile uint32_t ms_ticks;

void systick_init(uint32_t frequency) {
    struct systick_reg_map *systick = SYSTICK_BASE;
    ms_ticks = 0;

    //per millisecond
    if (frequency) systick->LOAD = (BASE_FREQ / frequency) - 1;
    else systick->LOAD = (BASE_FREQ / 1000) - 1;

    systick->CTRL |= (TICKINT_EN | STK_EN | CLK_SRC);
}

void systick_delay(uint32_t ticks) {
    //start holds initial value it started at
    uint32_t start = ms_ticks;
    //ms_ticks will keep incrementing and the difference = num of ticks passed
    while ((ms_ticks - start) < ticks);
}

uint32_t systick_get_ticks() {
    return ms_ticks;
}

void systick_c_handler() {
    ms_ticks++;

    //setting status from waiting to runnable and vice versa
    if (thread_info[running_ind].status == RUNNING) {
        thread_info[running_ind].exec_time++;
        thread_info[running_ind].ttl_wrk++;
    }

    //if the execution time is the same as the comp time then we are done and it can wait until period
    if ((thread_info[running_ind].exec_time == thread_info[running_ind].comp_time) &&
        (running_ind != DEF_IND && running_ind != IDLE_FN_IND) ) {
        thread_info[running_ind].status = WAITING;
        thread_info[running_ind].exec_time = 0;

        //if by the time it is done it still has a mutex then print warning
        if (thread_info[running_ind].acq_mux) {
            printk("Warning: finished comp time but still holds mutex...\n");
        }
    }
    else thread_info[running_ind].status = RUNNABLE;

    for (int i = 0; i < MAX_THR; i++) {
        //skip any threads that are dead or not initialized
        if (thread_info[i].status == INIT || thread_info[i].status == DEAD) 
            continue;
        
        //update relevant threads from waiting to runnable status
        uint32_t pd = thread_info[i].period;
        if (((ms_ticks % pd) == 0) && (thread_info[i].status == WAITING))
            thread_info[i].status = RUNNABLE;
    }

    pend_pendsv();
}

void systick_off () {
    struct systick_reg_map *systick = SYSTICK_BASE;
    ms_ticks = 0;
    systick->CTRL &= ~STK_EN;
}