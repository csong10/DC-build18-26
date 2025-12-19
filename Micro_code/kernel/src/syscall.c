/**
 * @file 
 *
 * @brief      
 *
 * @date       
 *
 * @author     
 */

#include <unistd.h>
#include <syscall.h>

//include stuff
#include <lcd_driver.h>
#include <printk.h>
#include <uart.h>
#include <systick.h>
#include <nvic.h>
//

#define UNUSED __attribute__((unused))

//
#define IRQ_NUM 38
extern char __heap_low;
extern char __heap_top;
//

void *sys_sbrk(int incr){
  static char *current_break = &__heap_low;
  char *old_break = current_break;

  //out of memory
  if (current_break + incr > &__heap_top) return (void*) -1;

  current_break += incr;
  return old_break;
}

int sys_write(int file, char *ptr, int len){

  if (file != STDOUT_FILENO) return -1; 

  for (int i = 0; i < len; i++) {
    // Enqueue the byte; wait if buffer is full
    while (uart_put_byte(ptr[i]) == -1);
  }

  return len;
}

int sys_read(int file, char *ptr, int len){

  if (file != STDIN_FILENO) return -1;

  int count = 0;

  while (count < len) {
    char c;
    //wait for char
    while (uart_get_byte(&c) == -1);
    //EOT
    if (c == 4) {
      ptr[count] = '\0';
      count++;
      break;
    }
    //backspace
    else if (c == '\b') {
      if (count > 0) {
        uart_put_byte('\b');
        uart_put_byte(' ');
        uart_put_byte('\b');
        count--;
      }
      continue;
    }
    //new line
    else if (c == '\r' || c == '\n') {
      ptr[count] = '\0'; //apparently need a \0 somewhere
      uart_put_byte(c);
      count++;
      break;
    }
    //normal bytes 
    else {
      ptr[count] = c;
      uart_put_byte(c);
      count++;
    }
  }

  return count;
}

void sys_exit(int status){

  printk("Program exited with status %d\n", status);
  uart_flush();

  sys_lcd_clear();
  sys_lcd_set_cursor(0, 0);
  sys_lcd_print("EXIT CODE:");
  sys_lcd_set_cursor(1, 0);
  sys_lcd_print((char *)status);

  nvic_irq(IRQ_NUM, IRQ_DISABLE);
  
  while (1) {
      systick_delay(5);
  }
}