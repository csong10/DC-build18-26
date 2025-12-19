/**
 * @file 
 *
 * @brief      
 *
 * @date       
 *
 * @author     
 */

#ifndef _UART_H_
#define _UART_H_

#include <unistd.h>

//helper functions
int chk_tx_full();
int chk_tx_empty();
int tx_enq(char c);
int tx_deq(char *c);

int chk_rx_full();
int chk_rx_empty();
int rx_enq(char c);
int rx_deq(char *c);

void uart_init(int baud);

int uart_put_byte(char c);

int uart_get_byte(char *c);

int uart_write( int file, char *ptr, int len );

int uart_read(int file, char *ptr, int len );

void uart_flush();

void uart_irq_handler();

#endif /* _UART_H_ */
