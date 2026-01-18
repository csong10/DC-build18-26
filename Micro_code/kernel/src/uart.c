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
#include <rcc.h>
#include <uart.h>
#include <uart_polling.h>
#include <motor_driver.h>
#include <encoder.h>
#include <pwm.h>
#include <nvic.h>
#include <gpio.h>
#include <arm.h>

/** @brief The UART register map. */
struct uart_reg_map {
    volatile uint32_t SR;   /**< Status Register */
    volatile uint32_t DR;   /**<  Data Register */
    volatile uint32_t BRR;  /**<  Baud Rate Register */
    volatile uint32_t CR1;  /**<  Control Register 1 */
    volatile uint32_t CR2;  /**<  Control Register 2 */
    volatile uint32_t CR3;  /**<  Control Register 3 */
    volatile uint32_t GTPR; /**<  Guard Time and Prescaler Register */
};

/** @brief Base address for UART2 */
#define UART2_BASE  (struct uart_reg_map *) 0x40004400

/** @brief Enable  Bit for UART Config register */
#define UART_EN (1 << 13)

/** @brief Enable Bit for clock enable */
#define APB1_EN (1 << 17)

/** @brief precomputed USART_DIV value == 8.6875*/
#define BAUD_BRRFMT 0x8B

/** @brief tx & rx enable */
#define TX_RX_EN (3 << 2)
#define TXE_INT (1 << 7)
#define RXNE_INT (1 << 5)

#define TX_EMP (1 << 7)
#define RX_NEMP (1 << 5)

#define MAX_BUF_SZ 64

#define UART2_IRQ_NUM 38

#define MIN_DUTY_CYCLE 15

volatile char tx_buf [MAX_BUF_SZ];
volatile char rx_buf [MAX_BUF_SZ];
volatile int tx_enq_pos, tx_deq_pos;
volatile int rx_enq_pos, rx_deq_pos;
volatile int tx_size;
volatile int rx_size;

int chk_tx_full(){
  return (tx_size == MAX_BUF_SZ);
}

int chk_rx_full(){
  return (rx_size == MAX_BUF_SZ);
}

int chk_tx_empty() {
  return (tx_size == 0);
}

int chk_rx_empty() {
  return (rx_size == 0);
}

int tx_enq(char c) {
  if (chk_tx_full()) return -1;
  tx_buf[tx_enq_pos] = c;
  tx_enq_pos = (tx_enq_pos+1) % MAX_BUF_SZ;
  tx_size++;
  return 0;
}

int tx_deq(char *c) {
  if (chk_tx_empty()) return -1;
  *c = tx_buf[tx_deq_pos];
  tx_deq_pos = (tx_deq_pos+1) % MAX_BUF_SZ;
  tx_size--;
  return 0;
}

int rx_enq(char c) {
  if (chk_rx_full()) return -1;
  rx_buf[rx_enq_pos] = c;
  rx_enq_pos = (rx_enq_pos+1) % MAX_BUF_SZ;
  rx_size++;
  return 0;
}

int rx_deq(char *c) {
  if (chk_rx_empty()) return -1;
  *c = rx_buf[rx_deq_pos];
  rx_deq_pos = (rx_deq_pos+1) % MAX_BUF_SZ;
  rx_size--;
  return 0;
}

void uart_init(int baud){
  //this is for predefined value of 115200 baudrate
  (void) baud;

  //tx init
  gpio_init(GPIO_A, 2, MODE_ALT, 
            OUTPUT_PUSH_PULL, OUTPUT_SPEED_LOW,
            PUPD_NONE, ALT7);
  //rx init
  gpio_init(GPIO_A, 3, MODE_ALT, 
            OUTPUT_OPEN_DRAIN, OUTPUT_SPEED_LOW,
            PUPD_NONE, ALT7);

  struct rcc_reg_map *rcc = RCC_BASE;
  rcc->apb1_enr |= APB1_EN;

  struct uart_reg_map *uart = UART2_BASE;
  uart->BRR = BAUD_BRRFMT;

  for (int i = 0; i < MAX_BUF_SZ; i++) {
    tx_buf[i] = '\0';
    rx_buf[i] = '\0';
  }
  tx_enq_pos = 0;
  tx_deq_pos = 0;
  rx_enq_pos = 0;
  rx_deq_pos = 0;
  tx_size = 0;
  rx_size = 0;

  uart->CR1 |= UART_EN;
  uart->CR1 |= TX_RX_EN;
  //enable receiving not empty interrupt because our read buffer is empty
  uart->CR1 |= RXNE_INT;

  nvic_irq(UART2_IRQ_NUM, IRQ_ENABLE);
  return;
}

int uart_put_byte(char c){
  int state = save_interrupt_state_and_disable();

  if (tx_enq(c) == -1)  {
    restore_interrupt_state(state);
    return -1;
  }
  struct uart_reg_map *uart = UART2_BASE;
  if (tx_size >= 1) uart->CR1 |= TXE_INT;

  restore_interrupt_state(state);
  return 0;
}

int uart_get_byte(char *c){
  int state = save_interrupt_state_and_disable();
  if (rx_deq(c) == -1) {
    restore_interrupt_state(state);
    return -1;
  }
  struct uart_reg_map *uart = UART2_BASE;
  if (rx_size >= (MAX_BUF_SZ - 2)) uart->CR1 |= RXNE_INT;//

  restore_interrupt_state(state);
  return 0;
}

void uart_irq_handler(){
  int state = save_interrupt_state_and_disable();
  struct uart_reg_map *uart = UART2_BASE;
  if (((uart->SR & TX_EMP) != 0) && !chk_tx_empty()) {
    char c;
    if (!tx_deq(&c)) {
      uart->DR = c;
    }
    if (chk_tx_empty()) {
      uart->CR1 &= (~TXE_INT);
    }
  }
  if (((uart->SR & RX_NEMP) != 0) && !chk_rx_full()) {
    char c = uart->DR;

    //set motor direction as soon as it receives UART from serial port
    switch(c) {
      case 'f':
        sys_motor_set(LF_MOTOR, MIN_DUTY_CYCLE, FORWARD);
        break;
      case 'b':
        sys_motor_set(LF_MOTOR, MIN_DUTY_CYCLE, BACKWARD);
        break;
      case 'r': 
        sys_motor_set(LF_MOTOR, MIN_DUTY_CYCLE, RIGHT);
        break;
      case 'l':
        sys_motor_set(LF_MOTOR, MIN_DUTY_CYCLE, LEFT);
        break;
      case 'x':
        sys_motor_set(LF_MOTOR, MIN_DUTY_CYCLE, FREE);
        break;
      case 's': 
        sys_motor_set(LF_MOTOR, MIN_DUTY_CYCLE, STOP);
    }

    rx_enq(c);
    if (chk_rx_full()) uart->CR1 &= (~RXNE_INT);
  }
  nvic_clear_pending(UART2_IRQ_NUM);
  restore_interrupt_state(state);
}

void uart_flush(){
  int state = save_interrupt_state_and_disable();
  struct uart_reg_map *uart = UART2_BASE;
  while (!chk_tx_empty()) {
    char c;
    if (tx_deq(&c) == -1) continue;
    //bit mask to get transmit data register status
    int send_status = (uart->SR >> 7) & 1;
    while (send_status == 0) {
        send_status = (uart->SR >> 7) & 1;
    }
    uart->DR = c;
  }
  restore_interrupt_state(state);
}