#include <unistd.h>

#include <gpio.h>
#include <rcc.h>
#include <uart_polling.h>

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

/**
 * @brief initializes UART to given baud rate with 8-bit word length, 1 stop bit, 0 parity bits
 *
 * @param baud Baud rate
 */
void uart_polling_init (int baud){
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
    uart->CR1 |= UART_EN;
    uart->CR1 |= TX_RX_EN;
    uart->BRR = BAUD_BRRFMT;
    return;
}

/**
 * @brief transmits a byte over UART
 *
 * @param c character to be sent
 */
void uart_polling_put_byte (char c){
    struct uart_reg_map *uart = UART2_BASE;
    //bit mask to get transmit data register status
    int send_status = (uart->SR >> 7) & 1;
    while (send_status == 0) {
        send_status = (uart->SR >> 7) & 1;
    }
    uart->DR = c;
    return;
}

/**
 * @brief receives a byte over UART
 */
char uart_polling_get_byte () {
    struct uart_reg_map *uart = UART2_BASE;
    int rx_status = (uart->SR >> 5) & 1;
    while (rx_status == 0) {
        rx_status = (uart->SR >> 5) & 1;
    }
    return uart->DR;
}

/**
 * @brief sends a string constant using uart polling
 * @param[in] s the string to be sent
 */
void uart_send_string(const char *s) {
    for (int i = 0; s[i] != '\0'; i++) {
        uart_polling_put_byte(s[i]);
    }
}

/**
 * @brief sends a uint16_t in decimal using uart polling
 * @param[in] num 16 bit unsigned integer assumed to be value read from adc
 */
void uart_send_uint(uint16_t num) {
    char digits[5];
    int i = 0;

    if (num == 0) {
        uart_polling_put_byte('0');
        return;
    }

    while (num > 0) {
        int digit = num % 10;

        // adding a '0' will convert it to ascii
        digits[i] = digit + '0';
        num = num  / 10;
        i++;
    }

    // least significant digit is stored in the beginning
    for (int j = i - 1; j >= 0; j++) {
        uart_polling_put_byte(digits[j]);
    }
}