/**
 * @file i2c.c
 *
 * @brief implementing the i2c of the lab 2
 *
 * @date 9/28/27
 *
 * @author Caleb Song calebson Andrew Liu andrewl5
 */
#include <unistd.h>

#include <gpio.h>
#include <i2c.h>
#include <rcc.h>
#include <printk.h>

/** @brief i2c register map */
struct i2c_reg_map {
    volatile uint32_t CR1;      /**< Control Register 1 */
    volatile uint32_t CR2;      /**< Control Register 2 */
    volatile uint32_t OAR1;     /**< Own Address Register 1 */
    volatile uint32_t OAR2;     /**< Own Address Register 2 */ 
    volatile uint32_t DR;       /**< Data Register */
    volatile uint32_t SR1;      /**< Status Register 1 */
    volatile uint32_t SR2;      /**< Status Register 2 */
    volatile uint32_t CCR;      /**< Clock Control Register */
    volatile uint32_t TRISE;    /**< TRISE Register */
    volatile uint32_t FLTR;     /**< FLTR Register */
};

/** @brief initialization constants */
#define I2C1_BASE (struct i2c_reg_map *) 0x40005400
#define I2C_CLK_EN (1 << 21)
#define I2C_FREQ (1 << 4)
#define I2C_ACK_EN (1 << 10)
#define I2C_CCR 0xA0
#define I2C_PWR 1

/** @brief port constants */
#define SCL_PORT GPIO_B
#define SCL_PIN  8   
#define SDA_PORT GPIO_B
#define SDA_PIN  9

/** @brief I2C control/status constants */
#define I2C_START (1 << 8)
#define I2C_STOP (1 << 9)
#define I2C_TXE (1 << 7)
#define I2C_BTF (1 << 2)
#define I2C_ADDR (1 << 1)

#define I2C_BUSY (1 << 1)
#define I2C_MSB 1


/**
 * @brief this initializes the i2c device to be in master mode.
 * @param[in] clk but scl clock is hard set to operate at 100kHz
 */
void i2c_master_init(uint16_t clk){
    (void) clk; 

    gpio_init(SCL_PORT, SCL_PIN, MODE_ALT, 
              OUTPUT_OPEN_DRAIN, OUTPUT_SPEED_LOW,
              PUPD_PULL_UP, ALT4);

    gpio_init(SDA_PORT, SDA_PIN, MODE_ALT, 
              OUTPUT_OPEN_DRAIN, OUTPUT_SPEED_LOW,
              PUPD_PULL_UP, ALT4);

    struct rcc_reg_map *rcc = RCC_BASE;
    rcc->apb1_enr |= I2C_CLK_EN;
    
    struct i2c_reg_map *i2c = I2C1_BASE;
    i2c->CR2 |= I2C_FREQ; 
    i2c->CCR |= I2C_CCR; 
    i2c->CR1 |= I2C_ACK_EN;
    i2c->CR1 |= I2C_PWR; 
    
    return;
}

/**
 * @brief sends the START sequence for data to be written or read on SDA
 */
void i2c_master_start() {
    struct i2c_reg_map *i2c = I2C1_BASE;

    //wait for not busy
    while (((i2c->SR2) & I2C_BUSY));

    i2c->CR1 |= I2C_START;

    //wait for EV5
    while (((i2c->SR1) & 1) == 0);

    return; 
}   

/**
 * @brief sends the STOP sequence on SDA
 */
void i2c_master_stop() {
    struct i2c_reg_map *i2c = I2C1_BASE;

    //wait for EV8_2 (either TxE or BTF is set)
    while (!(((i2c->SR1) & I2C_TXE) || ((i2c->SR1) & I2C_BTF)));

    i2c->CR1 |= I2C_STOP;

    //master -> slave mode
    while (((i2c->SR2) & I2C_MSB) == I2C_MSB);

    return;
}

/**
 * @brief writes desired data onto SDA line to a specified register/device.
 * ASSUMES: i2c_master_init has already been called
 * @param[in] buf the different 8 bit data packets to be written
 * @param[in] len the number of 8 bit packets to be written
 * @param[in] slave_addr the address of the slave device to be written to
 * @returns 0 if exited without error 
 */
int i2c_master_write(uint8_t *buf, uint16_t len, uint8_t slave_addr){

    struct i2c_reg_map *i2c = I2C1_BASE;
    i2c_master_start();

    i2c->DR = (slave_addr << 1);

    //check EV6 (check ADDR)
    while (!((i2c->SR1) & I2C_ADDR));

    //check ev8_1
    while (!((i2c->SR1) & I2C_TXE));
    
    //need to read from sr2 after to properly move onto ev8
    i2c->SR2;

    for (uint16_t i = 0; i < len; i++) {
        i2c->DR = buf[i];

        //check EV8
        while (!((i2c->SR1) & I2C_TXE));
    }

    i2c_master_stop();

    return 0;
}

/**
 * @brief reads desired data from SDA line but not needed to be implemented
 * ASSUMES: i2c_master_init has already been called
 * @param[in] buf the different 8 bit data packets to be read
 * @param[in] len the number of 8 bit packets to be read
 * @param[in] slave_addr the address of the slave device to be read from
 * @returns 0 if exited without error 
 */
int i2c_master_read(uint8_t *buf, uint16_t len, uint8_t slave_addr){
    (void) buf;
    (void) len;
    (void) slave_addr;

    return 0;
}
