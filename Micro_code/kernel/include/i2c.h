/** @file i2c.h
 *
 *  @brief  function headers for I2C protocol 
 *
 *  @date   January 7, 2026
 *
 *  @author Caleb Song
 */
#ifndef _I2C_H_
#define _I2C_H_

#include <stdint.h>

void i2c_master_init(uint16_t clk);

void i2c_master_start();

void i2c_master_stop();

int i2c_master_write(uint8_t *buf, uint16_t len, uint8_t slave_addr);

int i2c_master_read(uint8_t *buf, uint16_t len, uint8_t slave_addr);

#endif /* _I2C_H_ */