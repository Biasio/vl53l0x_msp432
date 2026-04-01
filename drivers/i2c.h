#ifndef I2C_H
#define I2C_H

#ifndef __MSP432P401R__
    #define __MSP432P401R__
#endif
#include "msp432.h"
#include <stdint.h>
#include <stdbool.h>

#define DEFAULT_SLAVE_ADDRESS (0x29)
#define EUSCI_SEL UCB1

#define USCI_REG(usci,reg)     GLUE2(usci, reg) 
#define GLUE2(a, b)       a ## b

void i2c_init(void);

void i2c_set_slave_address(uint8_t addr);

bool start_transfer(uint16_t addr, uint8_t addr_len);

void stop_transfer();

bool i2c_read_core(const uint16_t addr, uint8_t addr_len, uint8_t *data, uint16_t data_len);

bool i2c_write_core(const uint16_t addr, uint8_t addr_len, const uint8_t *data, uint16_t data_len);

void i2c_init();

void i2c_set_slave_address(uint8_t addr);

#endif /* I2C_H */