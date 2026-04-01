#ifndef I2C_H
#define I2C_H

#include "msp432.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define DEFAULT_SLAVE_ADDRESS (0x29)

/* Updated to the MSP432 CMSIS peripheral pointer */
#define EUSCI_SEL EUSCI_B1
#define I2C_SCL_PORT 6
#define I2C_SCL_PIN 5
#define I2C_SDA_PORT 6
#define I2C_SDA_PIN 4

#define BIT_PIN(pin) (BIT## pin)

/* Removed the invalid USCI_REG_VAR, GLUE2_VAR, and USCI_REG_PTR macros */

void i2c_init(void);

void i2c_set_slave_address(uint8_t addr);

bool start_transfer(uint16_t addr, uint8_t addr_len);

void stop_transfer();

bool i2c_read_core(const uint16_t addr, uint8_t addr_len, uint8_t *data, uint16_t data_len);

bool i2c_write_core(const uint16_t addr, uint8_t addr_len, const uint8_t *data, uint16_t data_len);

/* Note: Removed the duplicate i2c_init and i2c_set_slave_address declarations 
   from the end of this file to resolve the "#174-D linkage conflict" error */

#endif /* I2C_H */