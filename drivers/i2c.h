#ifndef I2C_H
#define I2C_H

#include "msp432.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "macro.h"

#define DEFAULT_SLAVE_ADDRESS (0x29)

#define EUSCI_SEL EUSCI_B1
#define I2C_SCL_PORT 6
#define I2C_SCL_PIN 5
#define I2C_SDA_PORT 6
#define I2C_SDA_PIN 4


void i2c_init(void);

void i2c_set_slave_address(uint8_t addr);

bool i2c_read_core(const uint16_t addr, uint8_t addr_len, uint8_t *data, uint16_t data_len);

bool i2c_write_core(const uint16_t addr, uint8_t addr_len, const uint8_t *data, uint16_t data_len);



#endif /* I2C_H */
