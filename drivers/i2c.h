#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include <stdbool.h>

#define DEFAULT_SLAVE_ADDRESS (0x29)
#define EUSCI_SEL UCB1

#define USCI_REG(usci,reg)     GLUE2(usci, reg) 
#define GLUE2(a, b)       a ## b

void i2c_init(void);
void i2c_set_slave_address(uint8_t addr);

/* =========================================================================
 * READ WRAPPERS 
 * Note: Multi-byte reads expect MSB first over the I2C bus.
 * Note: All functions are polling-based.
 * ========================================================================= */
bool i2c_read_addr8_data8(uint8_t addr, uint8_t *data);
bool i2c_read_addr16_data8(uint16_t addr, uint8_t *data);
bool i2c_read_addr8_data16(uint8_t addr, uint16_t *data);
bool i2c_read_addr16_data16(uint16_t addr, uint16_t *data);
bool i2c_read_addr8_data32(uint16_t addr, uint32_t *data);
bool i2c_read_addr16_data32(uint16_t addr, uint32_t *data);
bool i2c_read_addr8_bytes(uint8_t start_addr, uint8_t *bytes, uint16_t byte_count);

/* =========================================================================
 * WRITE WRAPPERS 
 * Note: Multi-byte writes send MSB first over the I2C bus.
 * Note: All functions are polling-based.
 * ========================================================================= */
bool i2c_write_addr8_data8(uint8_t addr, uint8_t data);
bool i2c_write_addr16_data8(uint16_t addr, uint8_t data);
bool i2c_write_addr8_data16(uint8_t addr, uint16_t data);
bool i2c_write_addr16_data16(uint16_t addr, uint16_t data);
bool i2c_write_addr8_bytes(uint8_t start_addr, uint8_t *bytes, uint16_t byte_count);

#endif /* I2C_H */