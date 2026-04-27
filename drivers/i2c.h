#ifndef I2C_H
#define I2C_H

#include <ti/devices/msp432p4xx/inc/msp.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "macro.h"
#include "config.h"



void i2c_init(void);

void i2c_set_slave_address(uint8_t addr);

bool i2c_read(const uint16_t addr, uint8_t addr_len, uint8_t *data, uint8_t data_len);

bool i2c_write(const uint16_t addr, uint8_t addr_len, const uint8_t *data, uint8_t data_len);

#define DEFAULT_SLAVE_ADDRESS (0x29)

#ifndef VL53L0X_EUSCI_SEL
#error "VL53L0X_EUSCI_SEL must be defined before including i2c.h"
#endif

#ifndef VL53L0X_SCL_PORT
#error "VL53L0X_SCL_PORT must be defined before including i2c.h"
#endif

#ifndef VL53L0X_SCL_PIN
#error "VL53L0X_SCL_PIN must be defined before including i2c.h"
#endif

#ifndef VL53L0X_SDA_PORT
#error "VL53L0X_SDA_PORT must be defined before including i2c.h"
#endif

#ifndef VL53L0X_SDA_PIN
#error "VL53L0X_SDA_PIN must be defined before including i2c.h"
#endif



#endif /* I2C_H */
