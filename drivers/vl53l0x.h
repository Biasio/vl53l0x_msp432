#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdbool.h>
#include <stdint.h>
#include <msp432.h>
#include "i2c.h"
#include "macro.h"

// Config for the sensor pin mapping
#define XSHUT_PORT 1
#define XSHUT_PIN 1

#define VL53L0X_OUT_OF_RANGE (8190)

void xshut_init(void);
void xshut_toggle(bool state);

bool vl53l0x_init();

bool vl53l0x_read_range_single(uint16_t *range);

#endif /* VL53L0X_H */
