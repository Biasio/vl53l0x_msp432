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

/**
 * Initializes the sensors in the vl53l0x_idx_t enum.
 * @note Each sensor must have its XSHUT pin connected.
 */

void xshut_init(void);
void xshut_toggle(bool state);

bool vl53l0x_init();
/**
 * Does a single range measurement
 * @param idx selects specific sensor
 * @param range contains the measured range or VL53L0X_OUT_OF_RANGE
 *        if out of range.
 * @return True if success, False if error
 * @note   Polling-based
 */
bool vl53l0x_read_range_single(uint16_t *range);

#endif /* VL53L0X_H */
