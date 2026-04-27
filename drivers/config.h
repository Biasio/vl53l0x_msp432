#ifndef CONFIG_H
#define CONFIG_H

// VL53L0X  driver's macro definitions
#define XSHUT_PORT 1
#define XSHUT_PIN 1
#define VL53L0X_INT_PORT 4
#define VL53L0X_INT_PIN 6


#define VL53L0X_EUSCI_SEL EUSCI_B1
#define VL53L0X_SCL_PORT 6
#define VL53L0X_SCL_PIN 5
#define VL53L0X_SDA_PORT 6
#define VL53L0X_SDA_PIN 4

#define VL53L0X_ADDRESS (0x29)

#define VL53L0X_LOW_THRESH (600U)
#define VL53L0X_HIGH_THRESH (0xFFFFU)

#define VL53L0X_INT_POLARITY 0

#endif
