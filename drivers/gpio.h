#ifndef GPIO_H
#define GPIO_H

#include <stdbool.h>

// Config for the sensor pin mapping
#define XSHUT_PORT 1
#define XSHUT_PIN 1


// Pre-glue macros for correct argument expansion
#define PORT_REG(port, reg) GLUE3(P, port, reg) // ex: PORT_REG(2, SEL) -> P2SEL
#define PIN_TO_BIT(pin)     GLUE2(BIT, pin)     // ex: PIN_TO_BIT(0)    -> BIT0


// Glue macros (no expansion of the macros passed as arguments)
#define GLUE2(a, b)       a ## b
#define GLUE3(a, b, c)    a ## b ## c



void gpio_init(void);

#endif
