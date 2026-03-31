#include "gpio.h"
#include <msp432.h>
#include <stdint.h>

void gpio_init(void)
{
    /* P1.0 GPIO and output low */
    PORT_REG(XSHUT_PORT, SEL) &= ~PIN_TO_BIT(XSHUT_PIN);
    PORT_REG(XSHUT_PORT, DIR) |= PIN_TO_BIT(XSHUT_PIN);
    PORT_REG(XSHUT_PORT, OUT) &= ~PIN_TO_BIT(XSHUT_PIN);
}
