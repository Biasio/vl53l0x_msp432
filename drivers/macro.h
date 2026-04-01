#ifndef __MACRO_H__
#define __MACRO_H__

// Pre-glue macros for correct argument expansion
#define PORT_REG(port, reg) (GLUE2(P, port) -> (reg)) // ex: PORT_REG(2, SEL) -> P2SEL
#define PIN_TO_BIT(pin)     GLUE2(BIT, pin)     // ex: PIN_TO_BIT(0)    -> BIT0


// Glue macros (no expansion of the macros passed as arguments)
#define GLUE2(a, b)       a ## b

#endif