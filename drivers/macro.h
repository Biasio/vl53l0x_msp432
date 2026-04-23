#ifndef __MACRO_H__
#define __MACRO_H__

// Pre-glue macros for correct argument expansion
#define PORT(port) MACRO_EXPANSION_2(P, port) // ex: PORT(2, SEL) -> P2
#define ONE_HOT_BIT(pin) MACRO_EXPANSION_2(BIT, pin)     // ex: ONE_HOT_BIT(0)    -> BIT0

#define MACRO_EXPANSION_2(a,b) GLUE2(a,b)
// Glue macros (no expansion of the macros passed as arguments)
#define GLUE2(a, b)       a ## b



#endif
