#ifndef __MACRO_H__
#define __MACRO_H__

// Pre-glue macros for correct argument expansion
#define PORT(port) MACRO_EXPANSION_2(P, port) // ex: PORT(2, SEL) -> P2
#define ONE_HOT_BIT(pin) MACRO_EXPANSION_2(BIT, pin)     // ex: ONE_HOT_BIT(0)    -> BIT0

#define MACRO_EXPANSION_2(a,b) GLUE2(a,b)
// Glue macros (no expansion of the macros passed as arguments)
#define GLUE2(a, b)       a ## b



#ifndef MCLK_HZ //used for the delay function
#warning "MCLK_HZ not specified, defaulting to 48Mhz. Generally this is ok since it's the fastest available frequency"
#define MCLK_HZ (48000000UL)    /* 48 MHz default */
#endif

#include <stdint.h>

static inline void __delay_us(uint32_t us)
{
    /* Each loop iteration costs ~3 cycles so 3*1000000ULL */
    uint64_t iterations= ( MCLK_HZ * (uint64_t) us) / 3000000ULL ;

    // maybe too small ms cause a resulting iterations = 0
    if(!iterations) iterations |= 1; //set iteration to 1
    while (--iterations) __asm__ volatile ("nop");   // 1 cycle NOP
}

#endif
