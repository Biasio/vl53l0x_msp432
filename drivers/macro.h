#ifndef __MACRO_H__
#define __MACRO_H__

#include <stdint.h>
#include <limits.h>
#include "config.h"

// Pre-glue macros for correct argument expansion
#define PORT(port) MACRO_EXPANSION_2(P, port) // ex: PORT(2, SEL) -> P2
#define ONE_HOT_BIT(pin) MACRO_EXPANSION_2(BIT, pin)     // ex: ONE_HOT_BIT(0)    -> BIT0

#define MACRO_EXPANSION_2(a,b) GLUE2(a,b)
// Glue macros (no expansion of the macros passed as arguments)
#define GLUE2(a, b)       a ## b

#define NVIC_ENABLE_PORT_INT(port) \
  (NVIC->ISER[((port) + 34) / 32] |= (1UL << (((port) + 34) & 0x1F)))

#ifndef MCLK_HZ //used for the delay function
    #define MCLK_HZ (48000000UL)    /* 48 MHz default */
    #warning "MCLK_HZ not specified, defaulting to 48Mhz. \
        Generally this is ok since it's the fastest available frequency"
#endif


#define MAX_US (((UINT32_MAX) * 7000000ULL) / (MCLK_HZ))

static void __delay_us(uint64_t us)
{
    // Delay full chunks of max_us
    while (us >= MAX_US) {
        volatile uint32_t iterations = UINT32_MAX;
        while (iterations--);
        us -= MAX_US;
    }

    // Delay the remaining microseconds (us < MAX_US)
    if (us > 0) {
        volatile uint32_t iterations = ( (MCLK_HZ) * (uint64_t) us )/7000000ULL;
        if (iterations == 0) iterations = 1; // triggered if MCLK_HZ < 2MHz
        while (iterations--);
    }
}

//MACRO to wait for a condition and set a temporal limit if this 
//condition wouldn't occour. The condition, when true, let escape the while
//returns true if the condition was met first, false if timeout occurred first
#define WAIT_UNTIL(condition, timeout_us)   \
({                                          \
        uint32_t _cnt = (timeout_us);       \
        while (!(condition) && --(_cnt)) {  \
            __delay_us(1);                  \
        }                                   \
        _cnt;                               \
})


#define I2C_POLL_UNTIL(reg, data_buf, condition, timeout_us) \
({                                                           \
    bool _success = false;                                   \
    uint32_t _cnt = (timeout_us);                            \
    do {                                                     \
        _success = i2c_read(reg, 1, data_buf, 1);            \
        __delay_us(1);                                       \
    } while (!(condition) && _success && --(_cnt));          \
    if (_cnt == 0) _success = false;                         \
    _success;                                                \
})

// computed for 400kHz I2C clock, which is the maximum supported by the VL53L0X
#ifdef SMCLK_HZ
    #define SCK_DIVIDER ((SMCLK_HZ)/(SCK_FREQ_HZ))
#endif

//used for the sck frequency of the I2C bus
#ifndef SMCLK_HZ 
#error "SMCLK_HZ not specified."
#endif

//used for the sck frequency of the I2C bus
#ifndef SCK_FREQ_HZ
#error "SCK_FREQ_HZ not specified."
#endif

#endif
