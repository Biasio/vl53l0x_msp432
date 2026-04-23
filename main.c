#include <msp432.h>
#include "drivers/i2c.h"
#include "drivers/vl53l0x.h"
#include <stdio.h>
#include <inttypes.h>


static void msp432_init()
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;
}


int main(void)
{
	msp432_init();
	i2c_init();

    bool success = vl53l0x_init();
    uint16_t range =0;
    while (success) {
        success = vl53l0x_read_range_single(&range);
        printf("%" PRIu16 "\n",  range);
        for(volatile uint32_t j = 0; j < 100000; j++);
    }
	return 0;
}
