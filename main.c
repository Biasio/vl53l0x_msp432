#include <ti/devices/msp432p4xx/inc/msp.h>
#include <stdint.h>
#include <stdbool.h>

// Your custom drivers
#include "vl53l0x.h"
#include "i2c.h"
#include "macro.h"
#include "config.h"

// Onboard Red LED maps to P1.0
#define LED_PORT_NUM 1
#define LED_PIN_NUM  0

volatile bool tof_interrupt_triggered = false;

void _ClockSystemInit(void) {
    // Stop the Watchdog Timer
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;
    // Unlock the Power Control Manager (PCM)
    PCM->CTL0 = PCM_CTL0_KEY_VAL | PCM_CTL0_AMR_1; // Switch to Active Mode LDO VCORE1
    // Wait for the voltage transition to complete
    while (PCM->CTL1 & PCM_CTL1_PMR_BUSY);

    // Set Flash Wait States to 1 (Required for operations > 24MHz)
    FLCTL->BANK0_RDCTL = (FLCTL->BANK0_RDCTL & ~FLCTL_BANK0_RDCTL_WAIT_MASK) | FLCTL_BANK0_RDCTL_WAIT_1;
    FLCTL->BANK1_RDCTL = (FLCTL->BANK1_RDCTL & ~FLCTL_BANK1_RDCTL_WAIT_MASK) | FLCTL_BANK1_RDCTL_WAIT_1;

    // LFXT (Low-Frequency Crystal) maps to Port J, Pins 0 and 1
    // HFXT (High-Frequency Crystal) maps to Port J, Pins 2 and 3
    // Set PJ.0, PJ.1, PJ.2, PJ.3 to primary module function mode
    PJ->SEL0 |= (BIT0 | BIT1 | BIT2 | BIT3);
    PJ->SEL1 &= ~(BIT0 | BIT1 | BIT2 | BIT3);

    // Unlock the Clock System registers
    CS->KEY = CS_KEY_VAL; // 0x695A

    // - Enable LFXT (32.768 kHz)
    // - Enable HFXT (48 MHz)
    // - Set HFXTFREQ to 6 (for crystals >32 MHz up to 48 MHz)
    CS->CTL2 |= (CS_CTL2_LFXT_EN | CS_CTL2_HFXT_EN | CS_CTL2_HFXTFREQ_6);

    // Wait for the crystals to stabilize. When they are starting up, they
    // trigger fault flags. We must clear them.
    while (CS->IFG & (CS_IFG_HFXTIFG | CS_IFG_LFXTIFG)) {
        CS->CLRIFG |= CS_CLRIFG_CLR_HFXTIFG | CS_CLRIFG_CLR_LFXTIFG;
    }

    // Clear the currently configured sources and dividers for MCLK and SMCLK
    CS->CTL1 &= ~(CS_CTL1_SELM_MASK | CS_CTL1_DIVM_MASK |
                  CS_CTL1_SELS_MASK | CS_CTL1_DIVS_MASK);

    // Route MCLK: Source = HFXT (48 MHz), Divider = 1 --> MCLK = 48 MHz
    // Route SMCLK: Source = HFXT (48 MHz), Divider = 4 --> SMCLK = 12 MHz
    CS->CTL1 |= (CS_CTL1_SELM__HFXTCLK | CS_CTL1_DIVM__1 |
                 CS_CTL1_SELS__HFXTCLK | CS_CTL1_DIVS__4);

    // Lock the Clock System registers
    CS->KEY = 0;
}






int main(void) {

    _ClockSystemInit();

    PORT(LED_PORT_NUM)->DIR |= ONE_HOT_BIT(LED_PIN_NUM);
    PORT(LED_PORT_NUM)->OUT &= ~ONE_HOT_BIT(LED_PIN_NUM);

    // Initialize gpio and i2c
    xshut_gpio_init();
    i2c_init();
    interrupt_gpio_init();

    // Initialize the VL53L0X sensor
    bool tof_ready = vl53l0x_init();

    // Start continuous ranging. The sensor will assert its INT pin
    // whenever a measurement is ready (or crosses the threshold).
    tof_ready &= vl53l0x_start_continuous();

    if (tof_ready) {
        // Clear any pending sensor-side interrupts before enabling local interrupts
        clear_interrupt();

        // Clear local MSP432 GPIO interrupt flag
        PORT(VL53L0X_INT_PORT)->IFG &= ~ONE_HOT_BIT(VL53L0X_INT_PIN);

        // Enable local MSP432 GPIO interrupt
        PORT(VL53L0X_INT_PORT)->IE |= ONE_HOT_BIT(VL53L0X_INT_PIN);

        // Enable interrupt in the NVIC using your macro from macro.h
        NVIC_ENABLE_PORT_INT(VL53L0X_INT_PORT);

        // Enable global processor interrupts
        __enable_irq();
    } else {
        // Sensor initialization failed. Trap execution here.
        while (1) {
            // Fast toggle to visually indicate an error state
            __delay_us(100000);
            PORT(LED_PORT_NUM)->OUT ^= ONE_HOT_BIT(LED_PIN_NUM);
        }
    }

    // 5. Main processing loop
    while (1) {
        if (tof_interrupt_triggered) {
            tof_interrupt_triggered = false;

            uint16_t range = 0;
            uint8_t error_code = 0;

            // Read the range data and clear the sensor's internal interrupt latch.
            if (vl53l0x_read_range_interrupt(&range, &error_code)) {
                if (range != VL53L0X_OUT_OF_RANGE && range < VL53L0X_LOW_THRESH){
                    PORT(LED_PORT_NUM)->OUT |= ONE_HOT_BIT(LED_PIN_NUM);
                    __delay_us(100000);
                    PORT(LED_PORT_NUM)->OUT &= ~ONE_HOT_BIT(LED_PIN_NUM);
                }
            }

            PORT(VL53L0X_INT_PORT)->IE  |= ONE_HOT_BIT(VL53L0X_INT_PIN);
        }

        __disable_irq(); // Prevent race conditions before sleeping
        if (!tof_interrupt_triggered) {
            __enable_irq();
            __WFI();
        } else {
            __enable_irq();
        }
    }
}

void PORT4_IRQHandler(void) {
    // Check if the interrupt specifically comes from the VL53L0X pin
    if (PORT(VL53L0X_INT_PORT)->IFG & ONE_HOT_BIT(VL53L0X_INT_PIN))
    {
        PORT(VL53L0X_INT_PORT)->IE  &= ~ONE_HOT_BIT(VL53L0X_INT_PIN);
        PORT(VL53L0X_INT_PORT)->IFG &= ~ONE_HOT_BIT(VL53L0X_INT_PIN);
        tof_interrupt_triggered = true;
    }
}
