#include "i2c.h"
#include <msp432.h>
#include <stddef.h>


static bool start_transfer(uint16_t addr, uint8_t addr_len)
{
    USCI_REG(EUSCI_SEL,CTL1) |= UCTXSTT + UCTR; /* Set up master as TX and send start condition */

    /* Send MSB first if 16-bit address */
    if (addr_len == 2) {
        USCI_REG(EUSCI_SEL,TXBUF) = (addr >> 8) & 0xFF; //MSB
        while (USCI_REG(EUSCI_SEL,CTL1) & UCTXSTT); /* Wait for start condition to be sent */
        if (USCI_REG(EUSCI_SEL,STAT) & UCNACKIFG) return false; 
        while (!(IFG2 & USCI_REG(EUSCI_SEL,TXIFG))); /* Wait for byte to be sent */
        if (USCI_REG(EUSCI_SEL,STAT) & UCNACKIFG) return false;
    }

    /* Send LSB (or 8-bit address) */
    USCI_REG(EUSCI_SEL,TXBUF) = addr & 0xFF;
    if (addr_len == 1) {
        while (USCI_REG(EUSCI_SEL,CTL1) & UCTXSTT);
    }
    if (USCI_REG(EUSCI_SEL,STAT) & UCNACKIFG) return false;
    
    while (!(IFG2 & USCI_REG(EUSCI_SEL,TXIFG)));
    return !(USCI_REG(EUSCI_SEL,STAT) & UCNACKIFG);
}

static void stop_transfer()
{
    USCI_REG(EUSCI_SEL,CTL1) |= UCTXSTP;
    while (USCI_REG(EUSCI_SEL,CTL1) & UCTXSTP);
}

static bool i2c_read_core(uint16_t addr, uint8_t addr_len, uint8_t *data, uint16_t len)
{
    if (!start_transfer(addr, addr_len)) return false;

    USCI_REG(EUSCI_SEL,CTL1) &= ~UCTR;   /* Configure as receiver */
    USCI_REG(EUSCI_SEL,CTL1) |= UCTXSTT; /* Send RESTART condition */
    while (USCI_REG(EUSCI_SEL,CTL1) & UCTXSTT); 
    if (USCI_REG(EUSCI_SEL,STAT) & UCNACKIFG) return false;

    for (uint16_t i = 0; i < len; i++) {
        if (i == len - 1) {
            USCI_REG(EUSCI_SEL,CTL1) |= UCTXSTP; /* Send stop before reading the last byte */
        }
        while (!(IFG2 & USCI_REG(EUSCI_SEL,RXIFG)));
        data[i] = USCI_REG(EUSCI_SEL,RXBUF); 
    }
    return true;
}

static bool i2c_write_core(uint16_t addr, uint8_t addr_len, const uint8_t *data, uint16_t len)
{
    if (!start_transfer(addr, addr_len)) return false;

    for (uint16_t i = 0; i < len; i++) {
        USCI_REG(EUSCI_SEL,TXBUF) = data[i];
        while (!(IFG2 & USCI_REG(EUSCI_SEL,TXIFG)));
        if (USCI_REG(EUSCI_SEL,STAT) & UCNACKIFG) {
            stop_transfer();
            return false;
        }
    }
    stop_transfer();
    return true;
}


void i2c_init()
{
    // Primary function selection -> I2C
    P6SEL0 |= BIT5 + BIT4;
    P6SEL1 &= ~(BIT5 + BIT4);

    USCI_REG(EUSCI_SEL,CTL1) |= UCSWRST; //eUSCI logic held in reset state (enable modifications)
    USCI_REG(EUSCI_SEL,CTL0) = UCMST + UCSYNC + UCMODE_3;  //UCMST=1 sets Master mode, UCSYNC=1 sets Synchronous mode, UCMODE_3=1 sets I2C mode
    USCI_REG(EUSCI_SEL,CTL1) |= UCSSEL_2; // sets clock source, UCSSEL_2 selects SMCLK
    USCI_REG(EUSCI_SEL,BR0) = 10; //Bit Rate Control 1010
    USCI_REG(EUSCI_SEL,BR1) = 0; //Bit Rate Control
    USCI_REG(EUSCI_SEL,CTL1) &= ~UCSWRST;  //eUSCI reset released for operation
    i2c_set_slave_address(DEFAULT_SLAVE_ADDRESS); // set default slave address
}

void i2c_set_slave_address(uint8_t addr)
{
    /*The I2CSAx bits contain the slave address of the external
device to be addressed by the eUSCIx_B module. It is only used in master
mode. The address is right justified. In 7-bit slave addressing mode, bit 6 is the
MSB and bits 9-7 are ignored. In 10-bit slave addressing mode, bit 9 is the MSB.*/
    USCI_REG(EUSCI_SEL,I2CSA) = addr; 
}

/* --- READ FUNCTIONS --- */

bool i2c_read_addr8_data8(uint8_t addr, uint8_t *data) {
    return i2c_read_core(addr, 1, data, 1);
}

bool i2c_read_addr16_data8(uint16_t addr, uint8_t *data) {
    return i2c_read_core(addr, 2, data, 1);
}

bool i2c_read_addr8_bytes(uint8_t start_addr, uint8_t *bytes, uint16_t byte_count) {
    return i2c_read_core(start_addr, 1, bytes, byte_count);
}

bool i2c_read_addr8_data16(uint8_t addr, uint16_t *data) {
    uint8_t buf[2];
    if (!i2c_read_core(addr, 1, buf, 2)) return false;
    *data = (buf[0] << 8) | buf[1]; /* Assemble MSB first safely */
    return true;
}

bool i2c_read_addr16_data16(uint16_t addr, uint16_t *data) {
    uint8_t buf[2];
    if (!i2c_read_core(addr, 2, buf, 2)) return false;
    *data = (buf[0] << 8) | buf[1];
    return true;
}

bool i2c_read_addr8_data32(uint16_t addr, uint32_t *data) {
    uint8_t buf[4];
    if (!i2c_read_core(addr, 1, buf, 4)) return false;
    *data = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];
    return true;
}

bool i2c_read_addr16_data32(uint16_t addr, uint32_t *data) {
    uint8_t buf[4];
    if (!i2c_read_core(addr, 2, buf, 4)) return false;
    *data = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];
    return true;
}

/* --- WRITE FUNCTIONS --- */

bool i2c_write_addr8_data8(uint8_t addr, uint8_t value) {
    return i2c_write_core(addr, 1, &value, 1);
}

bool i2c_write_addr16_data8(uint16_t addr, uint8_t value) {
    return i2c_write_core(addr, 2, &value, 1);
}

bool i2c_write_addr8_bytes(uint8_t start_addr, uint8_t *bytes, uint16_t byte_count) {
    return i2c_write_core(start_addr, 1, bytes, byte_count);
}

bool i2c_write_addr8_data16(uint8_t addr, uint16_t value) {
    uint8_t buf[2] = { (value >> 8) & 0xFF, value & 0xFF }; /* Disassemble to MSB first */
    return i2c_write_core(addr, 1, buf, 2);
}

bool i2c_write_addr16_data16(uint16_t addr, uint16_t value) {
    uint8_t buf[2] = { (value >> 8) & 0xFF, value & 0xFF };
    return i2c_write_core(addr, 2, buf, 2);
}