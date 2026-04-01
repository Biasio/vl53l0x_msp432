#include "i2c.h"



static bool start_transfer(uint16_t addr, uint8_t addr_len)
{
    USCI_REG_PTR(EUSCI_SEL,CTL1) |= UCTXSTT + UCTR; /* Set up master as TX and send start condition */

    /* Send MSB first if 16-bit address */
    if (addr_len == 2) {
        USCI_REG_VAR(EUSCI_SEL,TXBUF) = (addr >> 8) & 0xFF; //MSB
        while (USCI_REG_PTR(EUSCI_SEL,CTL1) & UCTXSTT); /* Wait for start condition to be sent */
        /*If the slave does not acknowledge the transmitted data, the not-acknowledge interrupt flag UCNACKIFG is set*/
        if (USCI_REG_VAR(EUSCI_SEL,STAT) & UCNACKIFG) return false;
        while (!(IFG2 & USCI_REG_VAR(EUSCI_SEL,TXIFG))); /* Wait for byte to be sent */
        /*If the slave does not acknowledge the transmitted data, the not-acknowledge interrupt flag UCNACKIFG is set*/
        if (USCI_REG_VAR(EUSCI_SEL,STAT) & UCNACKIFG) return false;
    }

    /* Send LSB (or 8-bit address) */
    USCI_REG_VAR(EUSCI_SEL,TXBUF) = addr & 0xFF;
    if (addr_len == 1) {
        while (USCI_REG_PTR(EUSCI_SEL,CTL1) & UCTXSTT);
    }
    /*If the slave does not acknowledge the transmitted data, the not-acknowledge interrupt flag UCNACKIFG is set*/
    if (USCI_REG_VAR(EUSCI_SEL,STAT) & UCNACKIFG) return false;

    while (!(IFG2 & USCI_REG_VAR(EUSCI_SEL,TXIFG)));
    /*If the slave does not acknowledge the transmitted data, the not-acknowledge interrupt flag UCNACKIFG is set*/
    return !(USCI_REG_VAR(EUSCI_SEL,STAT) & UCNACKIFG);
}

static void stop_transfer()
{
    USCI_REG_PTR(EUSCI_SEL,CTL1) |= UCTXSTP; //Setting UCTXSTP generates a STOP condition after the next acknowledge from the slave 
    while (USCI_REG_PTR(EUSCI_SEL,CTL1) & UCTXSTP); // wait for the end of the communication
}

bool i2c_read_core(const uint16_t addr, uint8_t addr_len, uint8_t *data, uint16_t data_len)
{
    if (!start_transfer(addr, addr_len)) return false; // start the transfer to request data

    USCI_REG_PTR(EUSCI_SEL,CTL1) &= ~UCTR;   /* Configure as receiver */
    USCI_REG_PTR(EUSCI_SEL,CTL1) |= UCTXSTT; /* Send RESTART condition */
    while (USCI_REG_PTR(EUSCI_SEL,CTL1) & UCTXSTT); /* Wait for start condition to be received */
    if (USCI_REG_VAR(EUSCI_SEL,STAT) & UCNACKIFG) return false; /*If the slave does not acknowledge the transmitted data, the not-acknowledge interrupt flag UCNACKIFG is set*/
    for(uint16_t i = 0; i < data_len; ++i){
        if (i == data_len-1) {
            USCI_REG_PTR(EUSCI_SEL,CTL1) |= UCTXSTP; /* Send stop before reading the last byte */
        }
        while (!(IFG2 & USCI_REG_VAR(EUSCI_SEL,RXIFG))); /*The UCRXIFG0 interrupt flag is set when a character is received and loaded into UCBxRXBUF*/
        data[i] = USCI_REG_VAR(EUSCI_SEL,RXBUF); 
    }
    return true;
}


bool i2c_write_core(const uint16_t addr, uint8_t addr_len, const uint8_t *data, uint16_t data_len)
{
    if (!start_transfer(addr, addr_len)) return false; // start the transfer for adressing a slave's register
    
    for (uint16_t i = 0; i < data_len; ++i){
        USCI_REG_VAR(EUSCI_SEL,TXBUF) = data[i]; /* write a byte in the buffer */
        /* CTXIFG2 is set when UCBxTXBUF is empty
        in slave mode, if the slave address defined in UCBxI2COA2 was on the bus in
        the same frame. */
        while (!(IFG2 & USCI_REG_VAR(EUSCI_SEL,RXIFG))); 
        /*If the slave does not acknowledge the transmitted data, the not-acknowledge interrupt flag UCNACKIFG is set*/
        if (USCI_REG_VAR(EUSCI_SEL,STAT) & UCNACKIFG) {
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
    P6->SEL0 |= BIT5 + BIT4;
    P6->SEL1 &= ~(BIT5 + BIT4);

    USCI_REG_PTR(EUSCI_SEL,CTL1) |= UCSWRST; //eUSCI logic held in reset state (enable modifications)
    USCI_REG_PTR(EUSCI_SEL,CTL0) = UCMST + UCSYNC + UCMODE_3;  //UCMST=1 sets Master mode, UCSYNC=1 sets Synchronous mode, UCMODE_3=1 sets I2C mode
    USCI_REG_PTR(EUSCI_SEL,CTL1) |= UCSSEL_2; // sets clock source, UCSSEL_2 selects SMCLK
    USCI_REG_VAR(EUSCI_SEL,BR0) = 10; //Bit Rate Control 1010
    USCI_REG_VAR(EUSCI_SEL,BR1) = 0; //Bit Rate Control
    USCI_REG_PTR(EUSCI_SEL,CTL1) &= ~UCSWRST;  //eUSCI reset released for operation
    i2c_set_slave_address(DEFAULT_SLAVE_ADDRESS); // set default slave address
}


void i2c_set_slave_address(uint8_t addr)
{
    /*The I2CSAx bits contain the slave address of the external
device to be addressed by the eUSCIx_B module. It is only used in master
mode. The address is right justified. In 7-bit slave addressing mode, bit 6 is the
MSB and bits 9-7 are ignored. In 10-bit slave addressing mode, bit 9 is the MSB.*/
    USCI_REG_VAR(EUSCI_SEL,I2CSA) = addr; 
}