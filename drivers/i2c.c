#include "i2c.h"

static bool start_transfer(uint16_t addr, uint8_t addr_len)
{
    VL53L0X_EUSCI_SEL->CTLW0 |= UCTXSTT + UCTR; /* Set up master as TX and send start condition */

    /* Send MSB first if 16-bit address */
    if (addr_len == 2) {
        VL53L0X_EUSCI_SEL->TXBUF = (addr >> 8) & 0xFF; //MSB
        while (VL53L0X_EUSCI_SEL->CTLW0 & UCTXSTT); /* Wait for start condition to be sent */
        /*If the slave does not acknowledge the transmitted data, the not-acknowledge interrupt flag UCNACKIFG is set*/
        if (VL53L0X_EUSCI_SEL->IFG & EUSCI_B_IFG_NACKIFG) return false;
        while (!(VL53L0X_EUSCI_SEL->IFG & EUSCI_B_IFG_TXIFG0)); /* Wait for byte to be sent */
        /*If the slave does not acknowledge the transmitted data, the not-acknowledge interrupt flag UCNACKIFG is set*/
        if (VL53L0X_EUSCI_SEL->IFG & EUSCI_B_IFG_NACKIFG) return false;
    }

    /* Send LSB (or 8-bit address) */
    VL53L0X_EUSCI_SEL->TXBUF = addr & 0xFF;
    if (addr_len == 1) {
        while (VL53L0X_EUSCI_SEL->CTLW0 & UCTXSTT);
    }
    /*If the slave does not acknowledge the transmitted data, the not-acknowledge interrupt flag UCNACKIFG is set*/
    if (VL53L0X_EUSCI_SEL->IFG & EUSCI_B_IFG_NACKIFG) return false;

    while (!(VL53L0X_EUSCI_SEL->IFG & EUSCI_B_IFG_TXIFG0));
    /*If the slave does not acknowledge the transmitted data, the not-acknowledge interrupt flag UCNACKIFG is set*/
    return !(VL53L0X_EUSCI_SEL->IFG & EUSCI_B_IFG_NACKIFG);
}

static void stop_transfer()
{
    VL53L0X_EUSCI_SEL->CTLW0 |= UCTXSTP; //Setting UCTXSTP generates a STOP condition after the next acknowledge from the slave 
    while (VL53L0X_EUSCI_SEL->CTLW0 & UCTXSTP); // wait for the end of the communication
}

bool i2c_read(const uint16_t addr, uint8_t addr_len, uint8_t *data, uint8_t data_len)
{
    if (!start_transfer(addr, addr_len)) return false; // start the transfer to request data

    VL53L0X_EUSCI_SEL->CTLW0 &= ~UCTR;   /* Configure as receiver */
    VL53L0X_EUSCI_SEL->CTLW0 |= UCTXSTT; /* Send RESTART condition */
    while (VL53L0X_EUSCI_SEL->CTLW0 & UCTXSTT); /* Wait for start condition to be received */
    if (VL53L0X_EUSCI_SEL->IFG & EUSCI_B_IFG_NACKIFG) return false; /*If the slave does not acknowledge the transmitted data, the not-acknowledge interrupt flag UCNACKIFG is set*/
    for(uint16_t i = 0; i < data_len; ++i){
        if (i == data_len-1) {
            VL53L0X_EUSCI_SEL->CTLW0 |= UCTXSTP; /* Send stop before reading the last byte */
        }
        while (!(VL53L0X_EUSCI_SEL->IFG & EUSCI_B_IFG_RXIFG0)); /*The UCRXIFG0 interrupt flag is set when a character is received and loaded into UCBxRXBUF*/
        data[i] = VL53L0X_EUSCI_SEL->RXBUF; 
    }
    return true;
}


bool i2c_write(const uint16_t addr, uint8_t addr_len, const uint8_t *data, uint8_t data_len)
{
    if (!start_transfer(addr, addr_len)) return false; // start the transfer for adressing a slave's register
    
    for (uint16_t i = 0; i < data_len; ++i){
        VL53L0X_EUSCI_SEL->TXBUF = data[i]; /* write a byte in the buffer */
        /* CTXIFG2 is set when UCBxTXBUF is empty
        in slave mode, if the slave address defined in UCBxI2COA2 was on the bus in
        the same frame. */
        while (!(VL53L0X_EUSCI_SEL->IFG & EUSCI_B_IFG_TXIFG0)); 
        /*If the slave does not acknowledge the transmitted data, the not-acknowledge interrupt flag UCNACKIFG is set*/
        if (VL53L0X_EUSCI_SEL->IFG & EUSCI_B_IFG_NACKIFG) {
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
    PORT(VL53L0X_SDA_PORT)->SEL0 |= ONE_HOT_BIT(VL53L0X_SDA_PIN);
    PORT(VL53L0X_SCL_PORT)->SEL0 |= ONE_HOT_BIT(VL53L0X_SCL_PIN);
    PORT(VL53L0X_SDA_PORT)->SEL1 &= ~ONE_HOT_BIT(VL53L0X_SDA_PIN);
    PORT(VL53L0X_SCL_PORT)->SEL1 &= ~ONE_HOT_BIT(VL53L0X_SCL_PIN);

    VL53L0X_EUSCI_SEL->CTLW0 |= UCSWRST; //eUSCI logic held in reset state (enable modifications)
    
    // Changed '=' to '|=' here because MSP432 uses a 16-bit CTLW0 register. 
    // '=' would overwrite and clear the UCSWRST bit we just set above.
    VL53L0X_EUSCI_SEL->CTLW0 |= UCMST + UCSYNC + UCMODE_3;  //UCMST=1 sets Master mode, UCSYNC=1 sets Synchronous mode, UCMODE_3=1 sets I2C mode
    VL53L0X_EUSCI_SEL->CTLW0 |= UCSSEL_2; // sets clock source, UCSSEL_2 selects SMCLK
    
    // MSP432 uses a single 16-bit BRW register
    VL53L0X_EUSCI_SEL->BRW = 30; //Bit Rate Control 12,000,000 / 30 = 400kHz (fast mode, within VL53L0X spec)
    
    VL53L0X_EUSCI_SEL->CTLW0 &= ~UCSWRST;  //eUSCI reset released for operation
    i2c_set_slave_address(DEFAULT_SLAVE_ADDRESS); // set default slave address
}


void i2c_set_slave_address(uint8_t addr)
{
/*The I2CSAx bits contain the slave address of the external
device to be addressed by the eUSCIx_B module. It is only used in master
mode. The address is right justified. In 7-bit slave addressing mode, bit 6 is the
MSB and bits 9-7 are ignored. In 10-bit slave addressing mode, bit 9 is the MSB.*/
    VL53L0X_EUSCI_SEL->I2CSA = addr; 
}

