#include "i2c.h"


static inline bool stop_transfer(void)
{
    VL53L0X_EUSCI_SEL->CTLW0 |= UCTXSTP; //Setting UCTXSTP generates a STOP condition after the next acknowledge from the slave 
    WAIT_UNTIL(!(VL53L0X_EUSCI_SEL->CTLW0 & UCTXSTP), TIMEOUT); // wait for the end of the communication
    __delay_us(100); // small delay to ensure the bus is free before returning
    return false;
}



static bool start_transfer(uint16_t addr, uint8_t addr_len)
{
    WAIT_UNTIL(!(VL53L0X_EUSCI_SEL->STATW & UCBBUSY), TIMEOUT); // wait until the bus is free

    // Clear stale flags
    VL53L0X_EUSCI_SEL->IFG &= ~(EUSCI_B_IFG_NACKIFG | EUSCI_B_IFG_TXIFG0 |
                                 EUSCI_B_IFG_RXIFG0 | EUSCI_B_IFG_STTIFG |
                                 EUSCI_B_IFG_STPIFG);

    VL53L0X_EUSCI_SEL->CTLW0 |= UCTR;    // transmitter mode
    VL53L0X_EUSCI_SEL->CTLW0 |= UCTXSTT; // start
    
    // Wait for the start condition to be sent
    if (!WAIT_UNTIL(!(VL53L0X_EUSCI_SEL->CTLW0 & UCTXSTT), TIMEOUT))
        return stop_transfer();

    // Wait for address to be copied to the shift register
    if (!WAIT_UNTIL((VL53L0X_EUSCI_SEL->IFG & EUSCI_B_IFG_TXIFG0), TIMEOUT))
        return stop_transfer();

    // if SLAVE address was NACKed, abort
    if (VL53L0X_EUSCI_SEL->IFG & EUSCI_B_IFG_NACKIFG) return stop_transfer();
    
    uint8_t shift_count;
    while(addr_len-- > 0){
        //start from the MSB
        shift_count = (addr_len << 3); // *8
        VL53L0X_EUSCI_SEL->TXBUF = (addr >> shift_count) & 0xFF;

        //wait for the byte to be copied to the shift register
        if (!WAIT_UNTIL((VL53L0X_EUSCI_SEL->IFG & EUSCI_B_IFG_TXIFG0), TIMEOUT)) return stop_transfer();

        //if the transmission was NaCKed, abort
        if (VL53L0X_EUSCI_SEL->IFG & EUSCI_B_IFG_NACKIFG) return stop_transfer();
    }
    return true;
}



void i2c_init()
{
    // Primary function selection -> I2C
    PORT(VL53L0X_SDA_PORT)->SEL0 |= ONE_HOT_BIT(VL53L0X_SDA_PIN); //1
    PORT(VL53L0X_SDA_PORT)->SEL1 &= ~ONE_HOT_BIT(VL53L0X_SDA_PIN); //0
    PORT(VL53L0X_SDA_PORT)->REN  |=  ONE_HOT_BIT(VL53L0X_SDA_PIN); // enable resistor
    PORT(VL53L0X_SDA_PORT)->OUT  |=  ONE_HOT_BIT(VL53L0X_SDA_PIN); // pull-up (1 = pull-up, 0 = pull-down)
    PORT(VL53L0X_SDA_PORT)->IE   &= ~ONE_HOT_BIT(VL53L0X_SDA_PIN); // disable interrupt
    PORT(VL53L0X_SDA_PORT)->IFG  &= ~ONE_HOT_BIT(VL53L0X_SDA_PIN); // clear any pending flag
    
    // Primary function selection -> I2C
    PORT(VL53L0X_SCL_PORT)->SEL0 |= ONE_HOT_BIT(VL53L0X_SCL_PIN); //1
    PORT(VL53L0X_SCL_PORT)->SEL1 &= ~ONE_HOT_BIT(VL53L0X_SCL_PIN); //0
    PORT(VL53L0X_SCL_PORT)->REN  |=  ONE_HOT_BIT(VL53L0X_SCL_PIN); // enable resistor
    PORT(VL53L0X_SCL_PORT)->OUT  |=  ONE_HOT_BIT(VL53L0X_SCL_PIN); // pull-up (1 = pull-up, 0 = pull-down)
    PORT(VL53L0X_SCL_PORT)->IE   &= ~ONE_HOT_BIT(VL53L0X_SCL_PIN); // disable interrupt
    PORT(VL53L0X_SCL_PORT)->IFG  &= ~ONE_HOT_BIT(VL53L0X_SCL_PIN); // clear any pending flag

    //eUSCI logic held in reset state (enable modifications)
    VL53L0X_EUSCI_SEL->CTLW0 |= UCSWRST; 
    
    // UCMST=1 sets Master mode, UCSYNC=1 sets Synchronous mode, 
    // UCMODE_3=1 sets I2C mode, UCSSEL_2; // sets clock source, 
    // UCSSEL_2 selects SMCLK
    VL53L0X_EUSCI_SEL->CTLW0 |= UCMST | UCSYNC | UCMODE_3 | UCSSEL_2;  
    
    // MSP432 uses a single 16-bit BRW register
    VL53L0X_EUSCI_SEL->BRW = SCK_DIVIDER;

    VL53L0X_EUSCI_SEL->CTLW0 &= ~UCSWRST;  //eUSCI reset released for operation

    i2c_set_slave_address(DEFAULT_SLAVE_ADDRESS); // set default slave address
}



bool i2c_read(const uint16_t addr, uint8_t addr_len, uint8_t *data, uint8_t data_len)
{
    if (!start_transfer(addr, addr_len)) return false;

    // Clear pending flags
    VL53L0X_EUSCI_SEL->IFG &= ~(EUSCI_B_IFG_NACKIFG | EUSCI_B_IFG_RXIFG0 |
                                 EUSCI_B_IFG_STTIFG | EUSCI_B_IFG_STPIFG);

    VL53L0X_EUSCI_SEL->CTLW0 &= ~UCTR;   /* Configure as receiver */
    VL53L0X_EUSCI_SEL->CTLW0 |= UCTXSTT; /* Send second START condition */

    // Wait for the start condition to be sent
    if (!WAIT_UNTIL(!(VL53L0X_EUSCI_SEL->CTLW0 & UCTXSTT), TIMEOUT))
        return stop_transfer();

    // if SLAVE address was NACKed, abort
    if (VL53L0X_EUSCI_SEL->IFG & EUSCI_B_IFG_NACKIFG) return stop_transfer();

    for(uint16_t i = 0; i < data_len; ++i)
    {
        /* Send stop before reading the last byte */
        if (i == data_len-1)VL53L0X_EUSCI_SEL->CTLW0 |= UCTXSTP; 

        if (!WAIT_UNTIL((VL53L0X_EUSCI_SEL->IFG & EUSCI_B_IFG_RXIFG0), TIMEOUT)) return stop_transfer();
        //if NACK was set
        if (VL53L0X_EUSCI_SEL->IFG & EUSCI_B_IFG_NACKIFG) return stop_transfer();

        data[i] = VL53L0X_EUSCI_SEL->RXBUF; 
    }

    WAIT_UNTIL(!(VL53L0X_EUSCI_SEL->CTLW0 & UCTXSTP), TIMEOUT);
    return true;
}



bool i2c_write(const uint16_t addr, uint8_t addr_len, const uint8_t *data, uint8_t data_len)
{
    if (!start_transfer(addr, addr_len)) return false; // start the transfer for adressing a slave's register

    for (uint16_t i = 0; i < data_len; ++i)
    {
        VL53L0X_EUSCI_SEL->TXBUF = data[i]; /* write a byte in the buffer */
        //wait for the byte to be copied in the shift register
        if (!WAIT_UNTIL((VL53L0X_EUSCI_SEL->IFG & EUSCI_B_IFG_TXIFG0), TIMEOUT)) return stop_transfer();
        //if the transmission was NaCKed, abort
        if (VL53L0X_EUSCI_SEL->IFG & EUSCI_B_IFG_NACKIFG) return stop_transfer();
    }
    stop_transfer();
    return true;
}



void i2c_set_slave_address(uint8_t addr)
{
/*The I2CSAx bits contain the slave address of the external
device to be addressed by the eUSCIx_B module. It is only used in master
mode. The address is right justified. In 7-bit slave addressing mode, bit 6 is the
MSB and bits 9-7 are ignored. In 10-bit slave addressing mode, bit 9 is the MSB.*/
    VL53L0X_EUSCI_SEL->I2CSA = addr; 
}



void i2c_recover(void) {
    stop_transfer();

    VL53L0X_EUSCI_SEL->CTLW0 |= UCSWRST;   // put eUSCI in reset

    VL53L0X_EUSCI_SEL->CTLW0 |= UCMST | UCSYNC | UCMODE_3 | UCSSEL_2;  
    VL53L0X_EUSCI_SEL->BRW = SCK_DIVIDER;

    VL53L0X_EUSCI_SEL->CTLW0 &= ~UCSWRST;  // release reset
}




