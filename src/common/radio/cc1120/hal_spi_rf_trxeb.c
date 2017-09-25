
/******************************************************************************
 * INCLUDES
 */
#include <msp430.h>
#include "hal_spi_rf_trxeb.h"

      #define     RF_PORT_SEL1           P5SEL1
      #define     RF_PORT_SEL0           P5SEL0
      #define     RF_PORT_OUT            P5OUT
      #define     RF_PORT_DIR            P5DIR
      #define     RF_PORT_IN             P5IN

      #define     RF_MOSI_PIN            BIT0
      #define     RF_MISO_PIN            BIT1
      #define     RF_SCLK_PIN            BIT2

      /* Transceiver chip select signal */
      #define     RF_CS_N_PORT_SEL       P4SEL1
      #define     RF_CS_N_PORT_DIR       P4DIR
      #define     RF_CS_N_PORT_OUT       P4OUT
      #define     RF_CS_N_PIN            BIT4
      #define     RF_RESET_N_PORT_SEL1   P8SEL1
      #define     RF_RESET_N_PORT_SEL0   P8SEL0
/******************************************************************************
 * LOCAL FUNCTIONS
 */
static void trxReadWriteBurstSingle(uint8_t addr,uint8_t *pData,uint16_t len) ;


/******************************************************************************
 * FUNCTIONS
 */

/******************************************************************************
 * @fn          trxRfSpiInterfaceInit
 *
 * @brief       Function to initialize TRX SPI. CC1101/CC112x is currently
 *              supported. The supported prescalerValue must be set so that
 *              SMCLK/prescalerValue does not violate radio SPI constraints.
 *
 * input parameters
 *
 * @param       prescalerValue - SMCLK/prescalerValue gives SCLK frequency
 *
 * output parameters
 *
 * @return      void
 */
void trxRfSpiInterfaceInit(uint8_t prescalerValue) {

    // Disable the GPIO power-on default high-impedance mode to
    PM5CTL0 &= ~LOCKLPM5;

    // XT1 Setup
    PJSEL0 |= BIT4 | BIT5;

    // Unlock CS registers
    CSCTL0_H = CSKEY_H;

    // Set DCO to 1MHz
    CSCTL1 = DCOFSEL_0;
    CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK;

    // set all dividers
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;
    CSCTL4 &= ~LFXTOFF;
    do {
        // Clear XT1 fault flag
        CSCTL5 &= ~LFXTOFFG;
        SFRIFG1 &= ~OFIFG;
    }

    // Test oscillator fault flag
    while (SFRIFG1 & OFIFG);

    // Lock CS registers
    CSCTL0_H = 0;

    // Put state machine in reset
    UCB1CTLW0 = UCSWRST;

    // 3-pin, 8-bit SPI master Clock polarity high, MSB
    UCB1CTLW0 |= 0x00 | UCMST | UCSYNC | UCMODE_0 | UCMSB | UCCKPH;

    // ACLK
    UCB1CTLW0 |= UCSSEL__ACLK;

    // Set baud with prescaler
    UCB1BR1 = 0x00;
    UCB1BR0 = prescalerValue;

    // Chip select
    RF_CS_N_PORT_SEL &= ~RF_CS_N_PIN;
    RF_CS_N_PORT_DIR |= RF_CS_N_PIN;
    RF_CS_N_PORT_OUT |= RF_CS_N_PIN;

    // Configure GPIO MISO MOSI CLK
    RF_PORT_SEL1 &= ~(RF_MOSI_PIN | RF_MISO_PIN | RF_SCLK_PIN);
    RF_PORT_SEL0 |= (RF_MOSI_PIN | RF_MISO_PIN | RF_SCLK_PIN);

    // CC1120 GPIO reset
    RF_RESET_N_PORT_SEL1 &= ~RF_RESET_N_PIN;
    RF_RESET_N_PORT_SEL0 &= ~RF_RESET_N_PIN;
    RF_RESET_N_PORT_DIR |= RF_RESET_N_PIN;
    RF_RESET_N_PORT_OUT |= RF_RESET_N_PIN;

    UCB1CTLW0 &= ~UCSWRST;

    return;
}

rfStatus_t trxSpiCmdStrobe(uint8_t cmd) {

    uint8_t rc;
    TRXEM_SPI_BEGIN();
    while(RF_PORT_IN & RF_MISO_PIN);
    TRXEM_SPI_TX(cmd);
    TRXEM_SPI_WAIT_DONE();
    rc = TRXEM_SPI_RX();
    TRXEM_SPI_END();
    return(rc);
}


/*******************************************************************************
 * @fn          trx8BitRegAccess
 *
 * @brief       This function performs a read or write from/to a 8bit register
 *              address space. The function handles burst and single read/write
 *              as specfied in addrByte. Function assumes that chip is ready.
 *
 * input parameters
 *
 * @param       accessType - Specifies if this is a read or write and if it's
 *                           a single or burst access. Bitmask made up of
 *                           RADIO_BURST_ACCESS/RADIO_SINGLE_ACCESS/
 *                           RADIO_WRITE_ACCESS/RADIO_READ_ACCESS.
 * @param       addrByte - address byte of register.
 * @param       pData    - data array
 * @param       len      - Length of array to be read(TX)/written(RX)
 *
 * output parameters
 *
 * @return      chip status
 */
rfStatus_t trx8BitRegAccess(uint8_t accessType, uint8_t addrByte, uint8_t *pData, uint16_t len) {

  uint8_t readValue;

  /* Pull CS_N low and wait for SO to go low before communication starts */
  TRXEM_SPI_BEGIN();
  while(TRXEM_MISO_PORT_IN & TRXEM_SPI_MISO_PIN);
  /* send register address byte */
  TRXEM_SPI_TX(accessType|addrByte);
  TRXEM_SPI_WAIT_DONE();
  /* Storing chip status */
  readValue = TRXEM_SPI_RX();
  trxReadWriteBurstSingle(accessType|addrByte,pData,len);
  TRXEM_SPI_END();
  /* return the status byte value */
  return(readValue);
}

/******************************************************************************
 * @fn          trx16BitRegAccess
 *
 * @brief       This function performs a read or write in the extended adress
 *              space of CC112X.
 *
 * input parameters
 *
 * @param       accessType - Specifies if this is a read or write and if it's
 *                           a single or burst access. Bitmask made up of
 *                           RADIO_BURST_ACCESS/RADIO_SINGLE_ACCESS/
 *                           RADIO_WRITE_ACCESS/RADIO_READ_ACCESS.
 * @param       extAddr - Extended register space address = 0x2F.
 * @param       regAddr - Register address in the extended address space.
 * @param       *pData  - Pointer to data array for communication
 * @param       len     - Length of bytes to be read/written from/to radio
 *
 * output parameters
 *
 * @return      rfStatus_t
 */
rfStatus_t trx16BitRegAccess(uint8_t accessType, uint8_t extAddr, uint8_t regAddr, uint8_t *pData, uint8_t len) {

  uint8_t readValue;

  TRXEM_SPI_BEGIN();
  while(TRXEM_MISO_PORT_IN & TRXEM_SPI_MISO_PIN);
  /* send extended address byte with access type bits set */
  TRXEM_SPI_TX(accessType|extAddr);
  TRXEM_SPI_WAIT_DONE();
  /* Storing chip status */
  readValue = TRXEM_SPI_RX();
  TRXEM_SPI_TX(regAddr);
  TRXEM_SPI_WAIT_DONE();
  /* Communicate len number of bytes */
  trxReadWriteBurstSingle(accessType|extAddr,pData,len);
  TRXEM_SPI_END();
  /* return the status byte value */
  return(readValue);
}

/*******************************************************************************
 * @fn          trxSpiCmdStrobe
 *
 * @brief       Send command strobe to the radio. Returns status byte read
 *              during transfer of command strobe. Validation of provided
 *              is not done. Function assumes chip is ready.
 *
 * input parameters
 *
 * @param       cmd - command strobe
 *
 * output parameters
 *
 * @return      status byte
 */

/*******************************************************************************
 * @fn          trxReadWriteBurstSingle
 *
 * @brief       When the address byte is sent to the SPI slave, the next byte
 *              communicated is the data to be written or read. The address
 *              byte that holds information about read/write -and single/
 *              burst-access is provided to this function.
 *
 *              Depending on these two bits this function will write len bytes to
 *              the radio in burst mode or read len bytes from the radio in burst
 *              mode if the burst bit is set. If the burst bit is not set, only
 *              one data byte is communicated.
 *
 *              NOTE: This function is used in the following way:
 *
 *              TRXEM_SPI_BEGIN();
 *              while(TRXEM_MISO_PORT_IN & TRXEM_SPI_MISO_PIN);
 *              ...[Depending on type of register access]
 *              trxReadWriteBurstSingle(uint8_t addr,uint8_t *pData,uint16_t len);
 *              TRXEM_SPI_END();
 *
 * input parameters
 *
 * @param       none
 *
 * output parameters
 *
 * @return      void
 */
static void trxReadWriteBurstSingle(uint8_t addr,uint8_t *pData,uint16_t len) {

	uint16_t i;
	/* Communicate len number of bytes: if RX - the procedure sends 0x00 to push bytes from slave*/
  if(addr&RADIO_READ_ACCESS)
  {
    if(addr&RADIO_BURST_ACCESS)
    {
      for (i = 0; i < len; i++)
      {
          TRXEM_SPI_TX(0);            /* Possible to combining read and write as one access type */
          TRXEM_SPI_WAIT_DONE();
          *pData = TRXEM_SPI_RX();     /* Store pData from last pData RX */
          pData++;
      }
    }
    else
    {
      TRXEM_SPI_TX(0);
      TRXEM_SPI_WAIT_DONE();
      *pData = TRXEM_SPI_RX();
    }
  }
  else
  {
    if(addr&RADIO_BURST_ACCESS)
    {
      /* Communicate len number of bytes: if TX - the procedure doesn't overwrite pData */
      for (i = 0; i < len; i++)
      {
        TRXEM_SPI_TX(*pData);
        TRXEM_SPI_WAIT_DONE();
        pData++;
      }
    }
    else
    {
      TRXEM_SPI_TX(*pData);
      TRXEM_SPI_WAIT_DONE();
    }
  }
  return;
}
