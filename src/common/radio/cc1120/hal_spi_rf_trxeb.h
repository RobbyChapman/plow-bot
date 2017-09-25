#ifndef HAL_SPI_RF_TRXEB_H
#define HAL_SPI_RF_TRXEB_H

#include <msp430.h>
#include <stdint.h>

#define st(x)      do { x } while (__LINE__ == -1)
#define NOP()      asm(" nop")

#define TRXEM_PORT_SEL0      P2SEL
#define TRXEM_PORT_SEL0      P2SEL
#define TRXEM_PORT_SEL       P2SEL

#define TRXEM_CS_PORT_OUT    P4OUT
#define TRXEM_MOSI_PORT_DIR  P5DIR
#define TRXEM_CS_PORT_DIR    P4DIR
#define TRXEM_MISO_PORT_IN   P5IN /* MISO input P5:0 */

#define TRXEM_CS_PORT_SEL1   P4SEL1
#define TRXEM_SPI_PORT_SEL0  P5SEL0
#define TRXEM_SPI_PORT_SEL1  P5SEL1

#define TRXEM_SPI_MOSI_PIN   BIT0
#define TRXEM_SPI_MISO_PIN   BIT1
#define TRXEM_SPI_SCLK_PIN   BIT2
#define TRXEM_SPI_SC_N_PIN   BIT4

#define RF_RESET_N_PORT_OUT  P8OUT
#define RF_RESET_N_PORT_SEL  P8SEL
#define RF_RESET_N_PORT_DIR  P8DIR
#define RF_RESET_N_PIN       BIT3

#define RADIO_BURST_ACCESS   0x40
#define RADIO_SINGLE_ACCESS  0x00
#define RADIO_READ_ACCESS    0x80
#define RADIO_WRITE_ACCESS   0x00

/* Bit fields in the chip status byte */
#define STATUS_CHIP_RDYn_BM             0x80
#define STATUS_STATE_BM                 0x70
#define STATUS_FIFO_BYTES_AVAILABLE_BM  0x0F

//st( RF_CS_N_PORT_OUT &= ~RF_CS_N_PIN; NOP(); )
/* Macros for Tranceivers(TRX) */
#define TRXEM_SPI_BEGIN()              st( TRXEM_CS_PORT_OUT &= ~TRXEM_SPI_SC_N_PIN; NOP();)
#define TRXEM_SPI_TX(x)                st( UCB1IFG &= ~UCRXIFG; UCB1TXBUF= (x); )
#define TRXEM_SPI_WAIT_DONE()          st( while(!(UCB1IFG & UCRXIFG)); )
#define TRXEM_SPI_RX()                 UCB1RXBUF
#define TRXEM_SPI_WAIT_MISO_LOW(x)     st( uint8 count = 200; \
                                           while(TRXEM_MISO_PORT_IN & TRXEM_SPI_MISO_PIN) \
                                           { \
                                              __delay_cycles(5000); \
                                              count--; \
                                              if (count == 0) break; \
                                           } \
                                           if(count>0) (x) = 1; \
                                           else (x) = 0; )

#define TRXEM_SPI_END()                st( NOP(); TRXEM_CS_PORT_OUT |= TRXEM_SPI_SC_N_PIN; )

typedef struct {
  uint16_t  addr;
  uint8_t   data;
}registerSetting_t;

typedef uint8_t rfStatus_t;

void trxRfSpiInterfaceInit(uint8_t clockDivider);
rfStatus_t trx8BitRegAccess(uint8_t accessType, uint8_t addrByte, uint8_t *pData, uint16_t len);
rfStatus_t trxSpiCmdStrobe(uint8_t cmd);

/* CC112X specific prototype function */
rfStatus_t trx16BitRegAccess(uint8_t accessType, uint8_t extAddr, uint8_t regAddr, uint8_t *pData, uint8_t len);

#endif
