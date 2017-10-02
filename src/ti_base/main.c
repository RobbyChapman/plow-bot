//      This is a baseline example for reference from TI
//
//
//
//******************************************************************************
//! @file       cc112x_easy_link_tx.c
//! @brief      This program sets up an easy link between two trxEB's with
//!             CC112x EM's connected.
//!             The program can take any recomended register settings exported
//!             from SmartRF Studio 7 without any modification with exeption
//!             from the assumtions decribed below.
//
//              Notes: The following asumptions must be fulfilled for
//              the program to work:
//
//                  1.  GPIO2 has to be set up with GPIO2_CFG = 0x06
//                      PKT_SYNC_RXTX for correct interupt
//                  2.  Packet engine has to be set up with status bytes enabled
//                      PKT_CFG1.APPEND_STATUS = 1
//
//  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//      Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//      Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//      Neither the name of Texas Instruments Incorporated nor the names of
//      its contributors may be used to endorse or promote products derived
//      from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//*****************************************************************************/


/*******************************************************************************
* INCLUDES
*/
#include <stdint.h>
#include "msp430.h"
#include "hal_spi_rf_trxeb.h"
#include "cc112x_spi.h"
#include "stdlib.h"
#include "io_pin_int.h"
#include "cc112x_easy_link_reg_config.h"

/*******************************************************************************
* DEFINES
*/
#define MASTER                  0
#define TELEM_OP_CODE           0xBB

#define ISR_ACTION_REQUIRED     1
#define ISR_IDLE                0

#define PKTLEN                  30  // 1 < PKTLEN < 126
#define RX_FIFO_ERROR           0x11
#define GPIO3                   0x04
#define GPIO2                   BIT2
#define GPIO0                   0x80
uint16_t rxCountBuffer[1024] = {0};

/*******************************************************************************
* LOCAL VARIABLES
*/
static uint8_t  packetSemaphore;
uint16_t packetCounter = 0;


/*******************************************************************************
* STATIC FUNCTIONS
*/
static void initMCU(void);
static void registerConfig(void);
static void manualCalibration(void);
static void runTX(void);
static void runRX(void);
static void createPacket(uint8_t randBuffer[]);
static void radioTxISR(void);
static void radioRxISR(void);
static void waitMs(uint16_t mSec);
static void waitUs(uint16_t uSec);


/*******************************************************************************
*   @fn         main
*
*   @brief      Runs the main routine
*
*   @param      none
*
*   @return     none
*/
void main(void) {

    WDTCTL = WDTPW | WDTHOLD;

    // Initialize MCU and peripherals
    initMCU();

    // Write radio registers
    registerConfig();

    #if (MASTER == 1)
        // Enter runTX, never coming back
        runTX();
    #else
        // Enter runTX, never coming back
        runRX();
    #endif

}


/*******************************************************************************
*   @fn         runTX
*
*   @brief      Continuously sends packets on button push until button is pushed
*               again. After the radio has gone into TX the function waits for
*               interrupt that packet has been sent. Updates packet counter and
*               display for each packet sent.
*
*   @param      none
*
*   @return    none
*/
static void runTX(void) {

    // Initialize packet buffer of size PKTLEN + 1
    uint8_t txBuffer[PKTLEN+1] = {0};
    static uint16_t txPacketCount = 0;

    // Connect ISR function to GPIO2
    ioPinIntRegister(IO_PIN_PORT_1, GPIO2, &radioTxISR);

    // Interrupt on falling edge
    ioPinIntTypeSet(IO_PIN_PORT_1, GPIO2, IO_PIN_FALLING_EDGE);

    // Clear ISR flag
    ioPinIntClear(IO_PIN_PORT_1, GPIO2);

    // Enable interrupt
    ioPinIntEnable(IO_PIN_PORT_1, GPIO2);

    // Calibrate radio according to errata
    manualCalibration();

    // Infinite loop
    while(1) {
                // Update packet counter
                packetCounter++;
                txPacketCount++;

                // Create a random packet with PKTLEN + 2 byte packet
                // counter + n x random bytes
                createPacket(txBuffer);

                // Adding logic for opCode
                if (txPacketCount >= 50) {
                    txPacketCount = 0;
                    // BB is our special byte in this case
                    txBuffer[3] = TELEM_OP_CODE;
                }
                // Write packet to TX FIFO
                cc112xSpiWriteTxFifo(txBuffer, sizeof(txBuffer));

                // Strobe TX to send packet
                trxSpiCmdStrobe(CC112X_STX);

                // Wait for interrupt that packet has been sent.
                // (Assumes the GPIO connected to the radioRxTxISR function is
                // set to GPIOx_CFG = 0x06)
                while(packetSemaphore != ISR_ACTION_REQUIRED);

                // Clear semaphore flag
                packetSemaphore = ISR_IDLE;

                //Wait for a random amount of time between each packet
                waitMs(3*(rand()%10+3));
    }
}


static void runRX(void) {

    uint8_t rxBuffer[128] = {0};
    uint8_t rxBytes;
    uint8_t marcState;

    // Connect ISR function to GPIO2
    ioPinIntRegister(IO_PIN_PORT_1, GPIO2, &radioRxISR);

    // Interrupt on falling edge
    ioPinIntTypeSet(IO_PIN_PORT_1, GPIO2, IO_PIN_FALLING_EDGE);

    // Clear ISR flag
    ioPinIntClear(IO_PIN_PORT_1, GPIO2);

    // Enable interrupt
    ioPinIntEnable(IO_PIN_PORT_1, GPIO2);

    // Calibrate radio according to errata
    manualCalibration();

    // Set radio in RX
    trxSpiCmdStrobe(CC112X_SRX);

    // Infinite loop
    while(1) {

        // Wait for packet received interrupt
        if(packetSemaphore == ISR_ACTION_REQUIRED) {

            // Read number of bytes in RX FIFO
            cc112xSpiReadReg(CC112X_NUM_RXBYTES, &rxBytes, 1);

            // Check that we have bytes in FIFO
            if(rxBytes != 0) {

                // Read MARCSTATE to check for RX FIFO error
                cc112xSpiReadReg(CC112X_MARCSTATE, &marcState, 1);

                // Mask out MARCSTATE bits and check if we have a RX FIFO error
                if((marcState & 0x1F) == RX_FIFO_ERROR) {

                    // Flush RX FIFO
                    trxSpiCmdStrobe(CC112X_SFRX);
                } else {

                    // Read n bytes from RX FIFO
                    cc112xSpiReadRxFifo(rxBuffer, rxBytes);

                    // Check CRC ok (CRC_OK: bit7 in second status byte)
                    // This assumes status bytes are appended in RX_FIFO
                    // (PKT_CFG1.APPEND_STATUS = 1)
                    // If CRC is disabled the CRC_OK field will read 1
                    if(rxBuffer[rxBytes - 1] & 0x80) {
                        // Update packet counter
                        packetCounter++;
                        rxCountBuffer[packetCounter] = rxBuffer[1] | rxBuffer[2];
                        if (rxBuffer[3] == TELEM_OP_CODE) {
                            _no_operation();
                        }
                    }
                }
            }

            // Reset packet semaphore
            packetSemaphore = ISR_IDLE;

            // Set radio back in RX
            trxSpiCmdStrobe(CC112X_SRX);
        }
    }
}

/*******************************************************************************
*   @fn         radioTxISR
*
*   @brief      ISR for packet handling in TX. Sets packet semaphore
*               and clears ISR flag
*
*   @param      none
*
*   @return     none
*/
static void radioTxISR(void) {

    // Set packet semaphore
    packetSemaphore = ISR_ACTION_REQUIRED;

    // Clear ISR flag
    ioPinIntClear(IO_PIN_PORT_1, GPIO2);
}

/*******************************************************************************
*   @fn         radioRxISR
*
*   @brief      ISR for packet handling in RX. Sets packet semaphore
*               and clears ISR flag
*
*   @param      none
*
*   @return     none
*/
static void radioRxISR(void) {

    // Set packet semaphore
    packetSemaphore = ISR_ACTION_REQUIRED;

    // Clear ISR flag
    ioPinIntClear(IO_PIN_PORT_1, GPIO2);
}

/*******************************************************************************
*   @fn         initMCU
*
*   @brief      Initialize MCU and board peripherals
*
*   @param      none
*
*   @return     none
*/
static void initMCU(void) {

    // Instantiate transceiver RF SPI interface to SCLK ~ 4 MHz
    // Input parameter is clockDivider
    // SCLK frequency = SMCLK/clockDivider
    trxRfSpiInterfaceInit(2);

    // Enable global interrupt
    __enable_interrupt();
}


/*******************************************************************************
*   @fn         registerConfig
*
*   @brief      Write register settings as given by SmartRF Studio found in
*               cc112x_easy_link_reg_config.h
*
*   @param      none
*
*   @return     none
*/
static void registerConfig(void) {

    uint8_t writeByte;

    // Reset radio
    trxSpiCmdStrobe(CC112X_SRES);

    // Write registers to radio
    for(uint16_t i = 0; i < (sizeof(preferredSettings)/sizeof(registerSetting_t)); i++) {
        writeByte = preferredSettings[i].data;
        cc112xSpiWriteReg(preferredSettings[i].addr, &writeByte, 1);
    }
}


/*******************************************************************************
*   @fn         createPacket
*
*   @brief      This function is called before a packet is transmitted. It fills
*               the txBuffer with a packet consisting of a length byte, two
*               bytes packet counter and n random bytes.
*
*               The packet format is as follows:
*               |--------------------------------------------------------------|
*               |           |           |           |         |       |        |
*               | pktLength | pktCount1 | pktCount0 | rndData |.......| rndData|
*               |           |           |           |         |       |        |
*               |--------------------------------------------------------------|
*                txBuffer[0] txBuffer[1] txBuffer[2]            txBuffer[PKTLEN]
*
*   @param       Pointer to start of txBuffer
*
*   @return      none
*/
static void createPacket(uint8_t txBuffer[]) {

    txBuffer[0] = PKTLEN;                           // Length byte
    txBuffer[1] = (uint8_t) (packetCounter >> 8);     // MSB of packetCounter
    txBuffer[2] = (uint8_t)  packetCounter;           // LSB of packetCounter

    // Fill rest of buffer with random bytes
    for(uint8_t i = 3; i < (PKTLEN + 1); i++) {
        txBuffer[i] = 0x00;
    }
}

/*******************************************************************************
*   @fn         waitMs
*
*   @brief      Busy wait function. Waits the specified number of milliseconds.
*               Use assumptions about number of clock cycles needed for the
*               various instructions.
*
*               NB! This function is highly dependent on architecture and
*               compiler!
*
*   @param      uint16 mSec - number of milliseconds delay
*
*   @return     none
*/
static void waitMs(uint16_t mSec) {
    while(mSec-- > 0) {
        waitUs(1000);
    }
}

/*******************************************************************************
*   @fn         waitUs
*
*   @brief      Busy wait function. Waits the specified number of microseconds.
*               Use assumptions about number of clock cycles needed for the
*               various instructions. The duration of one cycle depends on MCLK.
*               In this HAL it is set to 8 MHz, thus 8 cycles per us.
*
*               NB! This function is highly dependent on architecture
*               and compiler!
*
*   @param      uint16 uSec - number of microseconds delay
*
*   @return     none
*/
static void waitUs(uint16_t uSec) { // 5 cycles for calling

    // The least we can wait is 3 usec:
    // ~1 usec for call, 1 for first compare and 1 for return
    while(uSec > 3) {  // 2 cycles for compare
                       // 2 cycles for jump
        NOP();         // 1 cycle for nop
        NOP();         // 1 cycle for nop
        NOP();         // 1 cycle for nop
        NOP();         // 1 cycle for nop
        NOP();         // 1 cycle for nop
        NOP();         // 1 cycle for nop
        NOP();         // 1 cycle for nop
        NOP();         // 1 cycle for nop
        uSec -= 2;     // 1 cycle for optimized decrement
    }
}                      // 4 cycles for returning

/*******************************************************************************
*   @fn         manualCalibration
*
*   @brief      Calibrates radio according to CC112x errata
*
*   @param      none
*
*   @return     none
*/
#define VCDAC_START_OFFSET 2
#define FS_VCO2_INDEX 0
#define FS_VCO4_INDEX 1
#define FS_CHP_INDEX 2
static void manualCalibration(void) {

    uint8_t original_fs_cal2;
    uint8_t calResults_for_vcdac_start_high[3];
    uint8_t calResults_for_vcdac_start_mid[3];
    uint8_t marcstate;
    uint8_t writeByte;

    // 1) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
    writeByte = 0x00;
    cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);

    // 2) Start with high VCDAC (original VCDAC_START + 2):
    cc112xSpiReadReg(CC112X_FS_CAL2, &original_fs_cal2, 1);
    writeByte = original_fs_cal2 + VCDAC_START_OFFSET;
    cc112xSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1);

    // 3) Calibrate and wait for calibration to be done
    //   (radio back in IDLE state)
    trxSpiCmdStrobe(CC112X_SCAL);

    do {
        cc112xSpiReadReg(CC112X_MARCSTATE, &marcstate, 1);
    } while (marcstate != 0x41);

    // 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with
    //    high VCDAC_START value
    cc112xSpiReadReg(CC112X_FS_VCO2,
                     &calResults_for_vcdac_start_high[FS_VCO2_INDEX], 1);
    cc112xSpiReadReg(CC112X_FS_VCO4,
                     &calResults_for_vcdac_start_high[FS_VCO4_INDEX], 1);
    cc112xSpiReadReg(CC112X_FS_CHP,
                     &calResults_for_vcdac_start_high[FS_CHP_INDEX], 1);

    // 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
    writeByte = 0x00;
    cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);

    // 6) Continue with mid VCDAC (original VCDAC_START):
    writeByte = original_fs_cal2;
    cc112xSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1);

    // 7) Calibrate and wait for calibration to be done
    //   (radio back in IDLE state)
    trxSpiCmdStrobe(CC112X_SCAL);

    do {
        cc112xSpiReadReg(CC112X_MARCSTATE, &marcstate, 1);
    } while (marcstate != 0x41);

    // 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained
    //    with mid VCDAC_START value
    cc112xSpiReadReg(CC112X_FS_VCO2,
                     &calResults_for_vcdac_start_mid[FS_VCO2_INDEX], 1);
    cc112xSpiReadReg(CC112X_FS_VCO4,
                     &calResults_for_vcdac_start_mid[FS_VCO4_INDEX], 1);
    cc112xSpiReadReg(CC112X_FS_CHP,
                     &calResults_for_vcdac_start_mid[FS_CHP_INDEX], 1);

    // 9) Write back highest FS_VCO2 and corresponding FS_VCO
    //    and FS_CHP result
    if (calResults_for_vcdac_start_high[FS_VCO2_INDEX] >
        calResults_for_vcdac_start_mid[FS_VCO2_INDEX]) {
        writeByte = calResults_for_vcdac_start_high[FS_VCO2_INDEX];
        cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
        writeByte = calResults_for_vcdac_start_high[FS_VCO4_INDEX];
        cc112xSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1);
        writeByte = calResults_for_vcdac_start_high[FS_CHP_INDEX];
        cc112xSpiWriteReg(CC112X_FS_CHP, &writeByte, 1);
    } else {
        writeByte = calResults_for_vcdac_start_mid[FS_VCO2_INDEX];
        cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
        writeByte = calResults_for_vcdac_start_mid[FS_VCO4_INDEX];
        cc112xSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1);
        writeByte = calResults_for_vcdac_start_mid[FS_CHP_INDEX];
        cc112xSpiWriteReg(CC112X_FS_CHP, &writeByte, 1);
    }
}
