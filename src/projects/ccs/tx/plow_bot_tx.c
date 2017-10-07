/*
 * plow_bot_tx.c
 *
 *  Created on: Sep 25, 2017
 *      Author: Robert.Chapman
 */



#include <msp430.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "radio_utils.h"
#include "plow_joystick.h"
#include "plow_bot_utils.h"

#define RETARGET_PRINTF     1
#define TX_GPIO_ISR_PIN     BIT2
#define TX_GPIO_ISR_PORT    1

/* Pipes to UART if enabled */
#if (RETARGET_PRINTF == 1)
    #include "retarget_print_util.h"
#endif

static void runTxPacketTest(void);
static void radioTxCompleteHandler(uint8_t *bytes, size_t len);
static int packetCount = 0;

int main(void) {

    /* Stop watchdog timer */
    WDTCTL = WDTPW | WDTHOLD;
    /* Configure radio for SPI */
    configRadioSpi();
    /* Set default register values */
    configRadioRegisters();
    /* Init UART for printf redirects */
    initPrintTarget();
    /* We need to enable global interrupts */
    __enable_interrupt();
    /* Quick RX test */
    runTxPacketTest();
}

static void radioTxCompleteHandler(uint8_t *bytes, size_t len) {

    //LOG_DEBUG("Rx handler: Packet count: %i \r\n", packetCount);
}

static void runTxPacketTest(void) {

    RadioConfig config = {.mode = RadioModeTx, .isrPort = TX_GPIO_ISR_PORT, .isrPin = TX_GPIO_ISR_PIN};

    initRadioWithConfig(&config);
    /* Infinite loop */
    while(1) {
        packetCount++;
        /* Initialize packet buffer of size PKTLEN + 1 */
        uint8_t txBuffer[PKTLEN+1] = {0};
        /* Create a random packet with PKTLEN + 2 byte packet counter + n x random bytes */
        createPacket(txBuffer);
        RadioPacket packet = {.payload = txBuffer, .len = sizeof(txBuffer), .handler = (RadioRxTxHandler)radioTxCompleteHandler};
        transmitPacket(&packet);
    }
}
