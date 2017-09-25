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
#include "plow_bot_utils.h"
#include "radio_utils.h"
#include "plow_joystick.h"

#define TX_GPIO_ISR_PIN     BIT2
#define TX_GPIO_ISR_PORT    1

static void runTxPacketTest(void);
static void radioTxCompleteHandler(uint8_t *bytes, size_t len);

int main(void) {

    /* Stop watchdog timer */
    WDTCTL = WDTPW | WDTHOLD;
    /* Configure radio for SPI */
    configRadioSpi();
    /* Set default register values */
    configRadioRegisters();
    /* We need to enable global interrupts */
    __enable_interrupt();
    /* Quick RX test */
    runTxPacketTest();
}

static void radioTxCompleteHandler(uint8_t *bytes, size_t len) {

    printf("We have the TX packet! \n");
}

static void runTxPacketTest(void) {

    /* Initialize packet buffer of size PKTLEN + 1 */
    uint8_t txBuffer[PKTLEN+1] = {0};
    RadioConfig config = {.mode = RadioModeTx, .isrPort = TX_GPIO_ISR_PORT, .isrPin = TX_GPIO_ISR_PIN};

    initRadioWithConfig(&config);
    /* Infinite loop */
    while(1) {
        /* Create a random packet with PKTLEN + 2 byte packet counter + n x random bytes */
        createPacket(txBuffer);
        RadioPacket packet = {.payload = txBuffer, .len = sizeof(txBuffer), .handler = (RadioRxTxHandler)radioTxCompleteHandler};
        transmitPacket(&packet);
        waitMs(3*(rand()%10+3));
    }
}
