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
#include "hal_spi_rf_trxeb.h"
#include "cc112x_spi.h"
#include "io_pin_int.h"
#include "cc112x_easy_link_reg_config.h"
#include "plow_bot_utils.h"
#include "radio_utils.h"

#define txGpioIsrPin    BIT2

static void runTxPacketTest(void);
static void radioTxCompleteHandler(uint8_t bytes, size_t len);

int main(void) {

    /* Stop watchdog timer */
    WDTCTL = WDTPW | WDTHOLD;
    configRadioSpi();
    configRadioRegisters();
    __enable_interrupt();
    runTxPacketTest();
}

static void radioTxCompleteHandler(uint8_t bytes, size_t len) {

    printf("We have the TX packet!");
}

static void runTxPacketTest(void) {

    /* Initialize packet buffer of size PKTLEN + 1 */
    uint8_t txBuffer[PKTLEN+1] = {0};
    RadioConfig config = {.mode = RadioModeTx, .isrPort = IO_PIN_PORT_1, .isrPin = txGpioIsrPin};

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
