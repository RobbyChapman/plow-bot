/*
 * plow_bot_rx.c
 *
 *  Created on: Sep 25, 2017
 *      Author: Robert.Chapman
 */

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <msp430.h>
#include "hal_spi_rf_trxeb.h"
#include "cc112x_spi.h"
#include "io_pin_int.h"
#include "cc112x_easy_link_reg_config.h"
#include "plow_bot_utils.h"
#include "radio_utils.h"

#define txGpioIsrPin    BIT2

static void runRxPacketTest(void);
static void radioRxCompleteHandler(uint8_t *bytes, size_t len);

void main(void) {

    /* Stop watchdog timer */
    WDTCTL = WDTPW | WDTHOLD;
    /* Configure radio for SPI */
    configRadioSpi();
    /* Set default register values */
    configRadioRegisters();
    /* We need to enable global interrupts */
    __enable_interrupt();
    /* Quick RX test */
    runRxPacketTest();
}

static void radioRxCompleteHandler(uint8_t *bytes, size_t len) {

    printf("We have the RX packet! \n");
    dumpHex(bytes, len);
}

static void runRxPacketTest(void) {

    RadioConfig config = {.mode = RadioModeRx, .isrPort = IO_PIN_PORT_1, .isrPin = txGpioIsrPin};

    initRadioWithConfig(&config);
    /* Infinite loop */
    while(1) {
        uint8_t rxBuffer[MAX_RX_BUF_LEN] = {0};
        RadioPacket packet = {.payload = rxBuffer, .len = sizeof(rxBuffer), .handler = (RadioRxTxHandler)radioRxCompleteHandler};
        receivePacket(&packet);
    }
}
