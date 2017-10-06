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
#include "cc112x_spi.h"
#include "io_pin_int.h"
#include "radio_utils.h"
#include "plow_bot_utils.h"
#include "hal_spi_rf_trxeb.h"
#include "retarget_print_util.h"
#include "cc112x_easy_link_reg_config.h"

#define TX_GPIO_ISR_PIN     BIT2
#define TX_GPIO_ISR_PORT    IO_PIN_PORT_1
#define RETARGET_PRINTF     1

/* Pipes to UART if enabled */
#if (RETARGET_PRINTF == 1)
    #include "retarget_print_util.h"
#endif

static void runRxPacketTest(void);
static void radioRxCompleteHandler(uint8_t *bytes, size_t len);
static int packetCount = 0;

void main(void) {

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
    /* Start test */
    runRxPacketTest();
}

static void radioRxCompleteHandler(uint8_t *bytes, size_t len) {

    //LOG_DEBUG("Rx handler: Packet count: %i \r\n", packetCount);
    //dumpHex(bytes, len);
}

static void runRxPacketTest(void) {

    RadioConfig config = {.mode = RadioModeRx, .isrPort = TX_GPIO_ISR_PORT, .isrPin = TX_GPIO_ISR_PIN};

    initRadioWithConfig(&config);
    /* Infinite loop */
    while(1) {
        packetCount++;
        uint8_t rxBuffer[MAX_RX_BUF_LEN] = {0};
        RadioPacket packet = {.payload = rxBuffer, .len = sizeof(rxBuffer), .handler = (RadioRxTxHandler)radioRxCompleteHandler};
        receivePacket(&packet);
    }
}
