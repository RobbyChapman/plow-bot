/*
 * plow_bot_utils.c
 *
 *  Created on: Sep 25, 2017
 *      Author: Robert.Chapman
 */

#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>
#include "plow_bot_utils.h"

static void waitUs(uint16_t uSec);

void createPacket(uint8_t txBuffer[]) {

    txBuffer[0] = PKTLEN;
}

void waitMs(uint16_t mSec) {
    while(mSec-- > 0) {
        waitUs(1000);
    }
}

void dumpHex(uint8_t *bytes, size_t len) {

    for (uint32_t i = 0; i < len; i++) {
        char *format = (i == (len -1)) ? "%02X\n" : "%02X-";
        printf(format, bytes[i]);
    }
}

static void waitUs(uint16_t uSec) {

    while(uSec > 3) {
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        uSec -= 2;
    }
}
