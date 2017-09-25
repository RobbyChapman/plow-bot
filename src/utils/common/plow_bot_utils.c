/*
 * plow_bot_utils.c
 *
 *  Created on: Sep 25, 2017
 *      Author: Robert.Chapman
 */

#include <stdint.h>
#include <stdlib.h>
#include "plow_bot_utils.h"

static void waitUs(uint16_t uSec);

void createPacket(uint8_t txBuffer[]) {

    txBuffer[0] = PKTLEN;
    txBuffer[1] = (uint8_t) (44 >> 8);
    txBuffer[2] = (uint8_t)  44;

    for(uint8_t i = 3; i < (PKTLEN + 1); i++) {
        txBuffer[i] = (uint8_t)rand();
    }
}

void waitMs(uint16_t mSec) {
    while(mSec-- > 0) {
        waitUs(1000);
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
