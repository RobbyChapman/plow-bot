/*
 * plow_bot_util.h
 *
 *  Created on: Sep 25, 2017
 *      Author: Robert.Chapman
 */

#ifndef PLOW_BOT_UTIL_H_
#define PLOW_BOT_UTIL_H_

#include <msp430.h>
#include <stdint.h>

#define PKTLEN                  30

#define st(x)      do { x } while (__LINE__ == -1)
#define NOP()      asm(" nop")

void waitMs(uint16_t mSec);
void createPacket(uint8_t txBuffer[]);

#endif
