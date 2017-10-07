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
#include <stddef.h>

#define PKTLEN                  30

#define st(x)      do { x } while (__LINE__ == -1)
#define NOP()      asm(" nop")

void waitMs(uint16_t mSec);
void waitUs(uint16_t uSec);
void createPacket(uint8_t txBuffer[]);
void dumpHex(uint8_t *bytes, size_t len);

#endif
