/*
 * radio_utils.h
 *
 *  Created on: Sep 25, 2017
 *      Author: Robert.Chapman
 */

#ifndef RADIO_UTILS_H_
#define RADIO_UTILS_H_

#include <stdint.h>
#include <stddef.h>

#define MAX_RX_BUF_LEN  128

typedef void (*RadioIsr)(void);
typedef void (*RadioRxTxHandler)(uint8_t *bytes, size_t len);

typedef enum RadioMode {
    RadioModeTx = 0x02,
    RadioModeRx
} RadioMode;

typedef struct RadioConfig {
    RadioMode mode;
    RadioIsr isr;
    uint32_t isrPort;
    uint8_t isrPin;
} RadioConfig;

typedef struct RadioPacket {
    uint8_t *payload;
    size_t len;
    RadioRxTxHandler handler;
} RadioPacket;

void setRadioAsRx(void);
void configRadioSpi(void);
void calibrateRadio(void);
void configRadioRegisters(void);
void receivePacket(RadioPacket *packet);
void transmitPacket(RadioPacket *packet);
void initRadioWithConfig(RadioConfig *config);


#endif
