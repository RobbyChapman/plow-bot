/*
 * radio_utils.c
 *
 *  Created on: Sep 25, 2017
 *      Author: Robert.Chapman
 */

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <msp430.h>
#include "cc112x_spi.h"
#include "io_pin_int.h"
#include "radio_utils.h"
#include "hal_spi_rf_trxeb.h"
#include "cc112x_easy_link_reg_config.h"

#define FS_CHP_INDEX            2
#define FS_VCO2_INDEX           0
#define FS_VCO4_INDEX           1
#define VCDAC_START_OFFSET      2
#define RX_FIFO_ERROR           0x11
#define ISR_ACTION_REQUIRED     1
#define ISR_IDLE                0

static void radioRxISR(void);
static void radioTxISR(void);
static void setRadioInterrupt(RadioConfig *config);

static RadioConfig radioConfig;
static volatile uint8_t  packetSemaphore;

static void setRadioInterrupt(RadioConfig *config) {

    if (!config->isr) {
        /* Set defaults */
        config->isr = (config->mode == RadioModeTx) ? radioTxISR : radioRxISR;
    }
    /* Copy over configuration */
    memcpy(&radioConfig, config, sizeof(*config));
}

void initRadioWithConfig(RadioConfig *config) {

    /* Clear configuration */
    memset(&radioConfig, 0x00, sizeof(radioConfig));
    /* Set defaults for interrupt */
    setRadioInterrupt(config);
    /* Connect ISR function to GPIO2 */
    ioPinIntRegister(config->isrPort, config->isrPin, config->isr);
    /* Interrupt on falling edge */
    ioPinIntTypeSet(config->isrPort, config->isrPin, IO_PIN_FALLING_EDGE);
    /* Clear ISR flag */
    ioPinIntClear(config->isrPort, config->isrPin);
    /* Enable interrupt */
    ioPinIntEnable(config->isrPort, config->isrPin);
    /* Calibrate radio according to errata */
    calibrateRadio();
    if (config->mode == RadioModeRx) {
        /* Set radio in RX */
        trxSpiCmdStrobe(CC112X_SRX);
    }
}

void transmitPacket(RadioPacket *packet) {

    /* Write packet to TX FIFO */
    cc112xSpiWriteTxFifo(packet->payload, packet->len);
    /* Strobe TX to send packet */
    trxSpiCmdStrobe(CC112X_STX);
    /* Wait until transmission is finished */
    while(packetSemaphore != ISR_ACTION_REQUIRED);
     packetSemaphore = ISR_IDLE;
     if (packet->handler) packet->handler(packet->payload, packet->len);
}

void receivePacket(RadioPacket *packet) {

    uint8_t marcState;
    uint8_t numBytes;

    /* Wait for packet received interrupt */
    if (packetSemaphore == ISR_ACTION_REQUIRED) {
        /* Read number of bytes in RX FIFO */
        cc112xSpiReadReg(CC112X_NUM_RXBYTES, &numBytes, 1);
        packet->len = numBytes;
        /* Check that we have bytes in FIFO */
        if (numBytes != 0) {
            /* Read MARCSTATE to check for RX FIFO error */
            cc112xSpiReadReg(CC112X_MARCSTATE, &marcState, 1);
            /* Mask out MARCSTATE bits and check if we have a RX FIFO error */
            if ((marcState & 0x1F) == RX_FIFO_ERROR) {
                /* Flush RX FIFO */
                trxSpiCmdStrobe(CC112X_SFRX);
            } else {
                /* Read n bytes from RX FIFO */
                cc112xSpiReadRxFifo(packet->payload, packet->len);
                /* Check CRC ok (CRC_OK: bit7 in second status byte). This assumes status bytes are appended in RX_FIFO
                 (PKT_CFG1.APPEND_STATUS = 1) If CRC is disabled the CRC_OK field will read 1 */
                if (packet->payload[packet->len - 1] & 0x80) {
                    /* Finished, notify caller */
                    if (packet->handler) packet->handler(packet->payload, packet->len);
                }
            }
        }
        /* Reset packet semaphore */
        packetSemaphore = ISR_IDLE;
        /* Set radio back in RX */
        setRadioAsRx();
    }
}

void setRadioAsRx(void) {

    trxSpiCmdStrobe(CC112X_SRX);
}

void calibrateRadio(void) {

    uint8_t original_fs_cal2;
    uint8_t calResults_for_vcdac_start_high[3] = {0};
    uint8_t calResults_for_vcdac_start_mid[3] = {0};
    uint8_t marcstate;
    uint8_t writeByte;

    /* 1) Set VCO cap-array to 0 (FS_VCO2 = 0x00) */
    writeByte = 0x00;
    cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
    /* 2) Start with high VCDAC (original VCDAC_START + 2): */
    cc112xSpiReadReg(CC112X_FS_CAL2, &original_fs_cal2, 1);
    writeByte = original_fs_cal2 + VCDAC_START_OFFSET;
    cc112xSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1);
    /* 3) Calibrate and wait for calibration to be done (radio back in IDLE state) */
    trxSpiCmdStrobe(CC112X_SCAL);
    do {
        cc112xSpiReadReg(CC112X_MARCSTATE, &marcstate, 1);
    } while (marcstate != 0x41);
    /* 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with high VCDAC_START value */
    cc112xSpiReadReg(CC112X_FS_VCO2, &calResults_for_vcdac_start_high[FS_VCO2_INDEX], 1);
    cc112xSpiReadReg(CC112X_FS_VCO4, &calResults_for_vcdac_start_high[FS_VCO4_INDEX], 1);
    cc112xSpiReadReg(CC112X_FS_CHP, &calResults_for_vcdac_start_high[FS_CHP_INDEX], 1);
    /* 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00) */
    writeByte = 0x00;
    cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
    /* 6) Continue with mid VCDAC (original VCDAC_START) */
    writeByte = original_fs_cal2;
    cc112xSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1);
    /* 7) Calibrate and wait for calibration to be done (radio back in IDLE state) */
    trxSpiCmdStrobe(CC112X_SCAL);
    do {
        cc112xSpiReadReg(CC112X_MARCSTATE, &marcstate, 1);
    } while (marcstate != 0x41);
    /* 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with mid VCDAC_START value */
    cc112xSpiReadReg(CC112X_FS_VCO2, &calResults_for_vcdac_start_mid[FS_VCO2_INDEX], 1);
    cc112xSpiReadReg(CC112X_FS_VCO4, &calResults_for_vcdac_start_mid[FS_VCO4_INDEX], 1);
    cc112xSpiReadReg(CC112X_FS_CHP, &calResults_for_vcdac_start_mid[FS_CHP_INDEX], 1);
    /* 9) Write back highest FS_VCO2 and corresponding FS_VCO and FS_CHP result */
    if (calResults_for_vcdac_start_high[FS_VCO2_INDEX] >
        calResults_for_vcdac_start_mid[FS_VCO2_INDEX]) {
        writeByte = calResults_for_vcdac_start_high[FS_VCO2_INDEX];
        cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
        writeByte = calResults_for_vcdac_start_high[FS_VCO4_INDEX];
        cc112xSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1);
        writeByte = calResults_for_vcdac_start_high[FS_CHP_INDEX];
        cc112xSpiWriteReg(CC112X_FS_CHP, &writeByte, 1);
    } else {
        writeByte = calResults_for_vcdac_start_mid[FS_VCO2_INDEX];
        cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
        writeByte = calResults_for_vcdac_start_mid[FS_VCO4_INDEX];
        cc112xSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1);
        writeByte = calResults_for_vcdac_start_mid[FS_CHP_INDEX];
        cc112xSpiWriteReg(CC112X_FS_CHP, &writeByte, 1);
    }
}

void configRadioRegisters(void) {

    uint8_t writeByte = 0x00;

    /* Reset radio */
    trxSpiCmdStrobe(CC112X_SRES);
    /* Write registers to radio */
    for(uint16_t i = 0; i < (sizeof(preferredSettings)/sizeof(registerSetting_t)); i++) {
        writeByte = preferredSettings[i].data;
        cc112xSpiWriteReg(preferredSettings[i].addr, &writeByte, 1);
    }
}

void configRadioSpi(void) {

    trxRfSpiInterfaceInit(0);
}

static void radioRxISR(void) {

    /* Set packet semaphore */
    packetSemaphore = ISR_ACTION_REQUIRED;
    /* Clear ISR flag */
    ioPinIntClear(radioConfig.isrPort, radioConfig.isrPin);
}

static void radioTxISR(void) {

    /* Set packet semaphore */
    packetSemaphore = ISR_ACTION_REQUIRED;
    /* Clear ISR flag */
    ioPinIntClear(radioConfig.isrPort, radioConfig.isrPin);
}

