/*
 * retarget_print_util.c
 *
 *  Created on: Sep 28, 2017
 *      Author: Robert.Chapman
 */

#include <stdio.h>
#include <msp430.h>
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdarg.h>
#include <string.h>

void initPrintTarget(void) {

    /* USCI_A3 UART operation */
    P6SEL1 &= ~(BIT0 | BIT1);
    P6SEL0 |= (BIT0 | BIT1);
    /* Put eUSCI in reset */
    UCA3CTLW0 = UCSWRST;
    /* CLK = SMCLK */
    UCA3CTLW0 |= UCSSEL__SMCLK;
    /* 1000000/115200 = 8.68 */
    UCA3BRW = 8;
    /* 1000000/115200 - INT(1000000/115200)=0.68 UCBRSx value = 0xD6 (See UG) */
    UCA3MCTLW = 0xD600;
    /* Release from reset */
    UCA3CTLW0 &= ~UCSWRST;
}

int fputc(int _c, register FILE *_fp) {

    while (!(UCA3IFG & UCTXIFG));
    UCA3TXBUF = (unsigned char)_c;

    return ((unsigned char) _c);
}

int fputs(const char *_ptr, register FILE *_fp) {

    unsigned int i, len;

    len = strlen(_ptr);
    for (i = 0; i < len; i++) {
        while (!(UCA3IFG & UCTXIFG));
        UCA3TXBUF = (unsigned char) _ptr[i];
    }

    return len;
}
