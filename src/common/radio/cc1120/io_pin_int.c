#include "io_pin_int.h"

static uint8_t ioPort1PinHasIsr;
static uint8_t ioPort2PinHasIsr;

// Function pointer arrays
static void (*ioPort1IsrTable[8])(void);
static void (*ioPort2IsrTable[8])(void);

// Function prototypes
__interrupt void ioPort1Isr(void);
__interrupt void ioPort2Isr(void);

void ioPinIntRegister(uint32_t ui32Base, uint8_t ui8Pins, void (*pfnIntHandler)(void)) {

    uint8_t ui8Cnt;
    uint16_t ui16IntState;
    ui16IntState = __get_interrupt_state();
    __disable_interrupt();

    if(pfnIntHandler) {
        switch(ui32Base) {
        case IO_PIN_PORT_1:
            ioPort1PinHasIsr |=  ui8Pins;
            P1IFG &= (~ui8Pins);
            break;
        case IO_PIN_PORT_2:
            ioPort2PinHasIsr |=  ui8Pins;
            P2IFG &= (~ui8Pins);
            break;
        }
    }

    for(ui8Cnt = 0; ui8Cnt < 8; ui8Cnt++) {
        if(ui8Pins & (1 << ui8Cnt)) {
            switch(ui32Base) {
            case IO_PIN_PORT_1:
                ioPort1IsrTable[ui8Cnt] = pfnIntHandler;
                break;
            case IO_PIN_PORT_2:
                ioPort2IsrTable[ui8Cnt] = pfnIntHandler;
                break;
            default:
                break;
            }
        }
    }

    __set_interrupt_state(ui16IntState);
}

void ioPinIntUnregister(uint32_t ui32Base, uint8_t ui8Pins) {

    uint16_t ui16IntState = __get_interrupt_state();
    __disable_interrupt();
    ioPinIntRegister(ui32Base, ui8Pins, 0);
    switch(ui32Base) {
    case IO_PIN_PORT_1:
        ioPort1PinHasIsr &= ~ui8Pins;
        break;
    case IO_PIN_PORT_2:
        ioPort2PinHasIsr &= ~ui8Pins;
        break;
    }

    __set_interrupt_state(ui16IntState);
}

void ioPinIntEnable(uint32_t ui32Base, uint8_t ui8Pins) {

    switch(ui32Base) {
    case IO_PIN_PORT_1:
        P1IE |= ui8Pins;
        return;
    case IO_PIN_PORT_2:
        P2IE |= ui8Pins;
        return;
    default:
        return;
    }
}

void ioPinIntDisable(uint32_t ui32Base, uint8_t ui8Pins) {

    switch(ui32Base) {
    case IO_PIN_PORT_1:
        P1IE &= (~ui8Pins);
        return;
    case IO_PIN_PORT_2:
        P2IE &= (~ui8Pins);
        return;
    default:
        return;
    }
}

void ioPinIntTypeSet(uint32_t ui32Base, uint8_t ui8Pins, uint8_t ui8IntType) {

    switch(ui8IntType) {
    case IO_PIN_FALLING_EDGE:
        switch(ui32Base) {
        case IO_PIN_PORT_1:
            P1IES |= ui8Pins;
            return;
        case IO_PIN_PORT_2:
            P2IES |= ui8Pins;
            return;
        default:
            return;
        }

    case IO_PIN_RISING_EDGE:
        switch(ui32Base) {
        case IO_PIN_PORT_1:
            P1IES &= ~ui8Pins;
            return;
        case IO_PIN_PORT_2:
            P2IES &= ~ui8Pins;
            return;
        default:
            return;
        }
    default:
        return;
    }
}

uint8_t ioPinIntStatus(uint32_t ui32Base, uint8_t ui8Pins) {

    switch(ui32Base) {
    case IO_PIN_PORT_1:
        return (P1IFG & ui8Pins);
    case IO_PIN_PORT_2:
        return (P2IFG & ui8Pins);
    default:
        return (0);
    }
}

void ioPinIntClear(uint32_t ui32Base, uint8_t ui8Pins) {

    switch(ui32Base) {
    case IO_PIN_PORT_1:
        P1IFG &= ~(ui8Pins);
        return;
    case IO_PIN_PORT_2:
        P2IFG &= ~(ui8Pins);
        return;
    }
}

#pragma vector=PORT1_VECTOR
__interrupt void ioPort1Isr(void) {

    register uint8_t ui8IntBm, ui8IsrBm;
    ui8IntBm = P1IFG;
    ui8IntBm &= P1IE;
    ui8IsrBm = (ui8IntBm & ioPort1PinHasIsr);

    if((ui8IsrBm & BIT0)) {
        (*ioPort1IsrTable[0])();
    }
    if((ui8IsrBm & BIT1)) {
        (*ioPort1IsrTable[1])();
    }
    if((ui8IsrBm & BIT2)) {
        (*ioPort1IsrTable[2])();
    }
    if((ui8IsrBm & BIT3)) {
        (*ioPort1IsrTable[3])();
    }
    if((ui8IsrBm & BIT4)) {
        (*ioPort1IsrTable[4])();
    }
    if((ui8IsrBm & BIT5)) {
        (*ioPort1IsrTable[5])();
    }
    if((ui8IsrBm & BIT6)) {
        (*ioPort1IsrTable[6])();
    }
    if((ui8IsrBm & BIT7)) {
        (*ioPort1IsrTable[7])();
    }

    P1IFG &= (~ui8IntBm);

#ifndef IO_PIN_KEEP_POWER_MODE_ON_EXIT
    __low_power_mode_off_on_exit();
#endif
}

#pragma vector=PORT2_VECTOR
__interrupt void ioPort2Isr(void) {

    register uint8_t ui8IntBm, ui8IsrBm;
    ui8IntBm = P2IFG;
    ui8IntBm &= P2IE;
    ui8IsrBm = (ui8IntBm & ioPort2PinHasIsr);

    if((ui8IsrBm & BIT0)) {
        (*ioPort2IsrTable[0])();
    }
    if((ui8IsrBm & BIT1)) {
        (*ioPort2IsrTable[1])();
    }
    if((ui8IsrBm & BIT2)) {
        (*ioPort2IsrTable[2])();
    }
    if((ui8IsrBm & BIT3)) {
        (*ioPort2IsrTable[3])();
    }
    if((ui8IsrBm & BIT4)) {
        (*ioPort2IsrTable[4])();
    }
    if((ui8IsrBm & BIT5)) {
        (*ioPort2IsrTable[5])();
    }
    if((ui8IsrBm & BIT6)) {
        (*ioPort2IsrTable[6])();
    }
    if((ui8IsrBm & BIT7)) {
        (*ioPort2IsrTable[7])();
    }

    P2IFG &= (~ui8IntBm);

#ifndef IO_PIN_KEEP_POWER_MODE_ON_EXIT
    __low_power_mode_off_on_exit();
#endif
}
