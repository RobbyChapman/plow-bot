#ifndef __IO_PIN_INT_H__
#define __IO_PIN_INT_H__

#include <stdint.h>
#include <msp430.h>

#define IO_PIN_PORT_1           1
#define IO_PIN_PORT_2           2

#define IO_PIN_FALLING_EDGE     0
#define IO_PIN_RISING_EDGE      1

extern void ioPinIntRegister(uint32_t ui32Base, uint8_t ui8Pins,
                             void (*pfnIntHandler)(void) );
extern void ioPinIntUnregister(uint32_t ui32Base, uint8_t ui8Pins);
extern void ioPinIntEnable(uint32_t ui32Base, uint8_t ui8Pins);
extern void ioPinIntDisable(uint32_t ui32Base, uint8_t ui8Pins);
extern void ioPinIntTypeSet(uint32_t ui32Base, uint8_t ui8Pins,
                            uint8_t ui8IntType);
extern uint8_t ioPinIntStatus(uint32_t ui32Base, uint8_t ui8Pins);
extern void ioPinIntClear(uint32_t ui32Base, uint8_t ui8Pins);


#endif /* #ifndef __IO_PIN_INT_H__ */
