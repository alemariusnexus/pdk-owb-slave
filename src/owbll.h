/*
    pdk-owb-slave - A OneWire slave implementation for Padauk microcontrollers.
    Copyright (C) 2024 David "Alemarius Nexus" Lerch

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#pragma once


// TODO: Why is the second statement in OWBSetLow() necessary, even though we set OWB_Px during initialization and
//  then never change it afterwards? Without it, the bus will sometimes not be pulled low, specifically when
//  OWB_INT_USE_COMP is enabled.
// Direct bus manipulation
#define OWBLLSetInput()     OWB_PxC &= ~(1 << OWB_PIN)
#define OWBLLSetLow()                   \
        OWB_PxC |= (1 << OWB_PIN);      \
        OWB_Px &= ~(1 << OWB_PIN)
#define OWBLLGetValue()     (OWB_Px & (1 << OWB_PIN))

// The IRQ flag used for detecting LOW pulses on the bus
#ifdef OWB_INT_USE_COMP
#define OWB_LOW_DETECT_IRQ_FLAG     INTRQ_COMP
#else
#define OWB_LOW_DETECT_IRQ_FLAG     INTRQ_PA0
#endif

// Fetch the bit for the next READ operation. Be careful with OWBLLNextRead0INTRQFlag (see its definition).
#define OWBLLSetupNextRead()                                        \
        do {                                                        \
            OWBReadBit();                                           \
            if (OWBLLCurrentBitValue) {                             \
                OWBLLNextRead0INTRQFlag = 0;                        \
            } else {                                                \
                OWBLLNextRead0INTRQFlag = OWB_LOW_DETECT_IRQ_FLAG;  \
            }                                                       \
        } while (false)

// Make the driver switch to read-mode, i.e. interpret the following 1-Wire commands as either READ or RESET.
#define OWBLLSwitchToRead()                                         \
        do {                                                        \
            if (!(OWBLLStateFlags & OWB_STATE_FLAG_NEXT_IS_READ)) { \
                OWBLLStateFlags |= OWB_STATE_FLAG_NEXT_IS_READ;     \
                OWBLLSetupNextRead();                               \
            }                                                       \
        } while (false)

// Make the driver switch to write-mode, i.e. interpret the following 1-Wire commands as either WRITE0, WRITE1 or RESET.
// NOTE: This does NOT immediately take effect, but only after the current buffered read bit is sent. This is useful
//  for changing to write-mode within OWBReadBit(), which is called ahead of the actual READ operation it applies to.
#define OWBLLSwitchToWrite()    OWBLLStateFlags |= OWB_STATE_FLAG_DELAYED_SWITCH_TO_WRITE

// Make the driver switch to write-mode IMMEDIATELY, without waiting to complete any buffered READ bits.
#define OWBLLSwitchToWriteImmediately()                             \
        OWBLLNextRead0INTRQFlag = 0;                                \
        OWBLLStateFlags &= ~OWB_STATE_FLAG_DELAYED_SWITCH_TO_WRITE; \
        OWBLLStateFlags &= ~OWB_STATE_FLAG_NEXT_IS_READ

// SDCC currently doesn't support reading T16C from C, so we have to use ASM. The destination also MUST be
// 16-bit aligned, which doesn't reliably work with temporaries.
#define OWBLLGetT16Value() __asm__("ldt16 _T16Value\n")
#define OWBLLWaitForT16(minValue) do { OWBLLGetT16Value(); } while (T16Value < (minValue))

// To be used inside OWBWriteBit() to distinguish between WRITE0 and WRITE1
#define OWBLLGetWriteValue()            OWBLLCurrentBitValue

// To be used inside OWBReadBit() to make the next READ operation either a READ0 or a READ1
#define OWBLLSetReadValue(val)          OWBLLCurrentBitValue = (val)


enum
{
    OWB_STATE_FLAG_SEARCH_ROM_INVERT        = 0x02,

    OWB_STATE_FLAG_NEXT_IS_READ             = 0x10,
    OWB_STATE_FLAG_MIGHT_BE_RST             = 0x20,
    OWB_STATE_FLAG_TIMER_OVERFLOW           = 0x40,
    OWB_STATE_FLAG_DELAYED_SWITCH_TO_WRITE  = 0x80
};



// Miscellaneous state related flags used by both the low-level and high-level driver
extern volatile uint8_t OWBLLStateFlags;

// This should be set to OWB_LOW_DETECT_IRQ_FLAG if and only if the next 1-Wire operation should be interpreted as
// a READ0. In ALL other cases, it must be 0.
// The weirdness of this is due to micro-optimization in the interrupt for the READ0 operation. See the ISR for details.
extern volatile uint8_t OWBLLNextRead0INTRQFlag;

// Bit value of the current/next READ/WRITE operation
extern volatile uint8_t OWBLLCurrentBitValue;
