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

#include "owb.h"
#include "owbll.h"

#include <easy-pdk/serial_num.h>

#include <stdlib.h>


EASY_PDK_SERIAL_NUM(OWBROMCode);


volatile uint8_t OWBLLStateFlags = 0;

volatile uint8_t OWBLLNextRead0INTRQFlag = 0;
volatile uint8_t OWBLLCurrentBitValue;




// **********************************************************
// *                                                        *
// *                        HIGH-LEVEL                      *
// *                                                        *
// **********************************************************

// ********** READ ROM **********
uint8_t OWBREADROMByteOffset = 0;

// ********** SEARCH ROM **********
uint8_t OWBSEARCHROMCurrentByteIndex = 0;


uint8_t CurrentState = OWB_STATE_IDLE;
uint8_t CurrentByte = 0;
uint8_t CurrentBitValue = 1;


void OWBReset(void)
{
    CurrentByte = 0;
    CurrentBitValue = 1;

    CurrentState = OWB_STATE_RESET;

    OWBREADROMByteOffset = 0;
    OWBSEARCHROMCurrentByteIndex = 0;
}

void OWBWriteBit(void)
{
    if (CurrentState == OWB_STATE_SEARCH_ROM) {
        if (OWBLLGetWriteValue() == (CurrentByte & 0x01)) {
            // Bit match

            CurrentBitValue <<= 1;
            CurrentByte >>= 1;

            if (CurrentBitValue == 0) {
                // Byte finished
                OWBSEARCHROMCurrentByteIndex++;

                if (OWBSEARCHROMCurrentByteIndex == 8) {
                    // Command finished
                    CurrentState = OWB_STATE_IDLE;
                } else {
                    // Next byte
                    CurrentBitValue++; // CurrentBitValue = 1
                    CurrentByte = OWBROMCode[OWBSEARCHROMCurrentByteIndex];
                }
            }

            OWBLLSwitchToRead();
        } else {
            // Bit mismatch -> go inactive
            CurrentState = OWB_STATE_IDLE;
        }
    } else if (CurrentState == OWB_STATE_RESET) {
        if (OWBLLGetWriteValue()) {
            CurrentByte |= CurrentBitValue;
        }
        CurrentBitValue <<= 1;

        if (CurrentBitValue == 0) {
            // Received command

            if (CurrentByte == 0x33) {
                // READ ROM

                CurrentState = OWB_STATE_READ_ROM;

                CurrentByte = OWBROMCode[0];
                CurrentBitValue++; // CurrentBitValue = 1

                OWBLLSwitchToRead();
            } else if (CurrentByte == 0xF0) {
                // SEARCH ROM

                CurrentState = OWB_STATE_SEARCH_ROM;

                CurrentByte = OWBROMCode[0];
                CurrentBitValue++; // CurrentBitValue = 1

                OWBLLSwitchToRead();
            } else {
                CurrentState = OWB_STATE_IDLE;
            }
        }
    }
}

void OWBReadBit(void)
{
    if (CurrentState == OWB_STATE_SEARCH_ROM) {
        if (OWBLLStateFlags & OWB_STATE_FLAG_SEARCH_ROM_INVERT) {
            // Send inverted bit
            OWBLLSetReadValue(~CurrentByte & 0x01);
            OWBLLSwitchToWrite(); // Next is master bit
        } else {
            // Send non-inverted bit
            OWBLLSetReadValue(CurrentByte & 0x01);
        }
        OWBLLStateFlags ^= OWB_STATE_FLAG_SEARCH_ROM_INVERT; // Toggle inverted bit
    } else if (CurrentState == OWB_STATE_READ_ROM) {
        OWBLLSetReadValue(CurrentByte & 0x01);

        CurrentBitValue <<= 1;
        CurrentByte >>= 1;

        if (CurrentBitValue == 0) {
            // Finished reading byte

            OWBREADROMByteOffset++;

            if (OWBREADROMByteOffset == 8) {
                // All ROM code bytes read
                CurrentState = OWB_STATE_IDLE;
            } else {
                // Switch to next byte
                CurrentByte = OWBROMCode[OWBREADROMByteOffset];
                CurrentBitValue++; // CurrentBitValue = 1
            }
        }
    } else {
        OWBLLSetReadValue(1);
    }
}




// **********************************************************
// *                                                        *
// *                        LOW-LEVEL                       *
// *                                                        *
// **********************************************************

void OWBInit(void)
{
    PADIER = 0;

#ifdef OWB_INT_USE_COMP
    // Setup comparator to simply output the digital value of its minus input to its output.
    GPCC = 0; // Disable comparator
    // IMPORTANT: GPCS is a WRITE-ONLY register, so set it up in one go.
    GPCS = GPCS_COMP_RANGE2 | (15 << GPCS_COMP_VOLTAGE_LVL_BIT0); // Vintr = 0.125*Vdd
#if OWB_Px == PA  &&  OWB_PIN == 3
    GPCC = GPCC_COMP_PLUS_VINT_R | GPCC_COMP_MINUS_PA3 | GPCC_COMP_OUT_INVERT | GPCC_COMP_ENABLE;
#elif OWB_Px == PA  &&  OWB_PIN == 4
    GPCC = GPCC_COMP_PLUS_VINT_R | GPCC_COMP_MINUS_PA4 | GPCC_COMP_OUT_INVERT | GPCC_COMP_ENABLE;
#elif OWB_Px == PB  &&  OWB_PIN == 6
    GPCC = GPCC_COMP_PLUS_VINT_R | GPCC_COMP_MINUS_PB6 | GPCC_COMP_OUT_INVERT | GPCC_COMP_ENABLE;
#elif OWB_Px == PB  &&  OWB_PIN == 7
    GPCC = GPCC_COMP_PLUS_VINT_R | GPCC_COMP_MINUS_PB7 | GPCC_COMP_OUT_INVERT | GPCC_COMP_ENABLE;
#else
#error Invalid OWB pin: Must be configurable as minus input of comparator.
#endif
#endif

#ifdef OWB_DEBUG_ENABLED
    DBG_Px = 0;
    DBG_PxC = (1 << DBG_PIN);
#endif

    // Output value will always be LOW, because we want an open-drain port
    OWBLLSetInput();
    OWB_Px &= ~(1 << OWB_PIN);

    // Setup timer to tick at F_CPU, but disable it for now. Also reset it to 0
    T16M = T16M_CLK_DISABLE | T16M_CLK_DIV1;
    T16C = 0;

    INTRQ = 0;
#ifdef OWB_INT_USE_COMP
    // Setup interrupt on COMP rising edge (i.e. OWB pin falling edge), and enable it
#ifdef INTEGS_COMP_FALLING
    // IMPORTANT: INTEGS is a WRITE-ONLY register, so set it up in one go.
    INTEGS = INTEGS_COMP_FALLING;
#elif defined(MISC2_COMP_EDGE_INT_FALL)
    // IMPORTANT: INTEGS and MISC2 are WRITE-ONLY registers, so set them up in one go.
    INTEGS = 0;
    MISC2 = MISC2_COMP_EDGE_INT_FALL;
#else
#error Unable to select falling edge as COMP interrupt condition. Neither INTEGS nor MISC2 is supported.
#endif
    INTEN = INTEN_COMP;
#else
    // Setup interrupt on OWB pin falling edge, and enable it
    // IMPORTANT: INTEGS is a WRITE-ONLY register, so set it up in one go.
    INTEGS = INTEGS_PA0_FALLING;
    INTEN = INTEN_PA0;
#ifdef ROP
#if OWB_Px == PA  &&  OWB_PIN == 5
    ROP = ROP_INT_SRC_PA5;
#elif OWB_Px == PA  &&  OWB_PIN == 0
    ROP = ROP_INT_SRC_PA0;
#else
#error Invalid OWB pin: Must be configurable as interrupt
#endif
#else
#if OWB_Px != PA  ||  OWB_PIN != 0
#error Invalid OWB pin: Must be configurable as interrupt
#endif
#endif
#endif
}
