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

#include "global.h"



// **********************************************************
// *                                                        *
// *                        USER CONFIG                     *
// *                                                        *
// **********************************************************

// Enable this to use the comparator peripheral for the OWB interrupt. If enabled, the OWB pin must be one that can
// be used as the minus input to the comparator. If disabled, the OWB pin must be one that can be used as an external
// interrupt pin directly.
//#define OWB_INT_USE_COMP

// Enable this to check the minimum LOW pulse length for the shortest 1-Wire operations (W1, R). This can be useful
// to skip short glitches on the bus. When disabled, even short LOW pulses will be interpreted as 1-Wire operations.
// The latter can be useful if the master sends pulses shorter than what the slave's ISR can handle (i.e. R pulses
// of less than 5us).
//#define OWB_SKIP_SHORT_PULSES

// Configuration for the OWB pin
#define OWB_PxC     PAC
#define OWB_Px      PA
#define OWB_PIN     0

// Configuration for the debug pin
//#define OWB_DEBUG_ENABLED
#ifdef OWB_DEBUG_ENABLED
#define DBG_PxC     PAC
#define DBG_Px      PA
#define DBG_PIN     4
#endif

// Convert microseconds to T16 tick values, optionally adjusting for interrupt latency
#define OWB_TIMING_US_TO_TICKS(us) ((us) * (F_CPU/1000000))
#define OWB_TIMING_US_TO_TICKS_WITH_LATENCY(us) (OWB_TIMING_US_TO_TICKS(us) > OWB_TIMING_LOW_TO_ISR_LATENCY_TICKS \
        ? (OWB_TIMING_US_TO_TICKS(us)-OWB_TIMING_LOW_TO_ISR_LATENCY_TICKS) : 0)

// Define 1-Wire timing. Note that the slave's timing isn't very accurate, so the values should include quite a bit
// of error margin.
#define OWB_TIMING_W1_0_MIN     OWB_TIMING_US_TO_TICKS_WITH_LATENCY(3)
#define OWB_TIMING_W0_0_MIN     OWB_TIMING_US_TO_TICKS_WITH_LATENCY(30)
#define OWB_TIMING_R0_0         OWB_TIMING_US_TO_TICKS_WITH_LATENCY(30)
#define OWB_TIMING_RST_0_MIN    OWB_TIMING_US_TO_TICKS_WITH_LATENCY(200)
#define OWB_TIMING_RST_1        OWB_TIMING_US_TO_TICKS_WITH_LATENCY(15)
#define OWB_TIMING_RST_PP       OWB_TIMING_US_TO_TICKS_WITH_LATENCY(150)

#define OWB_TIMING_LOW_TO_ISR_LATENCY_TICKS     8


enum OWBState
{
    OWB_STATE_IDLE,
    OWB_STATE_RESET,
    OWB_STATE_READ_ROM,
    OWB_STATE_SEARCH_ROM
};

// IMPORTANT: This value must be 16-bit aligned because it's used by the ldt16 instruction. The most reliable way to
// align this with SDCC at the moment is to make it the FIRST VARIABLE IN THIS FILE.
extern volatile uint16_t T16Value;


#ifdef OWB_DEBUG_ENABLED
#define DbgPulse()                  \
        DBG_Px |= (1 << DBG_PIN);   \
        DBG_Px &= ~(1 << DBG_PIN);  \
        __asm__("nop\n")
#else
#define DbgPulse()
#endif



void OWBInit(void);


void OWBReset(void);
void OWBWriteBit(void);
void OWBReadBit(void);
