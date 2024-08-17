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

#include "global.h"
#include "owb.h"

#include <easy-pdk/calibrate.h>


// IMPORTANT: This value must be 16-bit aligned because it's used by the ldt16 instruction. The most reliable way to
// align this with SDCC at the moment is to make it the FIRST VARIABLE IN THIS FILE.
volatile uint16_t T16Value;


#include "interrupt.c"



int main(void)
{
    OWBInit();

    // IMPORTANT: PxDIER is a WRITE-ONLY register, so we can't use instructions that set/clear individual bits, not
    // even set0/set1 (yes, they do seem to do a read-modify-write operation on the entire register). We'll have to
    // setup the entire register in one go here.
    PADIER = (1u << OWB_PIN)
#ifdef OWB_DEBUG_ENABLED
            | (1u << DBG_PIN)
#endif
            ;

    __engint();

    while (1);
}

unsigned char __sdcc_external_startup(void)
{
#if F_CPU == 8000000
    PDK_SET_SYSCLOCK(SYSCLOCK_IHRC_8MHZ);
#elif F_CPU == 4000000
    PDK_SET_SYSCLOCK(SYSCLOCK_IHRC_4MHZ);
#elif F_CPU == 2000000
    PDK_SET_SYSCLOCK(SYSCLOCK_IHRC_2MHZ);
#elif F_CPU == 1000000
    PDK_SET_SYSCLOCK(SYSCLOCK_IHRC_1MHZ);
#else
#error Invalid value for F_CPU
#endif
    EASY_PDK_CALIBRATE_IHRC(F_CPU, TARGET_VDD_MV);

#ifdef MISCLVR
    MISCLVR = MISCLVR_2V5;
#endif
    
    return 0;
}
