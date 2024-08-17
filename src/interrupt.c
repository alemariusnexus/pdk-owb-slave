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


// The code below was tested to (barely) work to specifications when running the CPU at 4MHz. 8MHz is safer, and 2MHz
// is definitely too slow.
#if F_CPU < 4000000
#warning 1-Wire slave code might not work at CPU frequencies lower than 4MHz!
#endif


void interrupt(void) __interrupt(0) __naked // Naked for micro-optimization
{
    // There is a very time-critical section at the start of this ISR, specifically for the READ0 operation. For all
    // other 1-Wire operations, we have a relatively generous time buffer, because we have an entire 1-Wire time slot
    // to recognize and handle them.
    // READ however starts with a relatively short LOW pulse from the master (>= 5us), and if a 0-bit should be read,
    // the slave has to extend this LOW pulse for a while WITHOUT allowing the bus to go idle. This means we have to
    // enter the ISR (including the delay in recognizing the LOW pulse, which is quite significant, especially when
    // using OWB_INT_USE_COMP), recognize the READ0, and pull the bus low within 5us, which is not a lot of time.
    // For this reason, the block between here and the actual pulling low of the bus is micro-optimized quite a lot.

    // NOTE: The minimum LOW pulse length of 5us from the master comes from the minimum specified in the "Application
    // Note 126 Timing Calculation Worksheet" from Analog Devices linked here:
    //      https://www.analog.com/en/resources/technical-articles/1wire-communication-through-software.html
    // Some naughty masters actually send even shorter pulses than this (e.g. the Arduino OneWire library), which this
    // code simply can't handle in time (at least on 4MHz SYSCLOCK). In that case, the extended LOW signal for READ0
    // might not be continuous with the master's own LOW pulse (i.e. the bus might go HIGH for a bit in-between). This
    // isn't actually a problem if the master only samples the signal once, with enough delay after the pulse, but it's
    // not how 1-Wire is supposed to look. For such masters, it might also be necessary to disable
    // OWB_SKIP_SHORT_PULSES.

    // NOTE: This code reports a spurious WRITE0 or READ if the operation is actually a RESET. This is because we have
    // to recognize WRITE0 and especially READ _before_ the end of their LOW pulse, in which case the LOW pulse could
    // still become a RESET. This should hopefully not be a problem, since the RESET would reset any corrupted state
    // caused by these spurious recognitions.

    // Prolog. We delay saving the compiler's p register until after the time-critical block to save 2 cycles. This is
    // valid as long as we don't use the p register (e.g. by writing to T16C).
    __asm__(
            "push af\n"
            );

    // Enable timer. T16C should already be at 0 right now.
    T16M |= T16M_CLK_SYSCLK;

    // This is an optimized version of:
    //
    //      if (    (INTRQ & OWB_LOW_DETECT_IRQ_FLAG)
    //          &&  (OWBLLStateFlags & OWB_STATE_FLAG_NEXT_IS_READ)
    //          &&  (OWBLLCurrentBitValue == 0)
    //      ) { ... }
    //
    // It handles all of these checks with the one variable OWBLLNextRead0INTRQFlag, which must be set to the exact
    // value OWB_LOW_DETECT_IRQ_FLAG if and only if a READ0 should be recognized next. This is very confusing, but it
    // saves a lot of cycles that we desperately need here.
    if (INTRQ & OWBLLNextRead0INTRQFlag) {

        // This is either R0 or RST. We have to assume R0 for now

#ifdef OWB_SKIP_SHORT_PULSES
        // Skip the extended LOW pulse if the bus isn't LOW. It should still be LOW if it's an actual R, so this avoids
        // detecting glitches as R. The ASM here requires fewer cycles than what SDCC generates, especially on PDK13.
        __asm
            t0sn.io __pa, #(OWB_PIN)
            goto 1$
        __endasm;
#endif

        // Extend the master's LOW pulse for R0
        OWBLLSetLow();
        DbgPulse();

        // ***** End of time-critical block for READ0 *****

        // Wait for end of R0 pulse
        OWBLLWaitForT16(OWB_TIMING_R0_0);
        OWBLLSetInput();

        if (OWBLLStateFlags & OWB_STATE_FLAG_DELAYED_SWITCH_TO_WRITE) {
            OWBLLSwitchToWriteImmediately();
        } else {
            OWBLLSetupNextRead();
        }

        OWBLLStateFlags |= OWB_STATE_FLAG_MIGHT_BE_RST;

#ifdef OWB_SKIP_SHORT_PULSES
        __asm__("1$:\n");
#endif
    } else if (INTRQ & OWB_LOW_DETECT_IRQ_FLAG) {
        // Not a R0, but might still be R1, W1, W0 or RST

        if (OWBLLStateFlags & OWB_STATE_FLAG_NEXT_IS_READ) {
            // R1 or RST. In case of R1, we don't have to do anything. We just need to check whether it becomes RST

            if (OWBLLStateFlags & OWB_STATE_FLAG_DELAYED_SWITCH_TO_WRITE) {
                OWBLLSwitchToWriteImmediately();
            } else {
                OWBLLSetupNextRead();
            }

            OWBLLStateFlags |= OWB_STATE_FLAG_MIGHT_BE_RST;
        } else {
            // Detected start of a W1/W0 (or RST) operation

            // Wait until end of LOW or W0 time reached

            do {
                OWBLLGetT16Value();
            } while (!OWBLLGetValue()  &&  T16Value < OWB_TIMING_W0_0_MIN);

            if (T16Value >= OWB_TIMING_W0_0_MIN) {
                // This is either W0 or RST. We have to assume W0 for now

                // Report W0
                OWBLLCurrentBitValue = 0;
                OWBWriteBit();
                OWBLLStateFlags |= OWB_STATE_FLAG_MIGHT_BE_RST;
#ifdef OWB_SKIP_SHORT_PULSES
            } else if (T16Value >= OWB_TIMING_W1_0_MIN) {
#else
            } else {
#endif
                // W1 detected (short LOW pulse)
                OWBLLCurrentBitValue = 1;
                OWBWriteBit();
#ifdef OWB_SKIP_SHORT_PULSES
            } else {
                // LOW pulse too short even for W1 -> consider it a glitch and ignore it
            }
#else
            }
#endif
        }
    }

    // This is usually part of the prolog generated by SDCC. We delay it until now since we didn't use the p register
    // until now, and we want to squeeze out every cycle in the time-critical section above.
    __asm__(
            "mov a, p\n"
            "push af\n"
            );

    if (INTRQ & OWB_LOW_DETECT_IRQ_FLAG) {
        // Clear IRQ flag only now. We delayed it until now to squeeze out more cycles at the beginning.
        INTRQ &= ~OWB_LOW_DETECT_IRQ_FLAG;

        if (OWBLLStateFlags & OWB_STATE_FLAG_MIGHT_BE_RST) {
            // Wait until end of LOW or RST time reached
            do {
                OWBLLGetT16Value();
            } while (!OWBLLGetValue()  &&  T16Value < OWB_TIMING_RST_0_MIN);

            if (T16Value >= OWB_TIMING_RST_0_MIN  ||  (OWBLLStateFlags & OWB_STATE_FLAG_TIMER_OVERFLOW)) {
                // RST detected (very long LOW pulse)
                OWBReset();

                // Wait until the end of the RST LOW pulse
                while (!OWBLLGetValue());

                // Leave bus idle for a while before the presence pulse
                T16C = 0;
                OWBLLWaitForT16(OWB_TIMING_RST_1);

                // Send presence pulse
                T16C = 0;
                OWBLLSetLow();

                OWBLLSwitchToWriteImmediately();

                // Wait for end of presence pulse
                OWBLLWaitForT16(OWB_TIMING_RST_PP);
                OWBLLSetInput();

                // Reset IRQ signal again. Our own presence pulse will have falsely set it.
                INTRQ &= ~OWB_LOW_DETECT_IRQ_FLAG;
            }
        }

        // OWB operation complete (excluding final idle time) -> disable and reset timer
        T16M &= (uint8_t) ~T16M_CLK_SYSCLK;
        T16C = 0;
        OWBLLStateFlags &= ~OWB_STATE_FLAG_MIGHT_BE_RST;
    }

    // Epilog
    __asm__(
            "pop af\n"
            "mov p, a\n"
            "pop af\n"
            "reti\n"
            );
}
