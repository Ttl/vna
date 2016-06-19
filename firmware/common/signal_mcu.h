/*
 * Copyright 2012-2016 Benjamin Vernoux <bvernoux@airspy.com>
 *
 * This file is part of AirSpy.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */
#ifndef __SIGNAL_MCU__
#define __SIGNAL_MCU__

/* ---------- provided operations: ------------------------------------------ */

/* SEV causes an event to be signaled to all processors in a multiprocessor system
It also sets the local event register, see Power management (sleep mode, deep sleep mode of System Control Register for more details).
Instruction for Cortex M0, M3/M4 */
__attribute__ ((always_inline)) static inline void signal_sev(void)
{
  /* make sure all data transactions complete before next instruction is executed */
  __asm("dsb");
  __asm("sev");
}

/*
Wait For Event.
If the event register is 0, WFE suspends execution until one of the following events occurs:
 - an exception, unless masked by the exception mask registers or the current priority level
 - an exception enters the Pending state, if SEVONPEND in the System Control Register is set
 - a Debug Entry request, if debug is enabled
 - an event signaled by a peripheral or another processor in a multiprocessor system using the SEV instruction.
If the event register is 1, WFE clears it to 0 and completes immediately.
Instruction for Cortex M0, M3/M4
*/
__attribute__ ((always_inline)) static inline void signal_wfe(void)
{
  __asm("wfe");
}
/*
Wait For Interrupt.
WFI suspends execution until one of the following events occurs:
 - an exception
 - an interrupt becomes pending, which would preempt if PRIMASK was clear
 - a Debug Entry request, regardless of whether debug is enabled.
Instruction for Cortex M0, M3/M4
*/
__attribute__ ((always_inline)) static inline void signal_wfi(void)
{
  __asm("wfi");
}


#endif /* __SIGNAL_MCU__ */
