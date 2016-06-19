/*
* This file is part of the libopencm3 project.
*
* Copyright (C) 2012 Benjamin Vernoux <titanmkd@gmail.com>
*
* This library is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this library.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <libopencm3/lpc43xx/ipc.h>
#include <libopencm3/lpc43xx/creg.h>
#include <libopencm3/lpc43xx/rgu.h>

/* Set M0 in reset mode */
void ipc_halt_m0(void)
{
	volatile uint32_t rst_active_status1;

	/* Check if M0 is reset by reading status */
	rst_active_status1 = RESET_ACTIVE_STATUS1;

	/* If the M0 has reset not asserted, halt it... */
	while (rst_active_status1 & RESET_CTRL1_M0APP_RST) {
		RESET_CTRL1 = ((~rst_active_status1) | RESET_CTRL1_M0APP_RST);
		rst_active_status1 = RESET_ACTIVE_STATUS1;
	}
}

void ipc_start_m0(uint32_t cm0_baseaddr)
{
	volatile uint32_t rst_active_status1;

	/* Set M0 memory mapping to point to start of M0 image */
	CREG_M0APPMEMMAP = cm0_baseaddr;

	/* Start/run M0 core */

	/* Release Slave from reset, first read status */
	rst_active_status1 = RESET_ACTIVE_STATUS1;

	/* If the M0 is being held in reset, release it */
	/* 1 = no reset, 0 = reset */
	while (!(rst_active_status1 & RESET_CTRL1_M0APP_RST)) {
		RESET_CTRL1 = ((~rst_active_status1) & ~RESET_CTRL1_M0APP_RST);
		rst_active_status1 = RESET_ACTIVE_STATUS1;
	}
}

void ipc_m0apptxevent_clear(void) {
	CREG_M0TXEVENT &= ~CREG_M0TXEVENT_TXEVCLR;
}

/* Set M0s in reset mode (only for LPC4370) */
void ipc_halt_m0s(void)
{
	volatile uint32_t rst_active_status0;

	/* Check if M0s is reset by reading status */
	rst_active_status0 = RESET_ACTIVE_STATUS0;

	/* If the M0s has reset not asserted, halt it... */
	while (rst_active_status0 & RESET_CTRL0_M0_SUB_RST) {
		RESET_CTRL0 = ((~rst_active_status0) | RESET_CTRL0_M0_SUB_RST);
		rst_active_status0 = RESET_ACTIVE_STATUS0;
	}
}

/* Start M0s (only for LPC4370) */
void ipc_start_m0s(uint32_t cm0s_baseaddr)
{
	volatile uint32_t rst_active_status0;

	/* Set M0s memory mapping to point to start of M0 image */
	CREG_M0SUBMEMMAP = cm0s_baseaddr;

	/* Start/run M0s core */

	/* Release Slave from reset, first read status */
	rst_active_status0 = RESET_ACTIVE_STATUS0;

	/* If the M0 is being held in reset, release it */
	/* 1 = no reset, 0 = reset */
	while (!(rst_active_status0 & RESET_CTRL0_M0_SUB_RST)) {
		RESET_CTRL0 = ((~rst_active_status0) & ~RESET_CTRL0_M0_SUB_RST);
		rst_active_status0 = RESET_ACTIVE_STATUS0;
	}
}

/* Clear event M0s (only for LPC4370) */
void ipc_m0subtxevent_clear(void) {
	CREG_M0SUBTXEVENT &= ~CREG_M0SUBTXEVENT_TXEVCLR;
}
