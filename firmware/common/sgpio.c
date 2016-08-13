/*
 * Copyright 2012 Jared Boone <jared@sharebrained.com>
 * Copyright 2013 Benjamin Vernoux <titanmkd@gmail.com>
 *
 * This file is part of HackRF.
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

#include <libopencm3/lpc43xx/gpio.h>
#include <libopencm3/lpc43xx/scu.h>
#include <libopencm3/lpc43xx/sgpio.h>

#include <hackrf_core.h>

#include <sgpio.h>

//Clock division value
#define DITHER_PRESET 20

static const uint_fast8_t slice_indices[] = {
		SGPIO_SLICE_A,
		SGPIO_SLICE_I,
		SGPIO_SLICE_E,
		SGPIO_SLICE_J,
		SGPIO_SLICE_C,
		SGPIO_SLICE_K,
		SGPIO_SLICE_F,
		SGPIO_SLICE_L,
	};


uint32_t xorshf96(void) {
    static uint32_t x=123456789, y=362436069, z=521288629;
    uint32_t t;

    x ^= x << 16;
    x ^= x >> 5;
    x ^= x << 1;

    t = x;
    x = y;
    y = z;
    z = t ^ x ^ y;

    return z;
}

void sgpio_configure_pin_functions() {
    //Dither output pin P0_0
	scu_pinmux(SCU_PINMUX_SGPIO0, SCU_GPIO_FAST | SCU_CONF_FUNCTION3);
}

void sgpio_test_interface() {
	const uint_fast8_t out_pin = 0; // Output

	SGPIO_GPIO_OENREG = 0; // All inputs for the moment.

	// Disable all counters during configuration
	SGPIO_CTRL_ENABLE = 0;

	// Make all SGPIO controlled by SGPIO's "GPIO" registers
	for (uint_fast8_t i = 0; i < 16; i++) {
		SGPIO_OUT_MUX_CFG(i) =
			  SGPIO_OUT_MUX_CFG_P_OE_CFG(0)
			| SGPIO_OUT_MUX_CFG_P_OUT_CFG(4);
	}

	// Set SGPIO output values.
	SGPIO_GPIO_OUTREG =
		  (1L << out_pin);

	// Enable SGPIO pin outputs.
	SGPIO_GPIO_OENREG =
		  (1L << out_pin);

	while (1) {
        int v = xorshf96() & 0x01;
        SGPIO_GPIO_OUTREG = (v << out_pin);
	}
}

void fill_rng(void) {
	const uint_fast8_t slice_count = 8;

    for(uint_fast8_t i=0; i<slice_count; i++) {
		const uint_fast8_t slice_index = slice_indices[i];
        SGPIO_REG_SS(slice_index) = xorshf96();
    }
}

void sgpio_configure(void) {
	// Disable all counters during configuration
	SGPIO_CTRL_ENABLE = 0;

    SGPIO_GPIO_OUTREG =
        (1L << 0)	// SGPIO0 output
		;
	SGPIO_GPIO_OENREG =
        (1L << 0);

	/* SGPIO0 to SGPIO7 */
	for(uint_fast8_t i=0; i<8; i++) {
		SGPIO_OUT_MUX_CFG(i) =
		      SGPIO_OUT_MUX_CFG_P_OE_CFG(0)
		    | SGPIO_OUT_MUX_CFG_P_OUT_CFG(0) /* 1-bit output mode */
			;
	}

	const uint_fast8_t pos = 0x1f;
	const uint_fast8_t slice_count = 8;

	uint32_t slice_enable_mask = 0;
	/* Configure Slice A, I, E, J, C, K, F, L (sgpio_slice_mode_multislice mode) */
	for(uint_fast8_t i=0; i<slice_count; i++) {
		const uint_fast8_t slice_index = slice_indices[i];
		const bool input_slice = 0; /* Only for slice0/A and RX mode set input_slice to 1 */
		const uint_fast8_t concat_order = 3; /* 0x0=Self-loop(slice0/A RX mode), 0x3=8 slices */
		const uint_fast8_t concat_enable = 1; /* 0x0=External data pin(slice0/A RX mode), 0x1=Concatenate data */

		SGPIO_MUX_CFG(slice_index) =
		      SGPIO_MUX_CFG_CONCAT_ORDER(concat_order)
		    | SGPIO_MUX_CFG_CONCAT_ENABLE(concat_enable)
		    | SGPIO_MUX_CFG_QUALIFIER_SLICE_MODE(0) /* Select qualifier slice A(0x0) */
		    | SGPIO_MUX_CFG_QUALIFIER_PIN_MODE(1) /* Select qualifier pin SGPIO9(0x1) */
		    | SGPIO_MUX_CFG_QUALIFIER_MODE(0) /* Enable */
		    | SGPIO_MUX_CFG_CLK_SOURCE_SLICE_MODE(0) /* Select clock source slice D(0x0) */
		    | SGPIO_MUX_CFG_CLK_SOURCE_PIN_MODE(0) /* Source Clock Pin 0x0 = SGPIO8 */
			| SGPIO_MUX_CFG_EXT_CLK_ENABLE(0) /* External clock signal(pin) not selected */
			;

		SGPIO_SLICE_MUX_CFG(slice_index) =
		      SGPIO_SLICE_MUX_CFG_INV_QUALIFIER(0) /* 0x0=Use normal qualifier. */
		    | SGPIO_SLICE_MUX_CFG_PARALLEL_MODE(0) /* 0x0=Shift 1bit per clock. */
		    | SGPIO_SLICE_MUX_CFG_DATA_CAPTURE_MODE(0) /* 0x0=Detect rising edge. (Condition for input bit match interrupt) */
		    | SGPIO_SLICE_MUX_CFG_INV_OUT_CLK(0) /* 0x0=Normal clock. */
		    | SGPIO_SLICE_MUX_CFG_CLKGEN_MODE(0) /* 0x0=Use internal clock */
		    | SGPIO_SLICE_MUX_CFG_CLK_CAPTURE_MODE(0) /* 0x0=Use rising clock edge, 0x1=Use falling clock edge */
		    | SGPIO_SLICE_MUX_CFG_MATCH_MODE(0) /* 0x0=Do not match data */
			;

		SGPIO_PRESET(slice_index) = DITHER_PRESET;
		SGPIO_COUNT(slice_index) = 0;
		SGPIO_POS(slice_index) =
			  SGPIO_POS_POS_RESET(pos)
			| SGPIO_POS_POS(pos)
			;
		SGPIO_REG(slice_index) = xorshf96();     // Primary output data register
		SGPIO_REG_SS(slice_index) = xorshf96();   // Shadow output data register

		slice_enable_mask |= (1 << slice_index);
	}

	// Start SGPIO operation by enabling slice clocks.
	SGPIO_CTRL_ENABLE = slice_enable_mask;
}
