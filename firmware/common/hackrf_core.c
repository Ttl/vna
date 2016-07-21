/*
 * Copyright 2012 Michael Ossmann <mike@ossmann.com>
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

#include "hackrf_core.h"
#include "rf_path.h"
#include "sgpio.h"
#include <libopencm3/lpc43xx/i2c.h>
#include <libopencm3/lpc43xx/cgu.h>
#include <libopencm3/lpc43xx/gpio.h>
#include <libopencm3/lpc43xx/scu.h>
#include <libopencm3/lpc43xx/ccu.h>
#include <libopencm3/lpc43xx/ssp.h>
#include <libopencm3/lpc43xx/creg.h>

#define WAIT_CPU_CLOCK_INIT_DELAY   (10000)

void delay(uint32_t duration)
{
	uint32_t i;

	for (i = 0; i < duration; i++)
		__asm__("nop");
}

/* clock startup for Jellybean with Lemondrop attached
Configure PLL1 to max speed (204MHz).
Note: PLL1 clock is used by M4/M0 core, Peripheral, APB1. */ 
void cpu_clock_init(void)
{
	/* use IRC as clock source for APB1 (including I2C0) */
	CGU_BASE_APB1_CLK = CGU_BASE_APB1_CLK_CLK_SEL(CGU_SRC_IRC);

	/* use IRC as clock source for APB3 */
	CGU_BASE_APB3_CLK = CGU_BASE_APB3_CLK_CLK_SEL(CGU_SRC_IRC);

	//FIXME disable I2C
	/* Kick I2C0 down to 400kHz when we switch over to APB1 clock = 204MHz */
	i2c1_init(255);

	/*
	 * 12MHz clock is entering LPC XTAL1/OSC input now.  On
	 * Jellybean/Lemondrop, this is a signal from the clock generator.  On
	 * Jawbreaker, there is a 12 MHz crystal at the LPC.
	 * Set up PLL1 to run from XTAL1 input.
	 */

	//FIXME a lot of the details here should be in a CGU driver

	/* set xtal oscillator to low frequency mode */
	CGU_XTAL_OSC_CTRL &= ~CGU_XTAL_OSC_CTRL_HF_MASK;

	/* power on the oscillator and wait until stable */
	CGU_XTAL_OSC_CTRL &= ~CGU_XTAL_OSC_CTRL_ENABLE_MASK;

	/* Wait about 100us after Crystal Power ON */
	delay(WAIT_CPU_CLOCK_INIT_DELAY);

	/* use XTAL_OSC as clock source for BASE_M4_CLK (CPU) */
	CGU_BASE_M4_CLK = (CGU_BASE_M4_CLK_CLK_SEL(CGU_SRC_XTAL) | CGU_BASE_M4_CLK_AUTOBLOCK(1));

	/* use XTAL_OSC as clock source for APB1 */
	CGU_BASE_APB1_CLK = CGU_BASE_APB1_CLK_AUTOBLOCK(1)
			| CGU_BASE_APB1_CLK_CLK_SEL(CGU_SRC_XTAL);

	/* use XTAL_OSC as clock source for APB3 */
	CGU_BASE_APB3_CLK = CGU_BASE_APB3_CLK_AUTOBLOCK(1)
			| CGU_BASE_APB3_CLK_CLK_SEL(CGU_SRC_XTAL);

	cpu_clock_pll1_low_speed();

	/* use PLL1 as clock source for BASE_M4_CLK (CPU) */
	CGU_BASE_M4_CLK = (CGU_BASE_M4_CLK_CLK_SEL(CGU_SRC_PLL1) | CGU_BASE_M4_CLK_AUTOBLOCK(1));

	/* use XTAL_OSC as clock source for PLL0USB */
	CGU_PLL0USB_CTRL = CGU_PLL0USB_CTRL_PD(1)
			| CGU_PLL0USB_CTRL_AUTOBLOCK(1)
			| CGU_PLL0USB_CTRL_CLK_SEL(CGU_SRC_XTAL);
	while (CGU_PLL0USB_STAT & CGU_PLL0USB_STAT_LOCK_MASK);

	/* configure PLL0USB to produce 480 MHz clock from 12 MHz XTAL_OSC */
	/* Values from User Manual v1.4 Table 94, for 12MHz oscillator. */
	CGU_PLL0USB_MDIV = 0x06167FFA;
	CGU_PLL0USB_NP_DIV = 0x00302062;
	CGU_PLL0USB_CTRL |= (CGU_PLL0USB_CTRL_PD(1)
			| CGU_PLL0USB_CTRL_DIRECTI(1)
			| CGU_PLL0USB_CTRL_DIRECTO(1)
			| CGU_PLL0USB_CTRL_CLKEN(1));

	/* power on PLL0USB and wait until stable */
	CGU_PLL0USB_CTRL &= ~CGU_PLL0USB_CTRL_PD_MASK;
	while (!(CGU_PLL0USB_STAT & CGU_PLL0USB_STAT_LOCK_MASK));

	/* use PLL0USB as clock source for USB0 */
	CGU_BASE_USB0_CLK = CGU_BASE_USB0_CLK_AUTOBLOCK(1)
			| CGU_BASE_USB0_CLK_CLK_SEL(CGU_SRC_PLL0USB);

	/* Switch peripheral clock over to use PLL1 (204MHz) */
	CGU_BASE_PERIPH_CLK = CGU_BASE_PERIPH_CLK_AUTOBLOCK(1)
			| CGU_BASE_PERIPH_CLK_CLK_SEL(CGU_SRC_PLL1);

	/* Switch APB1 clock over to use PLL1 (204MHz) */
	CGU_BASE_APB1_CLK = CGU_BASE_APB1_CLK_AUTOBLOCK(1)
			| CGU_BASE_APB1_CLK_CLK_SEL(CGU_SRC_PLL1);

	/* Switch APB3 clock over to use PLL1 (204MHz) */
	CGU_BASE_APB3_CLK = CGU_BASE_APB3_CLK_AUTOBLOCK(1)
			| CGU_BASE_APB3_CLK_CLK_SEL(CGU_SRC_PLL1);

    ///XXX: Disable ADCHS clock
    CGU_BASE_VADC_CLK = CGU_BASE_VADC_CLK_PD(1);

    CGU_PLL0AUDIO_CTRL = CGU_PLL0AUDIO_CTRL_PD(1)
        | CGU_PLL0AUDIO_CTRL_AUTOBLOCK(1)
        | CGU_PLL0AUDIO_CTRL_CLK_SEL(CGU_SRC_GP_CLKIN);
    while (CGU_PLL0AUDIO_STAT & CGU_PLL0AUDIO_STAT_LOCK_MASK);

    /* configure PLL0AUDIO to produce xxMHz */
    /* PLL Register settings (SEL_EXT=1):
       Mdec=31=PLL0_MDIV[16:0] => CGU_PLL0AUDIO_MDIV
       Ndec=0=PLL0_NPDIV[21:12], Pdec=21=PLL0_NPDIV[6:0] => CGU_PLL0AUDIO_NP_DIV
   */

    CGU_PLL0AUDIO_MDIV = 0;
    CGU_PLL0AUDIO_NP_DIV = 0;

    CGU_PLL0AUDIO_CTRL |= (CGU_PLL0AUDIO_CTRL_PD(1)
        | CGU_PLL0AUDIO_CTRL_SEL_EXT(1)
        | CGU_PLL1_CTRL_NSEL(0)
        | CGU_PLL1_CTRL_PSEL(0)
        | CGU_PLL0AUDIO_CTRL_CLKEN(1));

    ///* power on PLL0AUDIO and wait until stable */
    CGU_PLL0AUDIO_CTRL &= ~CGU_PLL0AUDIO_CTRL_PD(1);
    while (!(CGU_PLL0AUDIO_STAT & CGU_PLL0AUDIO_STAT_LOCK_MASK));

    ///* use PLL0AUDIO as clock source for ADCHS */
    CGU_BASE_VADC_CLK = CGU_BASE_VADC_CLK_AUTOBLOCK(1)
        | CGU_BASE_VADC_CLK_CLK_SEL(CGU_SRC_PLL0AUDIO);


    ///XXX: Audio PLL to CLK0 pin
    CGU_BASE_OUT_CLK = CGU_BASE_OUT_CLK_CLK_SEL(CGU_SRC_PLL0AUDIO);
}


/* 
Configure PLL1 to low speed (48MHz).
Note: PLL1 clock is used by M4/M0 core, Peripheral, APB1.
This function shall be called after cpu_clock_init().
This function is mainly used to lower power consumption.
*/
void cpu_clock_pll1_low_speed(void)
{
	uint32_t pll_reg;

	/* Configure PLL1 Clock (48MHz) */
	/* Integer mode:
		FCLKOUT = M*(FCLKIN/N) 
		FCCO = 2*P*FCLKOUT = 2*P*M*(FCLKIN/N) 
	*/
	pll_reg = CGU_PLL1_CTRL;
	/* Clear PLL1 bits */
	pll_reg &= ~( CGU_PLL1_CTRL_CLK_SEL_MASK | CGU_PLL1_CTRL_PD_MASK | CGU_PLL1_CTRL_FBSEL_MASK |  /* CLK SEL, PowerDown , FBSEL */
				  CGU_PLL1_CTRL_BYPASS_MASK | /* BYPASS */
				  CGU_PLL1_CTRL_DIRECT_MASK | /* DIRECT */
				  CGU_PLL1_CTRL_PSEL_MASK | CGU_PLL1_CTRL_MSEL_MASK | CGU_PLL1_CTRL_NSEL_MASK ); /* PSEL, MSEL, NSEL- divider ratios */
	/* Set PLL1 up to 12MHz * 4 = 48MHz. */
	pll_reg |= CGU_PLL1_CTRL_CLK_SEL(CGU_SRC_XTAL)
				| CGU_PLL1_CTRL_PSEL(0)
				| CGU_PLL1_CTRL_NSEL(0)
				| CGU_PLL1_CTRL_MSEL(3)
				| CGU_PLL1_CTRL_FBSEL(1)
				| CGU_PLL1_CTRL_DIRECT(1);
	CGU_PLL1_CTRL = pll_reg;
	/* wait until stable */
	while (!(CGU_PLL1_STAT & CGU_PLL1_STAT_LOCK_MASK));

	/* Wait a delay after switch to new frequency with Direct mode */
	delay(WAIT_CPU_CLOCK_INIT_DELAY);
}

/* 
Configure PLL1 (Main MCU Clock) to max speed (204MHz).
Note: PLL1 clock is used by M4/M0 core, Peripheral, APB1.
This function shall be called after cpu_clock_init().
*/
void cpu_clock_pll1_max_speed(void)
{
	uint32_t pll_reg;

	/* Configure PLL1 to Intermediate Clock (between 90 MHz and 110 MHz) */
	/* Integer mode:
		FCLKOUT = M*(FCLKIN/N) 
		FCCO = 2*P*FCLKOUT = 2*P*M*(FCLKIN/N) 
	*/
	pll_reg = CGU_PLL1_CTRL;
	/* Clear PLL1 bits */
	pll_reg &= ~( CGU_PLL1_CTRL_CLK_SEL_MASK | CGU_PLL1_CTRL_PD_MASK | CGU_PLL1_CTRL_FBSEL_MASK |  /* CLK SEL, PowerDown , FBSEL */
				  CGU_PLL1_CTRL_BYPASS_MASK | /* BYPASS */
				  CGU_PLL1_CTRL_DIRECT_MASK | /* DIRECT */
				  CGU_PLL1_CTRL_PSEL_MASK | CGU_PLL1_CTRL_MSEL_MASK | CGU_PLL1_CTRL_NSEL_MASK ); /* PSEL, MSEL, NSEL- divider ratios */
	/* Set PLL1 up to 12MHz * 8 = 96MHz. */
	pll_reg |= CGU_PLL1_CTRL_CLK_SEL(CGU_SRC_XTAL)
				| CGU_PLL1_CTRL_PSEL(0)
				| CGU_PLL1_CTRL_NSEL(0)
				| CGU_PLL1_CTRL_MSEL(7)
				| CGU_PLL1_CTRL_FBSEL(1);
	CGU_PLL1_CTRL = pll_reg;
	/* wait until stable */
	while (!(CGU_PLL1_STAT & CGU_PLL1_STAT_LOCK_MASK));

	/* Wait before to switch to max speed */
	delay(WAIT_CPU_CLOCK_INIT_DELAY);

	/* Configure PLL1 Max Speed */
	/* Direct mode: FCLKOUT = FCCO = M*(FCLKIN/N) */
	pll_reg = CGU_PLL1_CTRL;
	/* Clear PLL1 bits */
	pll_reg &= ~( CGU_PLL1_CTRL_CLK_SEL_MASK | CGU_PLL1_CTRL_PD_MASK | CGU_PLL1_CTRL_FBSEL_MASK |  /* CLK SEL, PowerDown , FBSEL */
				  CGU_PLL1_CTRL_BYPASS_MASK | /* BYPASS */
				  CGU_PLL1_CTRL_DIRECT_MASK | /* DIRECT */
				  CGU_PLL1_CTRL_PSEL_MASK | CGU_PLL1_CTRL_MSEL_MASK | CGU_PLL1_CTRL_NSEL_MASK ); /* PSEL, MSEL, NSEL- divider ratios */
	/* Set PLL1 up to 12MHz * 17 = 204MHz. */
	pll_reg |= CGU_PLL1_CTRL_CLK_SEL(CGU_SRC_XTAL)
			| CGU_PLL1_CTRL_PSEL(0)
			| CGU_PLL1_CTRL_NSEL(0)
			| CGU_PLL1_CTRL_MSEL(9) //XXX: 16
			| CGU_PLL1_CTRL_FBSEL(1)
			| CGU_PLL1_CTRL_DIRECT(1);
	CGU_PLL1_CTRL = pll_reg;
	/* wait until stable */
	while (!(CGU_PLL1_STAT & CGU_PLL1_STAT_LOCK_MASK));

}

void ssp1_init(void)
{
	/*
     * Raise ADF4158 CE pin to de-select the device
	 */
    gpio_clear(PORT_SOURCE_CE, PIN_SOURCE_CE);
    gpio_set(PORT_SOURCE_LE, PIN_SOURCE_LE);
    gpio_clear(PORT_LO_CE, PIN_LO_LE);
    gpio_set(PORT_LO_LE, PIN_LO_LE);
    gpio_clear(PORT_ATT_LE, PIN_ATT_LE);

	/* Configure SSP1 Peripheral (to be moved later in SSP driver) */
	scu_pinmux(SCU_SSP1_MISO, (SCU_SSP_IO | SCU_CONF_FUNCTION5));
	scu_pinmux(SCU_SSP1_MOSI, (SCU_SSP_IO | SCU_CONF_FUNCTION5));
	scu_pinmux(SCU_SSP1_SCK,  (SCU_SSP_IO | SCU_CONF_FUNCTION1));
    ssp1_set_mode_16bit();
}

void ssp1_set_mode_16bit(void)
{
	/* FIXME speed up once everything is working reliably */
	/*
	// Freq About 0.0498MHz / 49.8KHz => Freq = PCLK / (CPSDVSR * [SCR+1]) with PCLK=PLL1=204MHz
	const uint8_t serial_clock_rate = 32;
	const uint8_t clock_prescale_rate = 128;
	*/
	// Freq About 4.857MHz => Freq = PCLK / (CPSDVSR * [SCR+1]) with PCLK=PLL1=204MHz
	const uint8_t serial_clock_rate = 21;
	const uint8_t clock_prescale_rate = 2;
	
	ssp_init(SSP1_NUM,
		SSP_DATA_16BITS,
		SSP_FRAME_SPI,
		SSP_CPOL_0_CPHA_0,
		serial_clock_rate,
		clock_prescale_rate,
		SSP_MODE_NORMAL,
		SSP_MASTER,
		SSP_SLAVE_OUT_ENABLE);
}

void ssp1_set_mode_8bit(void)
{
	/* FIXME speed up once everything is working reliably */
	/*
	// Freq About 0.0498MHz / 49.8KHz => Freq = PCLK / (CPSDVSR * [SCR+1]) with PCLK=PLL1=204MHz
	const uint8_t serial_clock_rate = 32;
	const uint8_t clock_prescale_rate = 128;
	*/
	// Freq About 4.857MHz => Freq = PCLK / (CPSDVSR * [SCR+1]) with PCLK=PLL1=204MHz
	const uint8_t serial_clock_rate = 21;
	const uint8_t clock_prescale_rate = 2;
	
	ssp_init(SSP1_NUM,
		SSP_DATA_8BITS,
		SSP_FRAME_SPI,
		SSP_CPOL_0_CPHA_0,
		serial_clock_rate,
		clock_prescale_rate,
		SSP_MODE_NORMAL,
		SSP_MASTER,
		SSP_SLAVE_OUT_ENABLE);
}

void pin_setup(void) {
    /* GPIO pinmuxes */
	scu_pinmux(SCU_PINMUX_LED1, SCU_GPIO_NOPULL);
	scu_pinmux(SCU_PINMUX_FILTER1, SCU_GPIO_NOPULL);
	scu_pinmux(SCU_PINMUX_FILTER2, SCU_GPIO_NOPULL);
	scu_pinmux(SCU_PINMUX_FILTER3, SCU_GPIO_NOPULL);
	scu_pinmux(SCU_PINMUX_FILTER4, SCU_GPIO_NOPULL);
	scu_pinmux(SCU_PINMUX_LO_LD, SCU_GPIO_NOPULL);
	scu_pinmux(SCU_PINMUX_SOURCE_LD, SCU_GPIO_NOPULL);
	scu_pinmux(SCU_PINMUX_SOURCE_LE, SCU_GPIO_NOPULL);
	scu_pinmux(SCU_PINMUX_SOURCE_CE, SCU_GPIO_NOPULL);
	scu_pinmux(SCU_PINMUX_LO_CE, SCU_GPIO_NOPULL);
	scu_pinmux(SCU_PINMUX_LO_LE, SCU_GPIO_NOPULL);
	scu_pinmux(SCU_PINMUX_RX_MIX_ENBL, SCU_GPIO_NOPULL);
	scu_pinmux(SCU_PINMUX_LO_MUXOUT, SCU_GPIO_NOPULL);
	scu_pinmux(SCU_PINMUX_SOURCE_MUXOUT, SCU_GPIO_NOPULL);
	scu_pinmux(SCU_PINMUX_RX_V1, SCU_GPIO_NOPULL | SCU_CONF_FUNCTION4);
	scu_pinmux(SCU_PINMUX_RX_V2, SCU_GPIO_NOPULL| SCU_CONF_FUNCTION4);
	scu_pinmux(SCU_PINMUX_LO_RF_ENABLE, SCU_GPIO_NOPULL | SCU_CONF_FUNCTION4);
	scu_pinmux(SCU_PINMUX_SOURCE_RF_ENABLE, SCU_GPIO_NOPULL | SCU_CONF_FUNCTION4);
	scu_pinmux(SCU_PINMUX_TX_SW_CTRL, SCU_GPIO_NOPULL);
	scu_pinmux(SCU_PINMUX_ATT_LE, SCU_GPIO_NOPULL);
	scu_pinmux(SCU_PINMUX_PWDN, SCU_GPIO_NOPULL);
	scu_pinmux(SCU_PINMUX_GP_CLKIN, SCU_CLK_IN | SCU_CONF_FUNCTION1);
	scu_pinmux(SCU_PINMUX_I2C_SDA, SCU_CONF_FUNCTION1);
	scu_pinmux(SCU_PINMUX_I2C_SCL, SCU_CONF_FUNCTION1);

	/* Configure all GPIO as Input (safe state) */
	GPIO0_DIR = 0;
	GPIO1_DIR = 0;
	GPIO2_DIR = 0;
	GPIO3_DIR = 0;
	GPIO4_DIR = 0;
	GPIO5_DIR = 0;
	GPIO6_DIR = 0;
	GPIO7_DIR = 0;

    ////rf_disable();

    ///* Set safe GPIO states */
    gpio_set(PORT_FILTER1, PIN_FILTER1);
    gpio_clear(PORT_FILTER2, PIN_FILTER2);
    gpio_clear(PORT_FILTER3, PIN_FILTER3);
    gpio_clear(PORT_FILTER4, PIN_FILTER4);
    gpio_set(PORT_SOURCE_LE, PIN_SOURCE_LE);
    gpio_clear(PORT_SOURCE_CE, PIN_SOURCE_CE);
    gpio_clear(PORT_LO_CE, PIN_LO_CE);
    gpio_set(PORT_LO_LE, PIN_LO_LE);
    gpio_clear(PORT_RX_MIX_ENBL, PIN_RX_MIX_ENBL);
    gpio_clear(PORT_RX_V1, PIN_RX_V1);
    gpio_clear(PORT_RX_V2, PIN_RX_V2);
    gpio_clear(PORT_LO_RF_ENABLE, PIN_LO_RF_ENABLE);
    gpio_clear(PORT_SOURCE_RF_ENABLE, PIN_SOURCE_RF_ENABLE);
    gpio_clear(PORT_TX_SW_CTRL, PIN_TX_SW_CTRL);
    gpio_clear(PORT_ATT_LE, PIN_ATT_LE);
    gpio_set(PORT_PWDN, PIN_PWDN);

    /* Set GPIO directions */
	GPIO0_DIR |= (PIN_LED1 | PIN_RX_MIX_ENBL);

    GPIO1_DIR |= (PIN_FILTER1 | PIN_FILTER2 | PIN_FILTER3 | PIN_FILTER4 | PIN_SOURCE_LE | PIN_SOURCE_CE | PIN_LO_CE | PIN_LO_LE | PIN_TX_SW_CTRL | PIN_ATT_LE | PIN_PWDN);

    GPIO5_DIR |= (PIN_RX_V1 | PIN_RX_V2 | PIN_LO_RF_ENABLE | PIN_SOURCE_RF_ENABLE);

	/* Configure SSP1 Peripheral (to be moved later in SSP driver) */
////	scu_pinmux(SCU_SSP1_MISO, (SCU_SSP_IO | SCU_CONF_FUNCTION5));
////	scu_pinmux(SCU_SSP1_MOSI, (SCU_SSP_IO | SCU_CONF_FUNCTION5));
////	scu_pinmux(SCU_SSP1_SCK, (SCU_SSP_IO | SCU_CONF_FUNCTION1));

    // Disable clock outputs
    scu_pinmux(CLK0, SCU_CLK_IN | SCU_CONF_FUNCTION7);
	scu_pinmux(CLK2, SCU_CLK_IN | SCU_CONF_FUNCTION7);

    //XXX: Debug
	//scu_pinmux(CLK2, SCU_CLK_IN | SCU_CONF_FUNCTION1);

	//sgpio_configure_pin_functions();
}
