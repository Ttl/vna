/*
 * Copyright 2012 Michael Ossmann <mike@ossmann.com>
 * Copyright 2012 Benjamin Vernoux <titanmkd@gmail.com>
 * Copyright 2012 Jared Boone <jared@sharebrained.com>
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

#ifndef __HACKRF_CORE_H
#define __HACKRF_CORE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

#define BOARD_ID 2

/*
 * SCU PinMux
 */

/* GPIO Output PinMux */

#define SCU_PINMUX_LED1     (P0_1) /* GPIO0[1] on P0_1 */

#define SCU_PINMUX_DITHER   (P0_0)
#define SCU_PINMUX_FILTER1  (P1_5)
#define SCU_PINMUX_FILTER2  (P1_6)
#define SCU_PINMUX_FILTER3  (P1_7)
#define SCU_PINMUX_FILTER4  (P1_8)
#define SCU_PINMUX_LO_LD    (P1_9)
#define SCU_PINMUX_SOURCE_LD    (P1_10)
#define SCU_PINMUX_SOURCE_LE    (P1_11)
#define SCU_PINMUX_SOURCE_CE    (P1_12)
#define SCU_PINMUX_LO_CE    (P1_13)
#define SCU_PINMUX_LO_LE    (P1_14)
#define SCU_PINMUX_RX_MIX_ENBL    (P1_15)
#define SCU_PINMUX_LO_MUXOUT    (P1_16)
#define SCU_PINMUX_SOURCE_MUXOUT    (P1_17)
#define SCU_PINMUX_RX_V1    (P2_0)
#define SCU_PINMUX_RX_V2    (P2_1)
#define SCU_PINMUX_LO_RF_ENABLE    (P2_2)
#define SCU_PINMUX_I2C_SDA    (P2_3)
#define SCU_PINMUX_I2C_SCL    (P2_4)
#define SCU_PINMUX_SOURCE_RF_ENABLE    (P2_5)
#define SCU_PINMUX_TX_SW_CTRL   (P2_11)
#define SCU_PINMUX_ATT_LE   (P2_12)
#define SCU_PINMUX_PWDN   (P2_13)
#define SCU_PINMUX_GP_CLKIN   (PF_4)

/* GPIO Input PinMux */
#define SCU_PINMUX_BOOT0    (P1_1)  /* GPIO0[8] on P1_1 */
#define SCU_PINMUX_BOOT1    (P1_2)  /* GPIO0[9] on P1_2 */
#define SCU_PINMUX_BOOT2    (P2_8)  /* GPIO5[7] on P2_8 */
#define SCU_PINMUX_BOOT3    (P2_9)  /* GPIO1[10] on P2_9 */

/* SSP1 Peripheral PinMux */
#define SCU_SSP1_MISO       (P1_3)  /* P1_3 */
#define SCU_SSP1_MOSI       (P1_4)  /* P1_4 */
#define SCU_SSP1_SCK        (P1_19) /* P1_19 */

/* SGPIO interface */
#define SCU_PINMUX_SGPIO0   (P0_0) /* D2 */
#define SCU_PINMUX_SGPIO1   (P0_1) /* D3 */
#define SCU_PINMUX_SGPIO2   (P1_15) /* D4 */
#define SCU_PINMUX_SGPIO3   (P1_16) /* D5 */
#define SCU_PINMUX_SGPIO4   (P2_0) /* D6 */
#define SCU_PINMUX_SGPIO5   (P2_1) /* D7 */
#define SCU_PINMUX_SGPIO6   (P2_2) /* D8 */
#define SCU_PINMUX_SGPIO7   (P1_0) /* D8 */
#define SCU_PINMUX_SGPIO8   (P1_12) /* CLK */
#define SCU_PINMUX_SGPIO9   (P1_13) /* CLK */
#define SCU_PINMUX_SGPIO10  (P1_14) /* D0 */
#define SCU_PINMUX_SGPIO11  (P1_17) /* D1 */

#define SCU_PINMUX_SGPIO12  (P1_18) /* NC */
#define SCU_PINMUX_SGPIO13  (P4_8) /* NC */
#define SCU_PINMUX_SGPIO14  (P1_6) /* NC */
#define SCU_PINMUX_SGPIO15  (P1_5) /* NC */

/* SPI flash */
#define SCU_SSP0_MISO       (P3_6)
#define SCU_SSP0_MOSI       (P3_7)
#define SCU_SSP0_SCK        (P3_3)
#define SCU_SSP0_SSEL       (P3_8) /* GPIO5[11] on P3_8 */
#define SCU_FLASH_HOLD      (P3_4) /* GPIO1[14] on P3_4 */
#define SCU_FLASH_WP        (P3_5) /* GPIO1[15] on P3_5 */

/*
 * GPIO Pins
 */

/* GPIO Output */
#define PIN_DITHER  (BIT0)
#define PORT_DITHER (GPIO0)

#define PIN_LED1    (BIT1)
#define PORT_LED1_3 (GPIO0)

#define PIN_FILTER1 (BIT8)
#define PORT_FILTER1 (GPIO1)

#define PIN_FILTER2 (BIT9)
#define PORT_FILTER2 (GPIO1)

#define PIN_FILTER3 (BIT0)
#define PORT_FILTER3 (GPIO1)

#define PIN_FILTER4 (BIT1)
#define PORT_FILTER4 (GPIO1)

#define PIN_LO_LD   (BIT2)
#define PORT_LO_LD  (GPIO1)

#define PIN_SOURCE_LD   (BIT3)
#define PORT_SOURCE_LD  (GPIO1)

#define PIN_SOURCE_LE   (BIT4)
#define PORT_SOURCE_LE  (GPIO1)

#define PIN_SOURCE_CE   (BIT5)
#define PORT_SOURCE_CE  (GPIO1)

#define PIN_LO_CE   (BIT6)
#define PORT_LO_CE  (GPIO1)

#define PIN_LO_LE   (BIT7)
#define PORT_LO_LE  (GPIO1)

#define PIN_RX_MIX_ENBL  (BIT2)
#define PORT_RX_MIX_ENBL  (GPIO0)

#define PIN_LO_MUXOUT  (BIT3)
#define PORT_LO_MUXOUT  (GPIO0)

#define PIN_SOURCE_MUXOUT  (BIT12)
#define PORT_SOURCE_MUXOUT  (GPIO0)

#define PIN_RX_V1  (BIT0)
#define PORT_RX_V1  (GPIO5)

#define PIN_RX_V2  (BIT1)
#define PORT_RX_V2  (GPIO5)

#define PIN_LO_RF_ENABLE  (BIT2)
#define PORT_LO_RF_ENABLE  (GPIO5)

#define PIN_SOURCE_RF_ENABLE  (BIT5)
#define PORT_SOURCE_RF_ENABLE  (GPIO5)

#define PIN_TX_SW_CTRL  (BIT11)
#define PORT_TX_SW_CTRL  (GPIO1)

#define PIN_ATT_LE  (BIT12)
#define PORT_ATT_LE  (GPIO1)

#define PIN_PWDN  (BIT13)
#define PORT_PWDN  (GPIO1)

#define PIN_FLASH_HOLD (BIT14) /* GPIO1[14] on P3_4 */
#define PIN_FLASH_WP   (BIT15) /* GPIO1[15] on P3_5 */
#define PORT_FLASH     (GPIO1)
#define PIN_SSP0_SSEL  (BIT11) /* GPIO5[11] on P3_8 */
#define PORT_SSP0_SSEL (GPIO5)

/* GPIO Input */
#define PIN_BOOT0   (BIT8)  /* GPIO0[8] on P1_1 */
#define PIN_BOOT1   (BIT9)  /* GPIO0[9] on P1_2 */
#define PIN_BOOT2   (BIT7)  /* GPIO5[7] on P2_8 */
#define PIN_BOOT3   (BIT10) /* GPIO1[10] on P2_9 */

/* Read GPIO Pin */
#define GPIO_STATE(port, pin) ((GPIO_PIN(port) & (pin)) == (pin))
#define BOOT0_STATE       GPIO_STATE(GPIO0, PIN_BOOT0)
#define BOOT1_STATE       GPIO_STATE(GPIO0, PIN_BOOT1)
#define BOOT2_STATE       GPIO_STATE(GPIO5, PIN_BOOT2)
#define BOOT3_STATE       GPIO_STATE(GPIO1, PIN_BOOT3)

typedef enum {
	TRANSCEIVER_MODE_OFF = 0,
	TRANSCEIVER_MODE_RX = 1,
	TRANSCEIVER_MODE_TX = 2
} transceiver_mode_t;

void delay(uint32_t duration);

void cpu_clock_init(void);
void cpu_clock_pll1_low_speed(void);
void cpu_clock_pll1_max_speed(void);
void ssp1_init(void);
void ssp1_set_mode_16bit(void);
void ssp1_set_mode_8bit(void);

void pin_setup(void);

#ifdef HACKRF_ONE
void enable_rf_power(void);
void disable_rf_power(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __HACKRF_CORE_H */
