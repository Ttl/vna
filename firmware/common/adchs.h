/*
 * Copyright 2013-2016 Benjamin Vernoux <bvernoux@airspy.com>
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

#ifndef __ADCHS_H
#define __ADCHS_H

#ifdef __cplusplus
extern "C"
{
#endif

#define ADCHS_TRIGGER() ((LPC_ADCHS->TRIGGER = 1))


void ADCHS_deinit(void);
void ADCHS_init(void);
void ADCHS_DMA_init(uint32_t dest_addr, uint8_t packed);
void ADCHS_desc_init(uint8_t chan_num);
void ADCHS_stop(uint8_t conf_num);
void ADCHS_restart_dma(void);

void adchs_isr(void);
void dma_isr(void);

//XXX: FIXME
#define USB_BULK_BUFFER_START (0x20008000)

#define ADCHS_DATA_BUFFER_SIZE_BYTE (32768)
#define ADCHS_DATA_BUFFER (USB_BULK_BUFFER_START)

#define ADCHS_DMA_NUM_LLI (4) /* Corresponds to number of transfer */
#define ADCHS_DMA_NB_BUFFER ( (ADCHS_DMA_NUM_LLI/2) )
#define ADCHS_DATA_TRANSFER_SIZE_BYTE ( ((ADCHS_DATA_BUFFER_SIZE_BYTE/ADCHS_DMA_NUM_LLI)*2) ) /* Size of Each Transfer in Byte */

#define ADCHS_DMA_WRITE 7
#define ADCHS_DMA_READ  8

#define ADCHS_DMA_READ_SRC  (LPC_ADCHS_BASE + 512)
#define ADCHS_DMA_WRITE_DST (LPC_ADCHS_BASE + 10*4) /* LAST_SAMPLE[0] */

#define LPC_GPDMA_BASE (0x40002000)
#define LPC_GPDMA ((LPC_GPDMA_Type*) LPC_GPDMA_BASE)

#define __I volatile /* Input Read Only */
#define __O volatile /* Output Write Only */
#define __IO volatile /* In/Out Read/Write */
typedef struct {                 /*!< (@ 0x40002000) GPDMA Structure        */
  __I  uint32_t INTSTAT;         /*!< (@ 0x40002000) DMA Interrupt Status Register */
  __I  uint32_t INTTCSTAT;       /*!< (@ 0x40002004) DMA Interrupt Terminal Count Request Status Register */
  __O  uint32_t INTTCCLEAR;      /*!< (@ 0x40002008) DMA Interrupt Terminal Count Request Clear Register */
  __I  uint32_t INTERRSTAT;      /*!< (@ 0x4000200C) DMA Interrupt Error Status Register */
  __O  uint32_t INTERRCLR;       /*!< (@ 0x40002010) DMA Interrupt Error Clear Register */
  __I  uint32_t RAWINTTCSTAT;    /*!< (@ 0x40002014) DMA Raw Interrupt Terminal Count Status Register */
  __I  uint32_t RAWINTERRSTAT;   /*!< (@ 0x40002018) DMA Raw Error Interrupt Status Register */
  __I  uint32_t ENBLDCHNS;       /*!< (@ 0x4000201C) DMA Enabled Channel Register */
  __IO uint32_t SOFTBREQ;        /*!< (@ 0x40002020) DMA Software Burst Request Register */
  __IO uint32_t SOFTSREQ;        /*!< (@ 0x40002024) DMA Software Single Request Register */
  __IO uint32_t SOFTLBREQ;       /*!< (@ 0x40002028) DMA Software Last Burst Request Register */
  __IO uint32_t SOFTLSREQ;       /*!< (@ 0x4000202C) DMA Software Last Single Request Register */
  __IO uint32_t CONFIG;          /*!< (@ 0x40002030) DMA Configuration Register */
  __IO uint32_t SYNC;            /*!< (@ 0x40002034) DMA Synchronization Register */
  __I  uint32_t RESERVED0[50];
  __IO uint32_t C0SRCADDR;       /*!< (@ 0x40002100) DMA Channel Source Address Register */
  __IO uint32_t C0DESTADDR;      /*!< (@ 0x40002104) DMA Channel Destination Address Register */
  __IO uint32_t C0LLI;           /*!< (@ 0x40002108) DMA Channel Linked List Item Register */
  __IO uint32_t C0CONTROL;       /*!< (@ 0x4000210C) DMA Channel Control Register */
  __IO uint32_t C0CONFIG;        /*!< (@ 0x40002110) DMA Channel Configuration Register */
  __I  uint32_t RESERVED1[3];
  __IO uint32_t C1SRCADDR;       /*!< (@ 0x40002120) DMA Channel Source Address Register */
  __IO uint32_t C1DESTADDR;      /*!< (@ 0x40002124) DMA Channel Destination Address Register */
  __IO uint32_t C1LLI;           /*!< (@ 0x40002128) DMA Channel Linked List Item Register */
  __IO uint32_t C1CONTROL;       /*!< (@ 0x4000212C) DMA Channel Control Register */
  __IO uint32_t C1CONFIG;        /*!< (@ 0x40002130) DMA Channel Configuration Register */
  __I  uint32_t RESERVED2[3];
  __IO uint32_t C2SRCADDR;       /*!< (@ 0x40002140) DMA Channel Source Address Register */
  __IO uint32_t C2DESTADDR;      /*!< (@ 0x40002144) DMA Channel Destination Address Register */
  __IO uint32_t C2LLI;           /*!< (@ 0x40002148) DMA Channel Linked List Item Register */
  __IO uint32_t C2CONTROL;       /*!< (@ 0x4000214C) DMA Channel Control Register */
  __IO uint32_t C2CONFIG;        /*!< (@ 0x40002150) DMA Channel Configuration Register */
  __I  uint32_t RESERVED3[3];
  __IO uint32_t C3SRCADDR;       /*!< (@ 0x40002160) DMA Channel Source Address Register */
  __IO uint32_t C3DESTADDR;      /*!< (@ 0x40002164) DMA Channel Destination Address Register */
  __IO uint32_t C3LLI;           /*!< (@ 0x40002168) DMA Channel Linked List Item Register */
  __IO uint32_t C3CONTROL;       /*!< (@ 0x4000216C) DMA Channel Control Register */
  __IO uint32_t C3CONFIG;        /*!< (@ 0x40002170) DMA Channel Configuration Register */
  __I  uint32_t RESERVED4[3];
  __IO uint32_t C4SRCADDR;       /*!< (@ 0x40002180) DMA Channel Source Address Register */
  __IO uint32_t C4DESTADDR;      /*!< (@ 0x40002184) DMA Channel Destination Address Register */
  __IO uint32_t C4LLI;           /*!< (@ 0x40002188) DMA Channel Linked List Item Register */
  __IO uint32_t C4CONTROL;       /*!< (@ 0x4000218C) DMA Channel Control Register */
  __IO uint32_t C4CONFIG;        /*!< (@ 0x40002190) DMA Channel Configuration Register */
  __I  uint32_t RESERVED5[3];
  __IO uint32_t C5SRCADDR;       /*!< (@ 0x400021A0) DMA Channel Source Address Register */
  __IO uint32_t C5DESTADDR;      /*!< (@ 0x400021A4) DMA Channel Destination Address Register */
  __IO uint32_t C5LLI;           /*!< (@ 0x400021A8) DMA Channel Linked List Item Register */
  __IO uint32_t C5CONTROL;       /*!< (@ 0x400021AC) DMA Channel Control Register */
  __IO uint32_t C5CONFIG;        /*!< (@ 0x400021B0) DMA Channel Configuration Register */
  __I  uint32_t RESERVED6[3];
  __IO uint32_t C6SRCADDR;       /*!< (@ 0x400021C0) DMA Channel Source Address Register */
  __IO uint32_t C6DESTADDR;      /*!< (@ 0x400021C4) DMA Channel Destination Address Register */
  __IO uint32_t C6LLI;           /*!< (@ 0x400021C8) DMA Channel Linked List Item Register */
  __IO uint32_t C6CONTROL;       /*!< (@ 0x400021CC) DMA Channel Control Register */
  __IO uint32_t C6CONFIG;        /*!< (@ 0x400021D0) DMA Channel Configuration Register */
  __I  uint32_t RESERVED7[3];
  __IO uint32_t C7SRCADDR;       /*!< (@ 0x400021E0) DMA Channel Source Address Register */
  __IO uint32_t C7DESTADDR;      /*!< (@ 0x400021E4) DMA Channel Destination Address Register */
  __IO uint32_t C7LLI;           /*!< (@ 0x400021E8) DMA Channel Linked List Item Register */
  __IO uint32_t C7CONTROL;       /*!< (@ 0x400021EC) DMA Channel Control Register */
  __IO uint32_t C7CONFIG;        /*!< (@ 0x400021F0) DMA Channel Configuration Register */
} LPC_GPDMA_Type;

#define LPC_ADCHS_BASE (0x400F0000)
#define __I volatile /* Input Read Only */
#define __O volatile /* Output Write Only */
#define __IO volatile /* In/Out Read/Write */
typedef struct {
  __O  uint32_t FLUSH;              /*!< (@ 0x400F0000) A/D Flush FIFO */
  __IO uint32_t DMA_REQ;            /*!< (@ 0x400F0004) A/D DMA request a DMA write to load a descriptor table from memory */
  __I  uint32_t FIFO_STS;           /*!< (@ 0x400F0008) A/D Full / count / empty status */
  __IO uint32_t FIFO_CFG;           /*!< (@ 0x400F000C) A/D FIFO configuration - regular or packed samples */
  __O  uint32_t TRIGGER;            /*!< (@ 0x400F0010) A/D Trigger to initiate timer and descriptor table processing */
  __IO uint32_t DSCR_STS;           /*!< (@ 0x400F0014) A/D Descriptor processing status register */
  __IO uint32_t POWER_DOWN;         /*!< (@ 0x400F0018) A/D ADC power down control */
  __IO uint32_t CONFIG;             /*!< (@ 0x400F001C) A/D ADC configuration register */
  __IO uint32_t THR_A;              /*!< (@ 0x400F0020) A/D Threshold register A */
  __IO uint32_t THR_B;              /*!< (@ 0x400F0024) A/D Threshold register B */
  __I  uint32_t LAST_SAMPLE[6];     /*!< (@ 0x400F0028 to 0x400F003C) A/D Last sample registers - sample data and results of window comparator */
  __I  uint32_t RESERVED0[48];
  __IO uint32_t ADC_DEBUG;          /*!< (@ 0x400F0100) A/D Debug Register*/
  __IO uint32_t ADC_SPEED;          /*!< (@ 0x400F0104) A/D Speed setting register */
  __IO uint32_t POWER_CONTROL;      /*!< (@ 0x400F0108) A/D Power control register*/
  __I  uint32_t RESERVED1[61];
  __I  uint32_t FIFO_OUTPUT[16];    /*!< (@ 0x400F0200) A/D FIFO output results */
  __I  uint32_t RESERVED2[48];
  __IO uint32_t DESCRIPTOR_0[8];    /*!< (@ 0x400F0300) A/D Descriptor entries table 0 */
  __IO uint32_t DESCRIPTOR_1[8];    /*!< (@ 0x400F0320) A/D Descriptor entries table 1 */
  __I  uint32_t RESERVED3[752];
  __O  uint32_t CLR_EN0;            /*!< (@ 0x400F0F00) A/D Interupt 0 bit mask fields Clear enable */
  __O  uint32_t SET_EN0;            /*!< (@ 0x400F0F04) A/D Interrupt 0 bit mask fields Set enable */
  __I  uint32_t MASK0;              /*!< (@ 0x400F0F08) A/D Interrupt 0 enable register */
  __I  uint32_t STATUS0;            /*!< (@ 0x400F0F0C) A/D Interrtpt 0 status register */
  __O  uint32_t CLR_STAT0;          /*!< (@ 0x400F0F10) A/D Interrupt 0 Clear Status */
  __O  uint32_t SET_STAT0;          /*!< (@ 0x400F0F14) A/D Interrupt 0 set status */
  __I  uint32_t RESERVED4[2];
  __O  uint32_t CLR_EN1;            /*!< (@ 0x400F0F20) A/D Interrupt 1 clear mask */
  __O  uint32_t SET_EN1;            /*!< (@ 0x400F0F24) A/D Interrupt 1 bit mask fields Set enable */
  __I  uint32_t MASK1;              /*!< (@ 0x400F0F28) A/D Interrupt 1 enable register */
  __I  uint32_t STATUS1;            /*!< (@ 0x400F0F2C) A/D Interrtpt 1 status register */
  __O  uint32_t CLR_STAT1;          /*!< (@ 0x400F0F30) A/D Interrupt 1 Clear Status */
  __O  uint32_t SET_STAT1;          /*!< (@ 0x400F0F34) A/D Interrupt 1 set status */
} LPC_ADCHS_Type;

#define LPC_ADCHS ((LPC_ADCHS_Type *)LPC_ADCHS_BASE)

#define STAT0_FIFO_LEVEL_TRIG   (0x1<<0)
#define STAT0_FIFO_FULL         (0x1<<0)
#define STAT0_FIFO_EMPTY        (0x1<<1)
#define STAT0_FIFO_OVERFLOW     (0x1<<2)
#define STAT0_DSCR_DONE         (0x1<<3)
#define STAT0_DSCR_ERROR        (0x1<<4)
#define STAT0_ADC_OVF           (0x1<<5)
#define STAT0_ADC_UNF           (0x1<<6)

#define STAT1_THCMP_BRANGE      (0x1<<0)
#define STAT1_THCMP_ARANGE      (0x1<<1)
#define STAT1_THCMP_DCROSS      (0x1<<2)
#define STAT1_THCMP_UCROSS      (0x1<<3)
#define STAT1_OVERRUN           (0x1<<4)

#define LAST_SAMPLE_DONE        (0x1<<0)
#define LAST_SAMPLE_OVERRUN     (0x1<<1)
#define LAST_SAMPLE_THCMP_RANGE (0x0<<2) /* Range Mask */
#define LAST_SAMPLE_THCMP_CROSS (0x0<<4) /* Cross Mask */

#define FIFO_EMPTY              (0x1<<15)
#define FIFO_FULL               (0x1<<4)

#define ADC_NUM           3 // LPC4370FET100 max 3 chan only
#define ADC_FIFO_LEVEL  0x8
#define ADC_DESC_NUM      8 // Num ADCHS descriptors
#define ADC_CRS         0x3
#define ADC_CRS_SPEED   0xF

#ifdef __cplusplus
}
#endif

#endif
