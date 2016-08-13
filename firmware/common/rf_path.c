#include <libopencm3/lpc43xx/gpio.h>
#include <libopencm3/lpc43xx/scu.h>
#include <libopencm3/lpc43xx/ssp.h>
#include <libopencm3/lpc43xx/ritimer.h>
#include "hackrf_core.h"
#include "adchs.h"
#include "sgpio.h"

#define ENABLE_RITIMER() (RITIMER_CTRL =  (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3))
#define DISABLE_RITIMER() (RITIMER_CTRL =  (1 << 0) | (1 << 1) | (1 << 2) | (0 << 3))

static volatile uint8_t isr_done;
static volatile uint8_t isr_count;
static uint8_t isr_len_channels;
static uint8_t isr_channels[4];

static void spi_write_register(uint32_t data);
static uint32_t spi_read_register(void);

void enable_pa(void) {
    gpio_clear(PORT_PWDN, PIN_PWDN);
}

void disable_pa(void) {
    gpio_set(PORT_PWDN, PIN_PWDN);
}

void enable_mixer(void) {
    gpio_clear(PORT_RX_MIX_ENBL, PIN_RX_MIX_ENBL);
}

void disable_mixer(void) {
    gpio_set(PORT_RX_MIX_ENBL, PIN_RX_MIX_ENBL);
}

void enable_signal(void) {
    gpio_set(PORT_LO_CE, PIN_LO_CE);
    gpio_set(PORT_SOURCE_CE, PIN_SOURCE_CE);
    gpio_set(PORT_SOURCE_RF_ENABLE, PIN_SOURCE_RF_ENABLE);
    gpio_set(PORT_LO_RF_ENABLE, PIN_LO_RF_ENABLE);
}

void disable_signal(void) {
    gpio_clear(PORT_LO_CE, PIN_LO_CE);
    gpio_clear(PORT_SOURCE_CE, PIN_SOURCE_CE);
    gpio_clear(PORT_SOURCE_RF_ENABLE, PIN_SOURCE_RF_ENABLE);
    gpio_clear(PORT_LO_RF_ENABLE, PIN_LO_RF_ENABLE);
}

void rf_disable(void) {
    disable_pa();
    disable_signal();
}

void rf_enable(void) {
    enable_pa();
    enable_signal();
}

void set_filter(uint8_t n) {
    /* Source filter bank */
    if (n == 1) {
        gpio_clear(PORT_FILTER2, PIN_FILTER2);
        gpio_clear(PORT_FILTER3, PIN_FILTER3);
        gpio_clear(PORT_FILTER4, PIN_FILTER4);
        gpio_set(PORT_FILTER1, PIN_FILTER1);
    } else if (n == 2) {
        gpio_clear(PORT_FILTER1, PIN_FILTER1);
        gpio_clear(PORT_FILTER3, PIN_FILTER3);
        gpio_clear(PORT_FILTER4, PIN_FILTER4);
        gpio_set(PORT_FILTER2, PIN_FILTER2);
    } else if (n == 3) {
        gpio_clear(PORT_FILTER1, PIN_FILTER1);
        gpio_clear(PORT_FILTER2, PIN_FILTER2);
        gpio_clear(PORT_FILTER4, PIN_FILTER4);
        gpio_set(PORT_FILTER3, PIN_FILTER3);
    } else if (n == 4) {
        gpio_clear(PORT_FILTER1, PIN_FILTER1);
        gpio_clear(PORT_FILTER2, PIN_FILTER2);
        gpio_clear(PORT_FILTER3, PIN_FILTER3);
        gpio_set(PORT_FILTER4, PIN_FILTER4);
    }
}

void set_tx_port(uint8_t n) {
    if (n == 0) {
        gpio_set(PORT_TX_SW_CTRL, PIN_TX_SW_CTRL);
    } else {
        gpio_clear(PORT_TX_SW_CTRL, PIN_TX_SW_CTRL);
    }
}
void set_rx_channel(uint8_t ch) {
    /* Mixer input switch */
    if (ch == 0) {
        gpio_clear(PORT_RX_V1, PIN_RX_V1);
        gpio_clear(PORT_RX_V2, PIN_RX_V2);
    } else if (ch == 1) {
        gpio_set(PORT_RX_V1, PIN_RX_V1);
        gpio_clear(PORT_RX_V2, PIN_RX_V2);
    } else if (ch == 2) {
        gpio_clear(PORT_RX_V1, PIN_RX_V1);
        gpio_set(PORT_RX_V2, PIN_RX_V2);
    } else if (ch == 3) {
        gpio_set(PORT_RX_V1, PIN_RX_V1);
        gpio_set(PORT_RX_V2, PIN_RX_V2);
    }
}

void spi_write_register(uint32_t data) {
    uint16_t transfer[2] = {data >> 16, data & 0x0000FFFF};

	ssp_transfer(SSP1_NUM, transfer[0]);
	ssp_transfer(SSP1_NUM, transfer[1]);
}

uint32_t spi_read_register(void) {
    uint32_t read = 0;

	read = (uint32_t)((uint16_t)ssp_transfer(SSP1_NUM, 0xFFFF) << 16);
	read |= (uint16_t)ssp_transfer(SSP1_NUM, 0xFFFF);
    return read;
}

void lo_write_register(uint32_t data) {
    gpio_clear(PORT_LO_LE, PIN_LO_LE);
    spi_write_register(data);
    gpio_set(PORT_LO_LE, PIN_LO_LE);
}

void source_write_register(uint32_t data) {
    gpio_clear(PORT_SOURCE_LE, PIN_SOURCE_LE);
    spi_write_register(data);
    gpio_set(PORT_SOURCE_LE, PIN_SOURCE_LE);
}

void wait_for_lock() {
    //Assumes that PORT_SOURCE_LD == PORT_LO_LD
    int i = 0;
    while ( i < 1000) {
        if (!gpio_get(PORT_SOURCE_LD, PIN_SOURCE_LD | PIN_LO_LD)) {
            i = 0;
        } else {
            i++;
        }
    };
}

/* FIXME: Send only 8 bits */
void att_write_register(uint8_t data) {
    gpio_clear(PORT_ATT_LE, PIN_ATT_LE);
	ssp_transfer(SSP1_NUM, (uint16_t)data);
    gpio_set(PORT_ATT_LE, PIN_ATT_LE);
    volatile int i;
    for(i=0; i<100;i++) {
         __asm__("nop");
    }
    gpio_clear(PORT_ATT_LE, PIN_ATT_LE);
}

void ritimer_isr(void) {
    ENABLE_RITIMER(); //Clear interrupt
    if (isr_count >= isr_len_channels) {
        DISABLE_RITIMER();
        isr_done = 1;
    } else {
        set_rx_channel(isr_channels[isr_count++]);
    }
}

void sample(uint32_t ports) {
//MCU clock*(samples/f_fsample)/channels)
#define CH_DELAY (120e6*(16384/9.6e6)/4)
    isr_count = 0;
    isr_done = 0;
    RITIMER_COUNTER = 0;

    if (ports == 1) {
        RITIMER_COMPVAL = 2*CH_DELAY;
        isr_len_channels = 1;
        isr_channels[0] = 2;
        set_rx_channel(0);
    }
    if (ports == 2) {
        RITIMER_COMPVAL = 2*CH_DELAY;
        isr_len_channels = 1;
        isr_channels[0] = 3;
        set_rx_channel(1);
    }
    if (ports == 3) {
        RITIMER_COMPVAL = CH_DELAY;
        isr_len_channels = 3;
        isr_channels[0] = 1;
        isr_channels[1] = 2;
        isr_channels[2] = 3;
        set_rx_channel(0);
    }

    wait_for_lock();
    ADCHS_restart_dma();
    ENABLE_RITIMER();
    while(isr_done == 0) {
        fill_rng();
    }
}
