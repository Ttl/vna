#ifndef _RF_PATH_H
#define _RF_PATH_H


void enable_pa(void);
void disable_pa(void);

void enable_mixer(void);
void disable_mixer(void);

void enable_signal(void);
void disable_signal(void);

void rf_disable(void);
void rf_enable(void);

void set_tx_port(uint8_t n);
void set_filter(uint8_t n);
void set_rx_channel(uint8_t ch);

void lo_write_register(uint32_t data);
void source_write_register(uint32_t data);
void att_write_register(uint8_t data);

void wait_for_lock();
void wait_for_source_lock();
void wait_for_lo_lock();

void sample(uint32_t ports);

#endif
