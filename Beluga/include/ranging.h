//
// Created by tom on 7/9/24.
//

#ifndef BELUGA_RANGING_H
#define BELUGA_RANGING_H

#include <deca_regs.h>
#include <stdbool.h>
#include <stdint.h>

enum pgdelay_ch {
    ch1 = TC_PGDELAY_CH1,
    ch2 = TC_PGDELAY_CH2,
    ch3 = TC_PGDELAY_CH3,
    ch4 = TC_PGDELAY_CH4,
    ch5 = TC_PGDELAY_CH5,
    ch7 = TC_PGDELAY_CH7,
};

void set_twr_mode(bool value);
bool get_twr_mode(void);
void set_rate(uint32_t rate);
uint32_t get_rate(void);
bool set_uwb_channel(uint32_t channel);
void set_tx_power(bool power_max);

void init_uwb(void);

void init_ranging_thread(void);
void init_responder_thread(void);

#endif // BELUGA_RANGING_H
