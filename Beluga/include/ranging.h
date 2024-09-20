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

enum uwb_datarate { UWB_DR_6M8, UWB_DR_850K, UWB_DR_110K };

enum uwb_preamble_length {
    UWB_PRL_64 = DWT_PLEN_64,
    UWB_PRL_128 = DWT_PLEN_128,
    UWB_PRL_256 = DWT_PLEN_256,
    UWB_PRL_512 = DWT_PLEN_512,
    UWB_PRL_1024 = DWT_PLEN_1024,
    UWB_PRL_2048 = DWT_PLEN_2048,
    UWB_PRL_4096 = DWT_PLEN_4096,
    UWB_PRL_ERROR
};

enum uwb_pulse_rate { UWB_PR_16M, UWB_PR_64M };

bool set_uwb_data_rate(enum uwb_datarate rate,
                       enum uwb_preamble_length *new_preamble);
bool set_uwb_preamble_length(enum uwb_preamble_length length);
bool set_pulse_rate(enum uwb_pulse_rate rate);

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
